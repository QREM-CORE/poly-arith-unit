/*
 * Module Name: pe2 (Processing Element 2)
 * Author(s): Jessica Buentipo, Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * This module implements a highly optimized, multi-mode Processing Element configured
 * specifically for operations requiring dual modular multiplications, such as the
 * Coordinate-Wise Multiplication (Basecase Multiplication) over the polynomial ring.
 * It is a core component of the Arithmetic Unit (AU) used for ML-KEM.
 *
 * Notable Architectural Differences from PE0/PE3:
 * 1. Dual Multipliers & Weights: Incorporates two modular multipliers (`mod_mul_1`,
 * `mod_mul_2`) and two separate weight/twiddle factor inputs (`w1_i`, `w2_i`) to
 * process parallel multiplications simultaneously.
 * 2. Cross-Term Output (`m_o`): Features a dedicated third data output (`m_o`) and
 * an associated valid flag (`valid_m_o`). In CWM mode, this outputs the addition
 * of the two parallel products (e.g., A*W1 + B*W2), which is necessary for
 * resolving the X^2 - zeta modulo reduction in the basecase polynomial multiplication.
 * 3. No Modular Divider: Unlike PE0, this unit omits the `mod_div_by_2` hardware,
 * as INTT scaling is likely handled elsewhere or the dual-multiplier path routes
 * differently for this specific PE.
 *
 * Important Usage Note:
 * The `ctrl_i` mode select signal is combinational to all internal MUXes. The Top-Level
 * AU Controller MUST hold `ctrl_i` steady for the entire duration of an operation and
 * insert a pipeline flush (wait for `valid_o` to clear) before switching to a new mode
 * to prevent structural hazards (Ghost Pulses).
 *
 * Supported Modes & Latency:
 * | Mode (ctrl_i)    | Operation                  | U Out       | V Out       | M Out         | Latency (U,V / M) |
 * |------------------|----------------------------|-------------|-------------|---------------|-------------------|
 * | PE_MODE_NTT      | Number Theoretic Transform | A*W1 + B*W2 | A*W1 - B*W2 | -             | 4 CCs / -         |
 * | PE_MODE_INTT     | Inverse NTT                | (A+B)*W1    | (A-B)*W2    | -             | 4 CCs / -         |
 * | PE_MODE_CWM      | Coordinate-Wise Mult       | A*W1        | B*W2        | A*W1 + B*W2   | 3 CCs / 4 CCs     |
 * | PE_MODE_COMP     | Compression                | A*W1        | B*W2        | -             | 3 CCs / -         |
 * | PE_MODE_DECOMP   | Decompression              | A*W1        | B*W2        | -             | 3 CCs / -         |
 * | PE_MODE_ADDSUB   | Point-wise Add/Sub         | A + B       | A - B       | -             | 1 CC  / -         |
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pe2 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a2_i,
    input   coeff_t         b2_i,
    input   coeff_t         w1_i,
    input   coeff_t         w2_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u2_o,
    output  coeff_t         v2_o,
    output  logic           valid_o,

    output  coeff_t         m_o,
    output  logic           valid_m_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // ============= Delay Register Wires =============

    // -------- Delay 4 Valid Register (for NTT and INTT) --------
    logic delay_4_valid_data_i;
    logic delay_4_valid_data_o;

    // -------- Delay 3 Valid Register (for CWM and CO/DECO) --------
    logic delay_3_valid_data_i;
    logic delay_3_valid_data_o;

    // -------- Delay 1 Valid Register (for ADD/SUB Mode) --------
    logic delay_1_valid_data_i;
    logic delay_1_valid_data_o;

    // -------- W1 Input Delay 1 Register --------
    coeff_t delay_1_w1_data_i;
    coeff_t delay_1_w1_data_o;

    // -------- W2 Input Delay 1 Register --------
    coeff_t delay_1_w2_data_i;
    coeff_t delay_1_w2_data_o;

    // -------- Delay 1 Addition Output Register --------
    // Inputs
    coeff_t delay_1_add_data_i;
    // Outputs
    coeff_t delay_1_add_data_o;


    // -------- Delay 1 Subtraction Output Register --------
    // Inputs
    coeff_t delay_1_sub_data_i;
    // Outputs
    coeff_t delay_1_sub_data_o;

    // ============= Arithmetic Module Wires =============

    // -------- Modular Multiplier 1 Logic --------
    // Inputs
    coeff_t mod_mul_1_op1_i;
    coeff_t mod_mul_1_op2_i;
    // Outputs
    coeff_t mod_mul_1_result_o;

    // -------- Modular Multiplier 2 Logic --------
    coeff_t mod_mul_2_op1_i;
    coeff_t mod_mul_2_op2_i;
    // Outputs
    coeff_t mod_mul_2_result_o;

    // -------- Modular Adder Logic --------
    coeff_t mod_add_op1_i;
    coeff_t mod_add_op2_i;
    // Outputs
    coeff_t mod_add_result_o;

    // -------- Modular Subtractor Logic --------
    // Inputs
    coeff_t mod_sub_op1_i;
    coeff_t mod_sub_op2_i;
    // Outputs
    coeff_t mod_sub_result_o;

    // U2 input mux
    coeff_t u2_mux_i;
    coeff_t v2_mux_i;

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================

    // 4-Cycle Pipeline (NTT, INTT)
    delay_n #(
        .DWIDTH(1),
        .DEPTH(4)
    ) u_delay_4_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_4_valid_data_i),
        .data_o(delay_4_valid_data_o)
    );
    assign delay_4_valid_data_i = ( ctrl_i == PE_MODE_NTT  ||
                                    ctrl_i == PE_MODE_INTT ||
                                    ctrl_i == PE_MODE_CWM) ? valid_i : 1'b0;

    // 3-Cycle Pipeline (CWM, CODECO)
    delay_n #(
        .DWIDTH(1),
        .DEPTH(3)
    ) u_delay_3_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_valid_data_i),
        .data_o(delay_3_valid_data_o)
    );
    assign delay_3_valid_data_i = ( ctrl_i == PE_MODE_CWM  ||
                                    ctrl_i == PE_MODE_COMP ||
                                    ctrl_i == PE_MODE_DECOMP) ? valid_i : 1'b0;

    // 1-Cycle Pipeline (ADDSUB)
    delay_n #(
        .DWIDTH(1),
        .DEPTH(1)
    ) u_delay_1_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_valid_data_i),
        .data_o(delay_1_valid_data_o)
    );
    assign delay_1_valid_data_i = (ctrl_i == PE_MODE_ADDSUB) ? valid_i : 1'b0;

    // -------- W1 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w1 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w1_data_i),
        .data_o(delay_1_w1_data_o)
    );
    assign delay_1_w1_data_i = w1_i;

    // -------- W2 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w2 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w2_data_i),
        .data_o(delay_1_w2_data_o)
    );
    assign delay_1_w2_data_i = w2_i;

    // -------- Delay 1 Addition Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_add (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_add_data_i),
        .data_o(delay_1_add_data_o)
    );
    assign delay_1_add_data_i = mod_add_result_o;

    // -------- Delay 1 Subtraction Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_sub (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_sub_data_i),
        .data_o(delay_1_sub_data_o)
    );
    assign delay_1_sub_data_i = mod_sub_result_o;

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Multiplier 1 Instantiation --------
    mod_mul u_mod_mul_1 (
        .clk           (clk),
        .rst           (rst),
        .valid_i       (),

        .op1_i         (mod_mul_1_op1_i),
        .op2_i         (mod_mul_1_op2_i),

        .result_o      (mod_mul_1_result_o),
        .valid_o       ()
    );

    assign mod_mul_1_op1_i = ctrl_i[0] ? delay_1_add_data_o : a2_i;
    assign mod_mul_1_op2_i = ctrl_i[0] ? delay_1_w1_data_o : w1_i;

    // -------- Modular Multiplier 2 Instantiation --------
    mod_mul u_mod_mul_2 (
        .clk           (clk),
        .rst           (rst),
        .valid_i       (),

        .op1_i         (mod_mul_2_op1_i),
        .op2_i         (mod_mul_2_op2_i),

        .result_o      (mod_mul_2_result_o),
        .valid_o       ()
    );

    assign mod_mul_2_op1_i = ctrl_i[0] ? delay_1_w2_data_o : w2_i;
    assign mod_mul_2_op2_i = ctrl_i[0] ? delay_1_sub_data_o : b2_i;

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = ctrl_i[0] ? a2_i : mod_mul_1_result_o;
    assign mod_add_op2_i    = ctrl_i[0] ? b2_i : mod_mul_2_result_o;

    // -------- Modular Subtractor Instantiation --------
    mod_sub u_mod_sub (
        .op1_i      (mod_sub_op1_i),
        .op2_i      (mod_sub_op2_i),

        .result_o   (mod_sub_result_o)
    );
    assign mod_sub_op1_i    = ctrl_i[0] ? a2_i : mod_mul_1_result_o;
    assign mod_sub_op2_i    = ctrl_i[0] ? b2_i : mod_mul_2_result_o;

    // =========================================================================
    // PE Outputs
    // =========================================================================

    // Data Output
    assign u2_mux_i = ctrl_i[2] ? mod_mul_1_result_o : delay_1_add_data_o;
    assign v2_mux_i = ctrl_i[2] ? mod_mul_2_result_o : delay_1_sub_data_o;

    assign u2_o = ctrl_i[1] ? u2_mux_i : mod_mul_1_result_o;
    assign v2_o = ctrl_i[1] ? v2_mux_i : mod_mul_2_result_o;
    assign m_o = delay_1_add_data_o;

    // Dynamic Valid Output Multiplexer
    always_comb begin
        if (ctrl_i == PE_MODE_ADDSUB) begin
            valid_o = delay_1_valid_data_o;
        end else if (   ctrl_i == PE_MODE_CWM ||
                        ctrl_i == PE_MODE_COMP ||
                        ctrl_i == PE_MODE_DECOMP) begin
            valid_o = delay_3_valid_data_o;
        end else begin
            valid_o = delay_4_valid_data_o;
        end
    end

    // Dedicated Valid Output for the M_O Cross-Term (Strictly 4-cycle latency in CWM)
    assign valid_m_o = (ctrl_i == PE_MODE_CWM) ? delay_4_valid_data_o : 1'b0;

endmodule
