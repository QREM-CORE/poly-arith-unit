/*
 * Module Name: pe0 (Processing Element 0)
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * This module implements a highly optimized, multi-mode Mixed-Radix Butterfly
 * Processing Element. It is a core component of the Arithmetic Unit (AU) used for
 * polynomial arithmetic in ML-KEM.
 *
 * To minimize area, the module utilizes a "folded" (re-entrant) pipeline architecture.
 * Depending on the operating mode, the internal multiplexers dynamically reorder the
 * data flow between the modular adder/subtractor and the modular multiplier. Parallel
 * delay shift-registers (delay_n) ensure that data arrives at the execution units
 * fully aligned, regardless of the routing path.
 *
 * Important Usage Note:
 * The `ctrl_i` mode select signal is combinational to all internal MUXes. The Top-Level
 * AU Controller MUST hold `ctrl_i` steady for the entire duration of an operation and
 * insert a pipeline flush (wait for `valid_o` to clear) before switching to a new mode
 * to prevent structural hazards (Ghost Pulses).
 *
 * Supported Modes & Latency:
 * | Mode (ctrl_i)    | Operation                  | U Out         | V Out         | Latency |
 * |------------------|----------------------------|---------------|---------------|---------|
 * | PE_MODE_NTT      | Number Theoretic Transform | A + B*W       | A - B*W       | 4 CCs   |
 * | PE_MODE_INTT     | Inverse NTT                | (A + B)/2     | (A - B)*W     | 4 CCs   |
 * | PE_MODE_CWM      | Coordinate-Wise Mult       | A + B*W       | B*W - A       | 4 CCs   |
 * | PE_MODE_CODECO   | Compression / Decomp       | A             | B*W           | 3 CCs   |
 * | PE_MODE_ADDSUB   | Point-wise Add/Sub         | A + B         | A - B         | 1 CC    |
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pe0 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a0_i,
    input   coeff_t         b0_i,
    input   coeff_t         w0_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u0_o,
    output  coeff_t         v0_o,
    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // ============= Delay Register Wires =============
    // -------- Delay 4 Valid Propagation Register --------
    logic   delay_4_valid_data_i;
    logic   delay_4_valid_data_o;

    // -------- Delay 3 Valid Propagation Register --------
    logic   delay_3_valid_data_i;
    logic   delay_3_valid_data_o;

    // -------- Delay 1 Valid Propagation Register --------
    logic   delay_1_valid_data_i;
    logic   delay_1_valid_data_o;

    // -------- Delay 1 W0 Input Register Logic --------
    coeff_t delay_1_w0_data_i;
    coeff_t delay_1_w0_data_o;

    // -------- Delay 3 Register --------
    coeff_t delay_3_data_i;
    coeff_t delay_3_data_o;

    // (delay_1_add and delay_1_sub registers were removed via retiming)

    // -------- Modular Adder Logic --------
    coeff_t mod_add_res_0;
    coeff_t mod_add_res_1;

    // -------- Modular Subtractor Logic --------
    coeff_t mod_sub_res_0;
    coeff_t mod_sub_res_1;
    coeff_t mod_sub_res_cwm;

    // -------- Modular Multiplier Logic --------
    coeff_t mod_mul_op1_i;
    coeff_t mod_mul_op2_i;
    coeff_t mod_mul_result_o;

    // -------- Modular Divider2 Logic --------
    coeff_t mod_div_by_2_op_i;
    coeff_t mod_div_by_2_op_o;

    // -------- Muxed Arithmetic Outputs --------
    coeff_t mod_add_result_o;
    coeff_t mod_sub_result_o;

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================

    // -------- Delay 4 Valid Propagation Register --------
    // For NTT, INTT, and CWM Modes (4-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (4)
    ) u_delay_4_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_4_valid_data_i),
        .data_o(delay_4_valid_data_o)
    );
    // Gate input: Only enter pipe during 4-cycle modes
    assign delay_4_valid_data_i = ( ctrl_i == PE_MODE_NTT ||
                                    ctrl_i == PE_MODE_INTT ||
                                    ctrl_i == PE_MODE_CWM) ? valid_i : 1'b0;

    // -------- Delay 3 Valid Propagation Register --------
    // For Co/Deco Modes (3-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (3)
    ) u_delay_3_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_valid_data_i),
        .data_o(delay_3_valid_data_o)
    );
    // Gate input: Only enter pipe during 3-cycle modes
    assign delay_3_valid_data_i = ( ctrl_i == PE_MODE_COMP ||
                                    ctrl_i == PE_MODE_DECOMP) ? valid_i : 1'b0;

    // -------- Delay 1 Valid Propagation Register --------
    // For ADD/SUB Modes (1-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (1)
    ) u_delay_1_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_valid_data_i),
        .data_o(delay_1_valid_data_o)
    );
    // Gate input: Only enter pipe during 1-cycle modes
    assign delay_1_valid_data_i = (ctrl_i == PE_MODE_ADDSUB) ? valid_i : 1'b0;

    // -------- W0 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w0_data_i),
        .data_o(delay_1_w0_data_o)
    );
    assign delay_1_w0_data_i    = w0_i;

    // -------- Delay 3 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (3)
    ) u_delay_3 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_data_i),
        .data_o(delay_3_data_o)
    );
    assign delay_3_data_i = ctrl_i[0] ? mod_div_by_2_op_o : a0_i;

    // (delay_1_add and delay_1_sub registers were removed via retiming)

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Adder Instantiations --------
    mod_add u_mod_add_0 (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (delay_3_data_o),
        .op2_i      (mod_mul_result_o),
        .result_o   (mod_add_res_0)
    );

    mod_add u_mod_add_1 (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (a0_i),
        .op2_i      (b0_i),
        .result_o   (mod_add_res_1)
    );

    assign mod_add_result_o = ctrl_i[0] ? mod_add_res_1 : mod_add_res_0;

    // -------- Modular Subtractor Instantiations --------
    mod_sub u_mod_sub_0 (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (delay_3_data_o),
        .op2_i      (mod_mul_result_o),
        .result_o   (mod_sub_res_0)
    );

    mod_sub u_mod_sub_1 (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (a0_i),
        .op2_i      (b0_i),
        .result_o   (mod_sub_res_1)
    );

    mod_sub u_mod_sub_cwm (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (mod_mul_result_o),
        .op2_i      (delay_3_data_o),
        .result_o   (mod_sub_res_cwm)
    );

    always_comb begin
        case(ctrl_i)
            PE_MODE_CWM : mod_sub_result_o = mod_sub_res_cwm;
            default     : mod_sub_result_o = ctrl_i[0] ? mod_sub_res_1 : mod_sub_res_0;
        endcase
    end

    // -------- Modular Divider2 Instantiation --------
    mod_mul u_mod_mul (
        .clk        (clk),
        .rst        (rst),
        .valid_i    (),

        .op1_i      (mod_mul_op1_i),
        .op2_i      (mod_mul_op2_i),

        .result_o   (mod_mul_result_o),
        .valid_o     ()
    );
    assign mod_mul_op1_i    = ctrl_i[0] ? delay_1_w0_data_o : w0_i;
    assign mod_mul_op2_i    = ctrl_i[0] ? mod_sub_result_o : b0_i;

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o)
    );
    assign mod_div_by_2_op_i = mod_add_result_o;

    // =========================================================================
    // PE Outputs
    // =========================================================================

    assign u0_o     = ctrl_i[2] ? delay_3_data_o : mod_add_result_o;
    assign v0_o     = ctrl_i[2] ? mod_mul_result_o : mod_sub_result_o;

    always_comb begin
        if (ctrl_i == PE_MODE_ADDSUB) begin
            // ADD/SUB Mode: 1-Cycle Latency
            valid_o = delay_1_valid_data_o;
        end else if (ctrl_i == PE_MODE_COMP || ctrl_i == PE_MODE_DECOMP) begin
            // Co/Deco Mode: 3-Cycle Latency (Bypasses Adder)
            valid_o = delay_3_valid_data_o;
        end else begin
            // NTT, INTT, CWM Modes: 4-Cycle Latency
            valid_o = delay_4_valid_data_o;
        end
    end

endmodule
