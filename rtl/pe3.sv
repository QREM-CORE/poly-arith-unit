/*
 * Module Name: pe3 (Processing Element 3)
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
 * Notable Architectural Differences from PE0:
 * 1. Twiddle Factor Injection: PE3 includes an additional input port (`tf_omega_4_i`)
 * to accept the specific complex root of unity required for the Mixed-Radix NTT.
 * 2. Dual-Stage Operand Routing: The multiplier's operand 1 path utilizes a two-stage
 * multiplexing scheme:
 * - Stage 1 ("The Diagram Mux" via ctrl_i[1]): Selects between the standard
 * base weight (`w3_i`) and the twiddle factor (`tf_omega_4_i`).
 * - Stage 2 ("The Timing Mux" via ctrl_i[0]): Dynamically selects between the
 * immediate Cycle 0 value (for NTT) and a 1-cycle delayed value (for INTT)
 * to ensure precise data alignment with the delayed subtractor output. This
 * creates a "Smart PE" that hides INTT pipeline slip from the top-level controller.
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
 * | PE_MODE_CWM      | Coordinate-Wise Mult       | A + B*W       | A - B*W       | 4 CCs   |
 * | PE_MODE_CODECO   | Compression / Decomp       | A             | B*W           | 3 CCs   |
 * | PE_MODE_ADDSUB   | Point-wise Add/Sub         | A + B         | A - B         | 1 CC    |
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pe3 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a3_i,
    input   coeff_t         b3_i,
    input   coeff_t         w3_i,
    input   coeff_t         tf_omega_4_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u3_o,
    output  coeff_t         v3_o,
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

    // -------- Delay 1 Multiplier Operand 1 Register --------
    coeff_t delay_1_w_data_i;
    coeff_t delay_1_w_data_o;

    // -------- Delay 3 Register --------
    coeff_t delay_3_data_i;
    coeff_t delay_3_data_o;

    // -------- Delay 1 Addition Output Register --------
    coeff_t delay_1_add_data_i;
    coeff_t delay_1_add_data_o;

    // -------- Delay 1 Subtraction Output Register --------
    coeff_t delay_1_sub_data_i;
    coeff_t delay_1_sub_data_o;

    // ============= Arithmetic Module Wires =============
    // -------- Modular Adder Logic --------
    // Inputs
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

    // -------- Modular Multiplier Logic --------
    // Inputs
    coeff_t mod_mul_op1_i;
    coeff_t mod_mul_op2_i;
    // Outputs
    coeff_t mod_mul_result_o;

    // -------- Modular Divider2 Logic --------
    coeff_t mod_div_by_2_op_i;
    coeff_t mod_div_by_2_op_o;

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

    // -------- Delay 1 Multiplier Operand 1 Register --------
    // Delays the selected W/TF by 1 cycle for INTT alignment
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w_data_i),
        .data_o(delay_1_w_data_o)
    );
    // "The Diagram Mux": Select between TF and W3 based on ctrl[1]
    assign delay_1_w_data_i = ctrl_i[1] ? tf_omega_4_i : w3_i;

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
    assign delay_3_data_i = ctrl_i[0] ? mod_div_by_2_op_o : a3_i;

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

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = ctrl_i[0] ? a3_i : delay_3_data_o;
    assign mod_add_op2_i    = ctrl_i[0] ? b3_i : mod_mul_result_o;

    // -------- Modular Subtractor Instantiation --------
    mod_sub u_mod_sub (
        .op1_i      (mod_sub_op1_i),
        .op2_i      (mod_sub_op2_i),

        .result_o   (mod_sub_result_o)
    );
    assign mod_sub_op1_i    = ctrl_i[0] ? a3_i : delay_3_data_o;
    assign mod_sub_op2_i    = ctrl_i[0] ? b3_i : mod_mul_result_o;

    // -------- Modular Multiplier Instantiation --------
    mod_mul u_mod_mul (
        .clk        (clk),
        .rst        (rst),
        .valid_i    (),

        .op1_i      (mod_mul_op1_i),
        .op2_i      (mod_mul_op2_i),

        .result_o   (mod_mul_result_o),
        .valid_o     ()
    );
    // modmul op1 modified to account for twiddle factor delay for INTT
    assign mod_mul_op1_i    =   ctrl_i[0] ?
                                delay_1_w_data_o :
                                delay_1_w_data_i;
    assign mod_mul_op2_i    = ctrl_i[0] ? delay_1_sub_data_o : b3_i;

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o)
    );
    assign mod_div_by_2_op_i = delay_1_add_data_o;

    // =========================================================================
    // PE Outputs
    // =========================================================================

    assign u3_o     = ctrl_i[2] ? delay_3_data_o : delay_1_add_data_o;
    assign v3_o     = ctrl_i[2] ? mod_mul_result_o : delay_1_sub_data_o;

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
