/*
 * Module Name: pe1 (Processing Element 1)
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
 * Processing Element 1 (PE1) acts as the complementary arithmetic unit to PE0
 * in the Polynomial Arithmetic Unit (PAU). Unlike the other PEs,
 * PE1 does not contain any modular multipliers. Instead, it features a modular
 * adder and a unified modular adder/subtractor to handle complex addition
 * cross-terms required in higher-radix butterflies and Karatsuba multiplication.
 *
 * To synchronize with the multiplier-heavy PEs, PE1 includes a 3-cycle delay
 * register array (`delay_3`) that pads its operations to match the 4-cycle
 * global latency of the NTT/INTT/CWM modes.
 *
 * Important Usage Note:
 * The `ctrl_i` mode select signal is combinational to all internal MUXes. The Top-Level
 * AU Controller MUST hold `ctrl_i` steady for the entire duration of an operation and
 * insert a pipeline flush (wait for `valid_o` to clear) before switching to a new mode
 * to prevent structural hazards (Ghost Pulses).
 *
 * Supported Modes & Latency:
 * | Mode (ctrl_i)    | Operation                  | Latency |
 * |------------------|----------------------------|---------|
 * | PE_MODE_NTT      | Number Theoretic Transform | 4 CCs   |
 * | PE_MODE_INTT     | Inverse NTT                | 4 CCs   |
 * | PE_MODE_CWM      | Coordinate-Wise Mult       | 4 CCs   |
 * | PE_MODE_COMP     | Compression                | 4 CCs   |
 * | PE_MODE_DECOMP   | Decompression              | 1 CC    |
 * | PE_MODE_ADDSUB   | Point-wise Add/Sub         | 1 CC    |
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pe1 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a1_i,
    input   coeff_t         b1_i,
    input   coeff_t         c0_i,
    input   coeff_t         c1_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u1_o,
    output  coeff_t         v1_o,
    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // ============= Delay Register Wires =============

    // -------- Valid Propagation Registers --------
    logic delay_4_valid_data_i, delay_4_valid_data_o;
    logic delay_1_valid_data_i, delay_1_valid_data_o;

    // (delay_1_add and delay_1_uni_add_sub registers were removed via retiming)

    // -------- Stage 2 Padding Registers --------
    coeff_t delay_3_u1_data_i, delay_3_u1_data_o;
    coeff_t delay_3_v1_data_i, delay_3_v1_data_o;

    // ============= Arithmetic Module Wires =============
    coeff_t mod_add_op1_i, mod_add_op2_i, mod_add_result_o;
    coeff_t mod_div_by_2_op_i, mod_div_by_2_op_o;
    coeff_t mod_uni_add_sub_op1_i, mod_uni_add_sub_op2_i, mod_uni_add_sub_result_o;
    logic   mod_uni_add_sub_is_sub_i;

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================

    // -------- 4-Cycle Valid Propagation Register --------
    delay_n #(
        .DWIDTH(1),
        .DEPTH(4)
    ) u_delay_4_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_4_valid_data_i),
        .data_o(delay_4_valid_data_o)
    );
    // Latency is strictly determined by ctrl_i[3]
    assign delay_4_valid_data_i = (ctrl_i[3] == 1'b1) ? valid_i : 1'b0;

    // -------- 1-Cycle Valid Propagation Register --------
    delay_n #(
        .DWIDTH(1),
        .DEPTH(1)
    ) u_delay_1_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_valid_data_i),
        .data_o(delay_1_valid_data_o)
    );
    // Latency is strictly determined by ctrl_i[3]
    assign delay_1_valid_data_i = (ctrl_i[3] == 1'b0) ? valid_i : 1'b0;

    // (delay_1_add and delay_1_uni_add_sub registers were removed via retiming)

    // -------- Delay 3 U1 Padding Register --------
    delay_n #(
        .DWIDTH(COEFF_WIDTH),
        .DEPTH(3)
    ) u_delay_3_u1 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_u1_data_i),
        .data_o(delay_3_u1_data_o)
    );
    assign delay_3_u1_data_i = ctrl_i[2] ? mod_div_by_2_op_o : mod_add_result_o;

    // -------- Delay 3 V1 Padding Register --------
    delay_n #(
        .DWIDTH(COEFF_WIDTH),
        .DEPTH(3)
    ) u_delay_3_v1 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_v1_data_i),
        .data_o(delay_3_v1_data_o)
    );
    assign delay_3_v1_data_i = mod_uni_add_sub_result_o;

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .clk        (clk),
        .rst        (rst),
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = a1_i;
    assign mod_add_op2_i    = ctrl_i[1] ? b1_i : c1_i;

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o)
    );
    assign mod_div_by_2_op_i = mod_add_result_o;

    // -------- Modular Unified Add/Sub Instantiation --------
    mod_uni_add_sub u_uni_add_sub(
        .clk        (clk),
        .rst        (rst),
        .op1_i      (mod_uni_add_sub_op1_i),
        .op2_i      (mod_uni_add_sub_op2_i),
        .is_sub_i   (mod_uni_add_sub_is_sub_i),
        .result_o   (mod_uni_add_sub_result_o)
    );
    assign mod_uni_add_sub_op1_i = ctrl_i[1] ? a1_i : c0_i;
    assign mod_uni_add_sub_op2_i = b1_i;
    assign mod_uni_add_sub_is_sub_i = ctrl_i[1];

    // =========================================================================
    // PE Outputs
    // =========================================================================

    // Bypass routes must use registered output
    assign u1_o = ctrl_i[3] ? delay_3_u1_data_o : mod_add_result_o;
    assign v1_o = ctrl_i[3] ? delay_3_v1_data_o : mod_uni_add_sub_result_o;

    // Output Valid MUX strictly controlled by ctrl_i[3]
    always_comb begin
        if (ctrl_i[3] == 1'b1) begin
            valid_o = delay_4_valid_data_o;
        end else begin
            valid_o = delay_1_valid_data_o;
        end
    end

endmodule
