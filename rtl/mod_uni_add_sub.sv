/*
 * Module Name: mod_uni_add_sub
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Sequential Modular Addition or Subtraction: (A +/- B) mod 3329.
 *
 * Latency: 1 Clock Cycle
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_uni_add_sub(
    input   logic   clk,
    input   logic   rst,

    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,
    input   logic   is_sub_i, // Control: 1 = Subtract, 0 = Add

    // Output: 12-bit result
    output  coeff_t result_o
);

    // Internal signals for calculation
    coeff_t         op2_final;
    logic [12:0]    sum_raw;
    logic [12:0]    sum_raw_q;
    logic [12:0]    sum_reduced;

    always_comb begin
        // =====================================================================
        // 1. Operand Preparation (Handle Subtraction)
        // =====================================================================
        // Optimization: A - B is implemented as A + (Q - B).
        // If Adding: Use B.
        // If Subtracting: Use (Q - B).
        if (is_sub_i) begin
            op2_final = 13'(Q) - op2_i;
        end else begin
            op2_final = op2_i;
        end

        // =====================================================================
        // 2. Unified Addition
        // =====================================================================
        // Max possible sum: 3328 + 3329 = 6657 (fits in 13 bits)
        sum_raw = op1_i + op2_final;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            sum_raw_q <= '0;
        end else begin
            sum_raw_q <= sum_raw;
        end
    end

    always_comb begin
        // =====================================================================
        // 3. Modular Reduction (Conditional Subtraction)
        // =====================================================================
        // If the sum exceeds Q, we wrap it back by subtracting Q.
        sum_reduced = sum_raw_q - 13'(Q);

        // Mux to select the correct result
        if (sum_raw_q >= 13'(Q)) begin
            result_o = sum_reduced[11:0];
        end else begin
            result_o = sum_raw_q[11:0];
        end
    end

endmodule
