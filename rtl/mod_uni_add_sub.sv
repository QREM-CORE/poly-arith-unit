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

    logic [13:0] sum;
    logic [13:0] sum_minus_q;
    logic [11:0] add_res;

    logic [13:0] diff;
    logic [13:0] diff_plus_q;
    logic [11:0] sub_res;

    logic [11:0] result_comb;

    always_comb begin
        // Add Path
        sum = {2'b0, op1_i} + {2'b0, op2_i};
        sum_minus_q = {2'b0, op1_i} + {2'b0, op2_i} - 14'(Q);
        add_res = sum_minus_q[13] ? sum[11:0] : sum_minus_q[11:0];

        // Sub Path
        diff = {2'b0, op1_i} - {2'b0, op2_i};
        diff_plus_q = {2'b0, op1_i} - {2'b0, op2_i} + 14'(Q);
        sub_res = diff[13] ? diff_plus_q[11:0] : diff[11:0];

        // Final Mux
        result_comb = is_sub_i ? sub_res : add_res;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            result_o <= '0;
        end else begin
            result_o <= result_comb;
        end
    end

endmodule
