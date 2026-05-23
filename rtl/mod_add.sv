/*
 * Module Name: mod_add
 * Author(s): Jessica Buentipo, Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Sequential Modular Addition: (A + B) mod 3329.
 *
 * Latency: 1 Clock Cycle
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_add(
    input   logic   clk,
    input   logic   rst,

    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,

    // Output: 12-bit result (0 to 3328)
    output  coeff_t result_o
);

    // Internal signals for calculation
    // 13 bits required to capture overflow (max sum = 6656)
    logic [12:0] sum;
    logic [12:0] sum_q;
    logic [12:0] sum_minus_q;

    always_comb begin
        // 1. Raw Addition
        sum = op1_i + op2_i;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            sum_q <= '0;
        end else begin
            sum_q <= sum;
        end
    end

    always_comb begin
        // 2. Prepare Reduced Value
        // Explicitly cast Q to 13 bits to ensure correct subtraction width
        sum_minus_q = sum_q - 13'(Q);

        // 3. Modular Reduction Selection
        // If sum_q >= 3329, we must subtract 3329.
        if (sum_q >= 13'(Q)) begin
            result_o = sum_minus_q[11:0];
        end else begin
            result_o = sum_q[11:0];
        end
    end

endmodule
