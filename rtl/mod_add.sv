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

    logic [12:0] sum;
    logic [12:0] sum_plus_c;
    logic [11:0] result_comb;

    always_comb begin
        sum = {1'b0, op1_i} + {1'b0, op2_i};
        // 767 is 4096 - 3329. If sum >= 3329, sum_plus_c >= 4096 (bit 12 is 1).
        sum_plus_c = {1'b0, op1_i} + {1'b0, op2_i} + 13'd767;

        if (sum_plus_c[12]) begin // sum >= Q
            result_comb = sum_plus_c[11:0];
        end else begin
            result_comb = sum[11:0];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            result_o <= '0;
        end else begin
            result_o <= result_comb;
        end
    end

endmodule
