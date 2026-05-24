/*
 * Module Name: mod_sub
 * Author(s): Jessica Buentipo, Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Sequential Modular Subtraction: (A - B) mod 3329.
 *
 * Latency: 1 Clock Cycle
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_sub(
    input   logic   clk,
    input   logic   rst,

    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,

    // Output: 12-bit result (0 to 3328)
    output  coeff_t result_o
);

    (* keep = "true" *) logic [12:0] diff;
    logic [11:0] result_comb;

    always_comb begin
        diff = {1'b0, op1_i} - {1'b0, op2_i};

        if (diff[12]) begin // negative, op1_i < op2_i
            result_comb = diff[11:0] + 12'd3329;
        end else begin
            result_comb = diff[11:0];
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
