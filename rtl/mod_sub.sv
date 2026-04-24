/*
 * Module Name: mod_sub
 * Author(s): Jessica Buentipo, Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Combinational Modular Subtraction: (A - B) mod 3329.
 *
 * Latency: 0 Clock Cycles (Combinational)
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_sub(
    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,

    // Output: 12-bit result (0 to 3328)
    output  coeff_t result_o
);

    // Using 13 bits to capture the sign/underflow of the subtraction
    logic [12:0] diff;
    logic [12:0] diff_plus_q;

    always_comb begin
        // 1. Raw Subtraction
        // If op1 < op2, this will underflow (wrap around in 13-bit unsigned arithmetic).
        // The MSB (bit 12) effectively acts as the sign bit in 2's complement view,
        // or indicates a borrow in unsigned view.
        diff = op1_i - op2_i;

        // 2. Prepare Corrected Value
        // If the result was negative (underflow), we add Q to bring it back to [0, Q-1].
        // We explicitly cast Q to 13 bits to match the width.
        diff_plus_q = diff + 13'(Q);

        // 3. Output Selection
        // If bit 12 is 1, it means the result was negative (underflow occurred).
        // Therefore, we select the corrected value (diff + Q).
        if (diff[12]) begin
            result_o = diff_plus_q[11:0];
        end else begin
            result_o = diff[11:0];
        end
    end

endmodule
