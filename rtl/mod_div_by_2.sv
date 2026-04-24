/*
 * Module Name: mod_div_by_2
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Implements a combinational modular division by 2 in the ring Z_q (q = 3329).
 * This unit is essential for the Inverse Number Theoretic Transform (INTT),
 * typically used in the Gentleman-Sande butterfly configuration to apply
 * scaling factors at each stage.
 *
 * Mathematical Logic:
 * Calculates: Output = Input * 2^-1 mod 3329
 * - If Input is Even: Output = Input >> 1
 * - If Input is Odd:  Output = (Input + 3329) >> 1
 *
 * Hardware Architecture:
 * - Purely combinational (0 Cycle Latency).
 * - Utilizes a 13-bit adder to capture the potential overflow when adding Q
 * to an odd input, followed by a hard-wired right shift (bit slice).
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_div_by_2 (
    input   coeff_t op_i,
    output  coeff_t op_o
);

    // Q = 3329
    // If op_i is odd, we add Q to make it even before shifting.
    // If op_i is even, we just shift.

    // Implementation trick:
    // We can do the addition in a 13-bit wire to capture the carry, then shift.

    logic [12:0] intermediate; // 13-bits to capture carry

    always_comb begin
        if (op_i[0]) begin
            // Odd case: (Operand + 3329) / 2
            intermediate = op_i + 12'(Q);
        end else begin
            // Even case: Operand / 2
            intermediate = {1'b0, op_i};
        end

        // The division by 2 is just taking the top 12 bits of the 13-bit sum
        // (effectively shifting right by 1)
        op_o = intermediate[12:1];
    end

endmodule
