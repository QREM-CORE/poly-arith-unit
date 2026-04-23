/*
 * Module Name: mac_pair_add
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Description:
 *   Small pure-combinational helper used by the PAU row accumulator.
 *
 *   The original mac_adder mixed together:
 *     - modular addition
 *     - first-term bypass semantics
 *     - output registration
 *
 *   This helper intentionally keeps only the arithmetic:
 *     sum_lane = (acc_lane + cwm_lane) mod q
 *
 *   Keeping this block combinational makes it easier to reuse from:
 *     1. the legacy registered mac_adder wrapper, and
 *     2. the new mac_row_accum scratchpad-based accumulator.
 */

`timescale 1ns / 1ps

import poly_arith_pkg::*;

module mac_pair_add (
    input  coeff_t acc0_i,
    input  coeff_t acc1_i,
    input  coeff_t cwm0_i,
    input  coeff_t cwm1_i,
    output coeff_t sum0_o,
    output coeff_t sum1_o
);

    // Two cheap modular adders are enough because the PAU's current CWM
    // datapath emits one coefficient pair per cycle.
    mod_add u_add0 (
        .op1_i    (acc0_i),
        .op2_i    (cwm0_i),
        .result_o (sum0_o)
    );

    mod_add u_add1 (
        .op1_i    (acc1_i),
        .op2_i    (cwm1_i),
        .result_o (sum1_o)
    );

endmodule
