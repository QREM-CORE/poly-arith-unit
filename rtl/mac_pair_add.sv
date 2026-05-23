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
 *   Keeping this block combinational makes it easier to reuse from the
 *   new mac_row_accum scratchpad-based accumulator.
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

    logic [12:0] sum0_raw, sum1_raw;

    assign sum0_raw = acc0_i + cwm0_i;
    assign sum1_raw = acc1_i + cwm1_i;

    assign sum0_o = (sum0_raw >= 13'(Q)) ? (sum0_raw - 13'(Q)) : sum0_raw[11:0];
    assign sum1_o = (sum1_raw >= 13'(Q)) ? (sum1_raw - 13'(Q)) : sum1_raw[11:0];

endmodule
