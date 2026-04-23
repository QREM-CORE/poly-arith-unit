/*
 * Module Name: mac_adder
 * Author(s): Salwan Aldhahab, Quardin Lyttle
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)"
 * from H. Jung, Q. D. Truong and H. Lee,
 * "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"
 * IEEE Open Journal of Circuits and Systems, 2025.
 *
 * Description:
 *   Legacy registered wrapper around a 2-lane modular add datapath.
 *
 * Why this file changed:
 *   The original control used `init_i`, where the real accumulation happened
 *   when the signal was low. That was functionally fine but mentally awkward.
 *   This branch switches to a positive `first_term_i` naming convention:
 *
 *     first_term_i = 1 -> pass the first CWM result through unchanged
 *     first_term_i = 0 -> accumulate new_cwm + old_partial_sum
 *
 *   A scratchpad-backed accumulator (mac_row_accum.sv) is now the preferred
 *   architecture for row-wise matrix operations. This module remains useful as
 *   a small timing-clean wrapper and for existing directed tests.
 */

`timescale 1ns / 1ps

import poly_arith_pkg::*;

module mac_adder (
    input   logic           clk,
    input   logic           rst,

    // Control
    input   logic           first_term_i, // 1 = overwrite/seed, 0 = accumulate
    input   logic           valid_i,

    // New CWM result pair
    input   coeff_t         a0_i,
    input   coeff_t         a1_i,

    // Previous partial sum pair
    input   coeff_t         b0_i,
    input   coeff_t         b1_i,

    // Registered accumulated output pair
    output  coeff_t         z0_o,
    output  coeff_t         z1_o,
    output  logic           valid_o
);

    coeff_t pair_sum_0;
    coeff_t pair_sum_1;
    coeff_t mac_result_0;
    coeff_t mac_result_1;

    // Reuse the pure combinational pair-add helper so the arithmetic is shared
    // with the new scratchpad-backed row accumulator.
    mac_pair_add u_pair_add (
        .acc0_i (b0_i),
        .acc1_i (b1_i),
        .cwm0_i (a0_i),
        .cwm1_i (a1_i),
        .sum0_o (pair_sum_0),
        .sum1_o (pair_sum_1)
    );

    // Positive semantics are much easier for the controller to reason about.
    assign mac_result_0 = first_term_i ? a0_i : pair_sum_0;
    assign mac_result_1 = first_term_i ? a1_i : pair_sum_1;

    // Registered output keeps the wrapper behavior from the original module.
    always_ff @(posedge clk) begin
        if (rst) begin
            z0_o    <= '0;
            z1_o    <= '0;
            valid_o <= 1'b0;
        end else begin
            z0_o    <= mac_result_0;
            z1_o    <= mac_result_1;
            valid_o <= valid_i;
        end
    end

endmodule
