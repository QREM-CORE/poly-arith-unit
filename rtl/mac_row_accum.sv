/*
 * Module Name: mac_row_accum
 * Author(s): OpenAI Codex (branch: quardins-attempted-updates)
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Description:
 *   Scratchpad-backed row accumulator for PAU CWM operations.
 *
 * Motivation:
 *   The legacy memory-backed MAC flow re-read and re-wrote the partial
 *   accumulator polynomial on every accumulation step:
 *
 *     read A_ij, read s_j, read old t_acc, write new t_acc
 *
 *   That makes CWM the most memory-hungry PAU phase. This module traps the
 *   high-bandwidth read-modify-write loop inside the PAU by storing one full
 *   polynomial worth of partial sums locally.
 *
 * Operation:
 *   - Accumulate phase:
 *       first_term_i = 1  -> overwrite scratch[pair_idx] with new CWM pair
 *       first_term_i = 0  -> scratch[pair_idx] += new CWM pair
 *
 *   - Drain phase:
 *       the stored scratch pair is emitted, optionally fused with e_hat.
 *       A one-deep output register is used so the drain side can tolerate a
 *       downstream stall without losing the pair currently being written back.
 *
 * Storage:
 *   One polynomial = 256 coefficients = 128 coefficient pairs.
 *   Each pair stores even/odd lanes separately:
 *     acc0_mem[p] -> coefficient 2p
 *     acc1_mem[p] -> coefficient 2p+1
 *
 * Notes:
 *   - Reads are written in a simple style that maps cleanly to LUTRAM /
 *     distributed RAM on FPGA.
 *   - This module assumes the controller will not start row i+1 until row i
 *     has finished draining, because there is only one scratch polynomial.
 */

`timescale 1ns / 1ps

import poly_arith_pkg::*;

module mac_row_accum #(
    parameter int NUM_PAIRS = 128
)(
    input  logic        clk,
    input  logic        rst_n,

    // ------------------------------------------------------------
    // Accumulate interface
    // ------------------------------------------------------------
    input  logic        acc_fire_i,
    input  logic        first_term_i,
    input  logic [6:0]  pair_idx_i,
    input  coeff_t      cwm0_i,
    input  coeff_t      cwm1_i,

    // ------------------------------------------------------------
    // Drain interface
    // ------------------------------------------------------------
    // The controller presents drain_req_i when it wants to push the next
    // pair into the 1-entry drain output register. The request is accepted
    // only when drain_accept_o is high.
    input  logic        drain_req_i,
    input  logic [6:0]  drain_idx_i,
    input  logic        fuse_e_i,
    input  coeff_t      e0_i,
    input  coeff_t      e1_i,
    input  logic        drain_ready_i,

    output logic        drain_accept_o,
    output logic        drain_valid_o,
    output logic [6:0]  drain_pair_idx_o,
    output coeff_t      drain0_o,
    output coeff_t      drain1_o
);

    coeff_t acc0_mem [0:NUM_PAIRS-1];
    coeff_t acc1_mem [0:NUM_PAIRS-1];

    coeff_t acc0_old;
    coeff_t acc1_old;
    coeff_t acc0_sum;
    coeff_t acc1_sum;

    coeff_t drain0_raw;
    coeff_t drain1_raw;
    coeff_t drain0_fused;
    coeff_t drain1_fused;

    logic   drain_valid_n;
    coeff_t drain0_n;
    coeff_t drain1_n;
    logic [6:0] drain_pair_idx_n;

    integer p;

    // ------------------------------------------------------------
    // Scratch reads
    // ------------------------------------------------------------
    assign acc0_old   = acc0_mem[pair_idx_i];
    assign acc1_old   = acc1_mem[pair_idx_i];
    assign drain0_raw = acc0_mem[drain_idx_i];
    assign drain1_raw = acc1_mem[drain_idx_i];

    // ------------------------------------------------------------
    // Accumulate datapath
    // ------------------------------------------------------------
    mac_pair_add u_acc_add (
        .acc0_i (acc0_old),
        .acc1_i (acc1_old),
        .cwm0_i (cwm0_i),
        .cwm1_i (cwm1_i),
        .sum0_o (acc0_sum),
        .sum1_o (acc1_sum)
    );

    // Optional +e_hat fuse during drain.
    mod_add u_drain_add0 (
        .op1_i    (drain0_raw),
        .op2_i    (e0_i),
        .result_o (drain0_fused)
    );

    mod_add u_drain_add1 (
        .op1_i    (drain1_raw),
        .op2_i    (e1_i),
        .result_o (drain1_fused)
    );

    // ------------------------------------------------------------
    // Scratch update
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (p = 0; p < NUM_PAIRS; p++) begin
                acc0_mem[p] <= '0;
                acc1_mem[p] <= '0;
            end
        end else if (acc_fire_i) begin
            // first_term_i means "seed this pair with the first CWM result"
            // rather than adding to a previous partial sum.
            if (first_term_i) begin
                acc0_mem[pair_idx_i] <= cwm0_i;
                acc1_mem[pair_idx_i] <= cwm1_i;
            end else begin
                acc0_mem[pair_idx_i] <= acc0_sum;
                acc1_mem[pair_idx_i] <= acc1_sum;
            end
        end
    end

    // ------------------------------------------------------------
    // Drain output register / handshake
    // ------------------------------------------------------------
    assign drain_accept_o = ~drain_valid_o | drain_ready_i;

    always_comb begin
        drain_valid_n     = drain_valid_o;
        drain_pair_idx_n  = drain_pair_idx_o;
        drain0_n          = drain0_o;
        drain1_n          = drain1_o;

        // If the current drain pair has been consumed and there is no new
        // request in the same cycle, drop valid.
        if (drain_valid_o && drain_ready_i && !drain_req_i) begin
            drain_valid_n = 1'b0;
        end

        if (drain_req_i && drain_accept_o) begin
            drain_valid_n    = 1'b1;
            drain_pair_idx_n = drain_idx_i;
            drain0_n         = fuse_e_i ? drain0_fused : drain0_raw;
            drain1_n         = fuse_e_i ? drain1_fused : drain1_raw;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drain_valid_o    <= 1'b0;
            drain_pair_idx_o <= '0;
            drain0_o         <= '0;
            drain1_o         <= '0;
        end else begin
            drain_valid_o    <= drain_valid_n;
            drain_pair_idx_o <= drain_pair_idx_n;
            drain0_o         <= drain0_n;
            drain1_o         <= drain1_n;
        end
    end

endmodule
