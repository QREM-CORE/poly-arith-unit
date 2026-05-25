/*
 * Module Name: mac_row_accum
 * Author(s): Quardin Lyttle
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
    input  logic        rst,

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
    coeff_t acc0_wr_data;
    coeff_t acc1_wr_data;

    coeff_t drain0_raw;
    coeff_t drain1_raw;

    // Keep a copy of the most recent scratch write so repeated hits on the
    // same pair index are deterministic even if the inferred memory has
    // ambiguous read-during-write behavior.
    logic       last_wr_valid_q;
    logic [6:0] last_wr_pair_idx_q;
    coeff_t     last_wr_acc0_q;
    coeff_t     last_wr_acc1_q;
    logic       acc_rd_bypass_hit;
    logic       drain_rd_bypass_hit;

    // first_term_i is a 1-cycle "start new CWM row" pulse, not a level that
    // stays high for the whole first source-term sweep. Keep an internal init
    // window active until the first sweep has written every pair slot once.
    logic       init_active_q;
    logic       init_active_n;
    logic       seed_mode;

    integer p;

    logic        acc_fire_q;
    logic        first_term_q;
    logic [6:0]  pair_idx_q;
    coeff_t      cwm0_q;
    coeff_t      cwm1_q;

    always_ff @(posedge clk) begin
        if (rst) begin
            acc_fire_q   <= 1'b0;
            first_term_q <= 1'b0;
            pair_idx_q   <= '0;
            cwm0_q       <= '0;
            cwm1_q       <= '0;
        end else begin
            acc_fire_q   <= acc_fire_i;
            first_term_q <= first_term_i;
            pair_idx_q   <= pair_idx_i;
            cwm0_q       <= cwm0_i;
            cwm1_q       <= cwm1_i;
        end
    end

    // ------------------------------------------------------------
    // Scratch reads
    // ------------------------------------------------------------
    // Write-through bypass guarantees that acc*_old sees the running partial
    // sum for back-to-back accesses to the same pair index.
    assign acc_rd_bypass_hit   = last_wr_valid_q && (last_wr_pair_idx_q == pair_idx_q);
    assign drain_rd_bypass_hit = last_wr_valid_q && (last_wr_pair_idx_q == drain_idx_i);

    assign acc0_old   = acc_rd_bypass_hit   ? last_wr_acc0_q : acc0_mem[pair_idx_q];
    assign acc1_old   = acc_rd_bypass_hit   ? last_wr_acc1_q : acc1_mem[pair_idx_q];
    assign drain0_raw = drain_rd_bypass_hit ? last_wr_acc0_q : acc0_mem[drain_idx_i];
    assign drain1_raw = drain_rd_bypass_hit ? last_wr_acc1_q : acc1_mem[drain_idx_i];

    // ------------------------------------------------------------
    // Accumulate datapath
    // ------------------------------------------------------------
    mac_pair_add u_acc_add (
        .acc0_i (acc0_old),
        .acc1_i (acc1_old),
        .cwm0_i (cwm0_q),
        .cwm1_i (cwm1_q),
        .sum0_o (acc0_sum),
        .sum1_o (acc1_sum)
    );

    // Optional +e_hat fuse during drain. (Moved to output side to break critical path)
    coeff_t drain0_fused_out;
    coeff_t drain1_fused_out;

    coeff_t drain0_raw_q;
    coeff_t drain1_raw_q;
    coeff_t e0_q;
    coeff_t e1_q;
    logic   fuse_e_q;

    mod_add u_drain_add0 (
        .clk      (clk),
        .rst      (rst),
        .op1_i    (drain0_raw),
        .op2_i    (e0_i),
        .result_o (drain0_fused_out)
    );

    mod_add u_drain_add1 (
        .clk      (clk),
        .rst      (rst),
        .op1_i    (drain1_raw),
        .op2_i    (e1_i),
        .result_o (drain1_fused_out)
    );

    // first_term_i is the single-cycle start pulse. Once it arrives, seed the
    // whole first 128-pair sweep by overwriting each slot until pair 127 has
    // been written, then switch back to normal accumulation for later terms.
    assign init_active_n = first_term_q ? 1'b1 :
                           ((acc_fire_q && init_active_q && (pair_idx_q == NUM_PAIRS-1)) ? 1'b0 : init_active_q);
    assign seed_mode     = first_term_q || init_active_q;
    assign acc0_wr_data  = seed_mode ? cwm0_q : acc0_sum;
    assign acc1_wr_data  = seed_mode ? cwm1_q : acc1_sum;

    // ------------------------------------------------------------
    // Scratch update
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            acc0_mem <= '{default: '0};
            acc1_mem <= '{default: '0};
            last_wr_valid_q    <= 1'b0;
            last_wr_pair_idx_q <= '0;
            last_wr_acc0_q     <= '0;
            last_wr_acc1_q     <= '0;
            init_active_q      <= 1'b0;
        end else begin
            last_wr_valid_q <= acc_fire_q;
            init_active_q   <= init_active_n;

            if (acc_fire_q) begin
                acc0_mem[pair_idx_q] <= acc0_wr_data;
                acc1_mem[pair_idx_q] <= acc1_wr_data;
                last_wr_pair_idx_q   <= pair_idx_q;
                last_wr_acc0_q       <= acc0_wr_data;
                last_wr_acc1_q       <= acc1_wr_data;
            end
        end
    end

    // ------------------------------------------------------------
    // Drain output register / handshake
    // ------------------------------------------------------------
    assign drain_accept_o = ~drain_valid_o | drain_ready_i;

    logic   drain_valid_n;
    logic [6:0] drain_pair_idx_n;
    coeff_t drain0_raw_n;
    coeff_t drain1_raw_n;
    coeff_t e0_n;
    coeff_t e1_n;
    logic   fuse_e_n;

    always_comb begin
        drain_valid_n     = drain_valid_o;
        drain_pair_idx_n  = drain_pair_idx_o;
        drain0_raw_n      = drain0_raw_q;
        drain1_raw_n      = drain1_raw_q;
        e0_n              = e0_q;
        e1_n              = e1_q;
        fuse_e_n          = fuse_e_q;

        // If the current drain pair has been consumed and there is no new
        // request in the same cycle, drop valid.
        if (drain_valid_o && drain_ready_i && !drain_req_i) begin
            drain_valid_n = 1'b0;
        end

        if (drain_req_i && drain_accept_o) begin
            drain_valid_n    = 1'b1;
            drain_pair_idx_n = drain_idx_i;
            drain0_raw_n     = drain0_raw;
            drain1_raw_n     = drain1_raw;
            e0_n             = e0_i;
            e1_n             = e1_i;
            fuse_e_n         = fuse_e_i;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            drain_valid_o    <= 1'b0;
            drain_pair_idx_o <= '0;
            drain0_raw_q     <= '0;
            drain1_raw_q     <= '0;
            e0_q             <= '0;
            e1_q             <= '0;
            fuse_e_q         <= '0;
        end else begin
            drain_valid_o    <= drain_valid_n;
            drain_pair_idx_o <= drain_pair_idx_n;
            drain0_raw_q     <= drain0_raw_n;
            drain1_raw_q     <= drain1_raw_n;
            e0_q             <= e0_n;
            e1_q             <= e1_n;
            fuse_e_q         <= fuse_e_n;
        end
    end

    assign drain0_o = fuse_e_q ? drain0_fused_out : drain0_raw_q;
    assign drain1_o = fuse_e_q ? drain1_fused_out : drain1_raw_q;

endmodule
