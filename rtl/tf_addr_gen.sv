/*
 * Module Name: tf_addr_gen (Twiddle Factor Address Generator)
 * Author(s): Salwan Aldhahab
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Generates ROM read addresses for the four-ROM twiddle factor store (tf_rom)
 * during NTT, INTT, and CWM operations.
 *
 * Radix-4 passes: simple sequential counter (t++), matching Algorithm 5 line 4
 *   (R4NTT_ROM[t++]) and Algorithm 6 line 11 (R4INTT_ROM[t++]).
 *   The counter increments once per Radix-4 *block*; it is held constant for
 *   all butterflies within that block (each block processes 4^p / 4 BFs).
 *
 * Radix-2 pass: counter incremented every 2 cycles (j/4 index), matching
 *   Algorithm 5 line 16 (omega_ROM[j/4]) and Algorithm 6 line 2 (omega_inv_ROM[j/4]).
 *   Each Radix-2 block has 2 butterflies sharing the same twiddle factor.
 *
 * CWM: address is just the block counter (0..63), output on r2_addr.
 *
 * Pass-at-a-Time Architecture:
 * The FSM executes one pass at a time, returning to S_IDLE after each pass
 * completes. The top-level controller pulses start_i for each pass with the
 * pass index on pass_idx_i (0..3). This decoupling allows the controller to
 * flush the PE pipeline, swap SRAM read/write pointers, and update ctrl_i
 * control lines between passes safely.
 *
 * NTT Schedule (Forward, Cooley-Tukey, Decimation-in-Time):
 * | Pass | Type    | Stages | Blocks | BFs/Block | Total Cycles | ROM        |
 * |------|---------|--------|--------|-----------|--------------|------------|
 * | 1    | Radix-4 | 1 & 2  | 1      | 64        | 64           | R4NTT  t=0 |
 * | 2    | Radix-4 | 3 & 4  | 4      | 16        | 64           | R4NTT  t=1..4 |
 * | 3    | Radix-4 | 5 & 6  | 16     | 4         | 64           | R4NTT  t=5..20 |
 * | 4    | Radix-2 | 7      | 64     | 2         | 128          | OMEGA  0..63 |
 *
 * INTT Schedule (Inverse, Gentleman-Sande, Decimation-in-Frequency):
 * | Pass | Type    | Stages | Blocks | BFs/Block | Total Cycles | ROM        |
 * |------|---------|--------|--------|-----------|--------------|------------|
 * | 1    | Radix-2 | 1      | 64     | 2         | 128          | OMEGA_INV 0..63 |
 * | 2    | Radix-4 | 2 & 3  | 16     | 4         | 64           | R4INTT t=0..15 |
 * | 3    | Radix-4 | 4 & 5  | 4      | 16        | 64           | R4INTT t=16..19 |
 * | 4    | Radix-4 | 6 & 7  | 1      | 64        | 64           | R4INTT t=20 |
 *
 * CWM Schedule (Coordinate-Wise Multiplication):
 * | Pass | Type    | Blocks | BFs/Block | Total Cycles |
 * |------|---------|--------|-----------|--------------|
 * | 1    | BaseMul | 64     | 1         | 64           |
 *
 * Latency: 0 clock cycles (combinational address output, registered in tf_rom)
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module tf_addr_gen (
    input   logic           clk,
    input   logic           rst,

    // ---- Control Interface (from AU Controller) ----
    input   logic           start_i,        // Pulse high for 1 cycle to begin
    input   logic           advance_i,      // Advance on a real issued TF consumer beat
    input   pe_mode_e       ctrl_i,         // Operating mode (NTT, INTT, CWM)
    input   logic [1:0]     pass_idx_i,     // Which pass to run (0..3)

    // ---- ROM Address Outputs (directly wired to tf_rom) ----
    output  logic [5:0]     tf_addr_o,       // Unified ROM address bus

    // ---- Control Outputs (directly wired to tf_rom) ----
    output  logic           is_radix2_o,    // 1 = current pass is Radix-2

    // ---- Status Outputs ----
    output  logic           valid_o,        // High when addresses are valid
    output  logic [1:0]     pass_o          // Current pass number (0-3)
);

    // =========================================================================
    // FSM State Encoding
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE      = 3'b000,
        S_PASS_1    = 3'b001,
        S_PASS_2    = 3'b010,
        S_PASS_3    = 3'b011,
        S_PASS_4    = 3'b100
    } state_e;

    state_e state_r, state_next;

    // =========================================================================
    // Internal Registers
    // =========================================================================

    // Mode latch (captured on start)
    pe_mode_e   mode_r;
    logic       is_intt_r;
    logic       is_cwm_r;

    // Butterfly counter within current block
    logic [5:0] bf_cnt_r;

    // Block counter within current pass
    logic [5:0] block_cnt_r;

    // Sequential Radix-4 ROM counter (t, persists across R4 passes)
    logic [4:0] r4_cnt_r;

    // Sequential Radix-2 ROM counter (j/4)
    logic [5:0] r2_cnt_r;

    // =========================================================================
    // Pass Configuration Logic
    // =========================================================================
    // Each pass has a fixed number of blocks and butterflies-per-block.

    logic [5:0] blocks_max;         // blocks_per_pass - 1 (0-indexed)
    logic [5:0] bfs_max;            // bfs_per_block - 1 (0-indexed)
    logic       pass_is_radix2;     // Current pass is Radix-2

    always_comb begin
        blocks_max     = '0;
        bfs_max        = '0;
        pass_is_radix2 = 1'b0;

        if (is_cwm_r) begin
            // CWM: 64 blocks x 1 BF
            blocks_max = 6'd63;
            bfs_max    = 6'd0;
        end else begin
            case (state_r)
                S_PASS_1: begin
                    if (is_intt_r) begin
                        // INTT Pass 1: R2, 64 groups x 1 issue cycle
                        // PE0 and PE2 perform the two R2 butterflies in parallel.
                        blocks_max     = 6'd63;
                        bfs_max        = 6'd0;
                        pass_is_radix2 = 1'b1;
                    end else begin
                        // NTT Pass 1: R4, 1 block x 64 BFs
                        blocks_max = 6'd0;
                        bfs_max    = 6'd63;
                    end
                end
                S_PASS_2: begin
                    if (is_intt_r) begin
                        // INTT Pass 2: R4, 16 blocks x 4 BFs
                        blocks_max = 6'd15;
                        bfs_max    = 6'd3;
                    end else begin
                        // NTT Pass 2: R4, 4 blocks x 16 BFs
                        blocks_max = 6'd3;
                        bfs_max    = 6'd15;
                    end
                end
                S_PASS_3: begin
                    if (is_intt_r) begin
                        // INTT Pass 3: R4, 4 blocks x 16 BFs
                        blocks_max = 6'd3;
                        bfs_max    = 6'd15;
                    end else begin
                        // NTT Pass 3: R4, 16 blocks x 4 BFs
                        blocks_max = 6'd15;
                        bfs_max    = 6'd3;
                    end
                end
                S_PASS_4: begin
                    if (is_intt_r) begin
                        // INTT Pass 4: R4, 1 block x 64 BFs
                        blocks_max = 6'd0;
                        bfs_max    = 6'd63;
                    end else begin
                        // NTT Pass 4: R2, 64 groups x 1 issue cycle
                        // PE0 and PE2 perform the two R2 butterflies in parallel.
                        blocks_max     = 6'd63;
                        bfs_max        = 6'd0;
                        pass_is_radix2 = 1'b1;
                    end
                end
                default: begin
                    blocks_max     = '0;
                    bfs_max        = '0;
                    pass_is_radix2 = 1'b0;
                end
            endcase
        end
    end

    // =========================================================================
    // End-of-block / End-of-pass Detection
    // =========================================================================
    logic bf_last;
    logic block_last;
    logic pass_last;

    assign bf_last    = (bf_cnt_r == bfs_max);
    assign block_last = (block_cnt_r == blocks_max);
    assign pass_last  = bf_last & block_last;

    // =========================================================================
    // Address Output Logic
    // =========================================================================
    always_comb begin
        if (state_r == S_IDLE) begin
            tf_addr_o = '0;
        end else if (is_cwm_r) begin
            tf_addr_o = block_cnt_r; // CWM uses 0..63
        end else if (pass_is_radix2) begin
            tf_addr_o = r2_cnt_r;    // Radix-2 uses 0..63
        end else begin
            tf_addr_o = {1'b0, r4_cnt_r}; // Radix-4 uses 0..20 (zero-padded)
        end
    end

    // =========================================================================
    // FSM: State Transition Logic
    // =========================================================================
    always_comb begin
        state_next = state_r;

        case (state_r)
            S_IDLE: begin
                if (start_i) begin
                    if (ctrl_i == PE_MODE_NTT || ctrl_i == PE_MODE_INTT) begin
                        case (pass_idx_i)
                            2'd0: state_next = S_PASS_1;
                            2'd1: state_next = S_PASS_2;
                            2'd2: state_next = S_PASS_3;
                            2'd3: state_next = S_PASS_4;
                            default: state_next = S_IDLE;
                        endcase
                    end else if (ctrl_i == PE_MODE_CWM) begin
                        state_next = S_PASS_1;
                    end
                end
            end

            S_PASS_1, S_PASS_2, S_PASS_3, S_PASS_4: begin
                if (advance_i && pass_last)
                    state_next = S_IDLE;
            end

            default: state_next = S_IDLE;
        endcase
    end

    // =========================================================================
    // FSM: Sequential Logic
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_r     <= S_IDLE;
            mode_r      <= PE_MODE_NTT;
            is_intt_r   <= 1'b0;
            is_cwm_r    <= 1'b0;
            bf_cnt_r    <= '0;
            block_cnt_r <= '0;
            r4_cnt_r    <= '0;
            r2_cnt_r    <= '0;
        end else begin
            state_r <= state_next;

            case (state_r)
                S_IDLE: begin
                    if (start_i) begin
                        mode_r      <= ctrl_i;
                        is_intt_r   <= (ctrl_i == PE_MODE_INTT);
                        is_cwm_r    <= (ctrl_i == PE_MODE_CWM);
                        bf_cnt_r    <= '0;
                        block_cnt_r <= '0;
                        r2_cnt_r    <= '0;
                        if (pass_idx_i == 2'd0)
                            r4_cnt_r <= '0;
                    end
                end

                S_PASS_1, S_PASS_2, S_PASS_3, S_PASS_4: begin
                    if (advance_i) begin
                        if (pass_last) begin
                            // End of pass: reset per-pass counters for next pass.
                            // Advance only when the controller actually issued the
                            // matching PE/ROM beat so TF state cannot outrun data.
                            bf_cnt_r    <= '0;
                            block_cnt_r <= '0;
                            r2_cnt_r    <= '0;
                            // r4_cnt_r persists across R4 passes AND increments at every
                            // block boundary (including the last block of a pass) to
                            // maintain the sequential t++ semantics (0, 1..4, 5..20).
                            if (!pass_is_radix2 && !is_cwm_r)
                                r4_cnt_r <= r4_cnt_r + 5'd1;
                        end else if (bf_last) begin
                            // End of block: advance to next block
                            bf_cnt_r    <= '0;
                            block_cnt_r <= block_cnt_r + 6'd1;

                            // Increment appropriate ROM counter at block boundary
                            if (pass_is_radix2) begin
                                r2_cnt_r <= r2_cnt_r + 6'd1;
                            end else if (!is_cwm_r) begin
                                r4_cnt_r <= r4_cnt_r + 5'd1;
                            end
                        end else begin
                            bf_cnt_r <= bf_cnt_r + 6'd1;
                        end
                    end
                end

                default: begin
                    bf_cnt_r    <= '0;
                    block_cnt_r <= '0;
                    r4_cnt_r    <= '0;
                    r2_cnt_r    <= '0;
                end
            endcase
        end
    end

    // =========================================================================
    // Status Outputs
    // =========================================================================
    assign valid_o     = (state_r != S_IDLE);
    assign is_radix2_o = pass_is_radix2;

    always_comb begin
        case (state_r)
            S_PASS_1: pass_o = 2'd0;
            S_PASS_2: pass_o = 2'd1;
            S_PASS_3: pass_o = 2'd2;
            S_PASS_4: pass_o = 2'd3;
            default:  pass_o = 2'd0;
        endcase
    end

endmodule
