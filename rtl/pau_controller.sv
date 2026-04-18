/*
 * Module Name: pau_controller
 * Author(s): Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * The pau_controller is the central control unit of PAU. It orchestrates
 * data movement, computation scheduling, and pipeline synchronization across
 * the PEs, memory, and tf subsystem.
 *
 * Functionality:
 * 1. PAU controller begins execution when start_i is asserted.
 * 2. op_type_i is latched at job start.
 * 3. Supported modes include NTT, INTT, CWM, Compression/Decompression, Add/Sub.
 *    Operation type pe_ctrl_o is sent to pe_unit.
 * 4. On this branch, CWM additionally exposes a row-accumulator control plane
 *    so pau_top can trap the running MAC sum inside a local scratch polynomial
 *    before draining the final result back to memory.
 */

import poly_arith_pkg::*;

module pau_controller #(
    parameter int NUM_POLYS = 32
)(
    input  logic             clk,
    input  logic             rst,

    // ---- Job control ----
    input  logic             start_i,
    input  pe_mode_e         op_type_i,
    input  logic [$clog2(NUM_POLYS)-1:0] poly_id_i,

    output logic             ready_o,
    output logic             done_o,

    // ---- Twiddle / pass control ----
    output logic             tf_start_o,
    output logic [1:0]       pass_idx_o,

    // ---- Interface to AU / PE array ----
    output pe_mode_e         pe_ctrl_o,
    output logic             pe_valid_o,

    // ---- Interface to scratchpad-backed CWM row MAC ----
    // These signals model the "issue -> accumulate locally -> drain once"
    // architecture discussed for the row-wise PAU update. The controller
    // still supports the original single-stream NTT/INTT/AddSub schedules.
    output logic             mac_issue_o,
    output logic             mac_first_term_o,
    output logic [6:0]       mac_pair_idx_o,
    output logic             mac_drain_issue_o,
    output logic [6:0]       mac_drain_idx_o,
    output logic             mac_fuse_e_o,
    input  logic             mac_drain_accept_i,

    // ---- Interface to CMI ----
    input  logic             cmi_ready_i,
    output logic             cmi_v_o,
    output logic             cmi_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] cmi_poly_id_o,
    output logic [3:0][7:0]  cmi_coeff_idx_o,
    output logic [3:0]       cmi_coeff_valid_o,
    output logic [3:0]       cmi_wb_latency_o,

    // ---- Optional pass status ----
    output logic [5:0]       block_cnt_o,
    output logic [5:0]       bf_cnt_o
);

    // =========================================================================
    // Local parameters (replace magic numbers)
    // =========================================================================

    // General architecture constants
    localparam logic [8:0] COEFFS_PER_POLY      = 9'd256;
    localparam logic [7:0] COEFFS_PER_ISSUE     = 8'd4;
    localparam logic [6:0] CWM_PAIR_LAST        = 7'd127;

    // Pass indices
    localparam logic [1:0] PASS_0               = 2'd0;
    localparam logic [1:0] PASS_1               = 2'd1;
    localparam logic [1:0] PASS_2               = 2'd2;
    localparam logic [1:0] PASS_3               = 2'd3;

    // Pipeline latencies
    localparam logic [3:0] PIPE_LAT_ADD_SUB     = 4'd1;
    localparam logic [3:0] PIPE_LAT_COMP        = 4'd3;
    localparam logic [3:0] PIPE_LAT_NTT_INTT    = 4'd8;

    // Common stride values
    localparam logic [7:0] STRIDE_1             = 8'd1;
    localparam logic [7:0] STRIDE_4             = 8'd4;
    localparam logic [7:0] STRIDE_16            = 8'd16;
    localparam logic [7:0] STRIDE_64            = 8'd64;

    // Max counter values per schedule
    // These are terminal counts, not total counts.
    localparam logic [5:0] COUNT_1              = 6'd0;
    localparam logic [5:0] COUNT_4              = 6'd3;
    localparam logic [5:0] COUNT_16             = 6'd15;
    localparam logic [5:0] COUNT_64             = 6'd63;

    // Index offsets
    localparam logic [7:0] IDX_OFF_0            = 8'd0;
    localparam logic [7:0] IDX_OFF_1            = 8'd1;
    localparam logic [7:0] IDX_OFF_2            = 8'd2;
    localparam logic [7:0] IDX_OFF_3            = 8'd3;

    // Zero constants
    localparam int POLY_W                       = $clog2(NUM_POLYS);
    localparam logic [5:0] ZERO6                = 6'd0;
    localparam logic [7:0] ZERO8                = 8'd0;
    localparam logic [3:0] ZERO4                = 4'd0;
    localparam logic [1:0] ZERO2                = 2'd0;

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE      = 3'd0,
        S_SETUP     = 3'd1,
        S_RUN       = 3'd2,
        S_DRAIN     = 3'd3,
        S_NEXT_PASS = 3'd4,
        S_DONE      = 3'd5
    } state_e;

    state_e state_r, state_n;

    // =========================================================================
    // Latched operation + current pass
    // =========================================================================
    pe_mode_e   op_r;
    logic [POLY_W-1:0] poly_id_r;
    logic [1:0] pass_idx_r, pass_idx_n;

    // =========================================================================
    // Current-pass configuration
    // =========================================================================
    logic          pass_is_radix2;
    logic [5:0]    blocks_max;
    logic [5:0]    bfs_max;
    logic [3:0]    pipe_lat;
    logic          pass_uses_tf;
    logic          last_pass;

    // =========================================================================
    // Per-pass counters
    // =========================================================================
    logic [5:0]    block_cnt_r, block_cnt_n;
    logic [5:0]    bf_cnt_r,    bf_cnt_n;

    logic          bf_last;
    logic          block_last;
    logic          issue_last;

    // =========================================================================
    // Drain control
    // =========================================================================
    logic [3:0]    drain_cnt_r, drain_cnt_n;
    logic          drain_done;

    // CWM-specific drain counter.
    logic [6:0]    cwm_drain_idx_r, cwm_drain_idx_n;

    // =========================================================================
    // Streaming issue counter (for non-NTT streaming ops)
    // =========================================================================
    logic [7:0]    issue_addr_r, issue_addr_n;

    // =========================================================================
    // Read issue / stall handling
    // =========================================================================
    logic issue_fire;
    logic cwm_drain_issue;

    // =========================================================================
    // 1-cycle delay for PE valid/control to match wrapper read latency
    // =========================================================================
    pe_mode_e pe_ctrl_d1_r;
    logic     pe_valid_d1_r;

    // =========================================================================
    // Index generation helpers
    // =========================================================================
    logic [7:0] idx0, idx1, idx2, idx3;
    logic [7:0] base_idx;
    logic [7:0] stride;

    // =========================================================================
    // Pass configuration
    // =========================================================================
    always_comb begin
        pass_is_radix2 = 1'b0;
        blocks_max     = ZERO6;
        bfs_max        = ZERO6;
        pipe_lat       = PIPE_LAT_ADD_SUB;
        pass_uses_tf   = 1'b0;
        last_pass      = 1'b1;

        unique case (op_r)

            PE_MODE_NTT: begin
                pipe_lat     = PIPE_LAT_NTT_INTT;
                pass_uses_tf = 1'b1;
                last_pass    = (pass_idx_r == PASS_3);

                unique case (pass_idx_r)
                    PASS_0: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_1;
                        bfs_max        = COUNT_64;
                    end
                    PASS_1: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_4;
                        bfs_max        = COUNT_16;
                    end
                    PASS_2: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_16;
                        bfs_max        = COUNT_4;
                    end
                    PASS_3: begin
                        pass_is_radix2 = 1'b1;
                        blocks_max     = COUNT_64;
                        bfs_max        = COUNT_1;
                    end
                    default: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = ZERO6;
                        bfs_max        = ZERO6;
                    end
                endcase
            end

            PE_MODE_INTT: begin
                pipe_lat     = PIPE_LAT_NTT_INTT;
                pass_uses_tf = 1'b1;
                last_pass    = (pass_idx_r == PASS_3);

                unique case (pass_idx_r)
                    PASS_0: begin
                        pass_is_radix2 = 1'b1;
                        blocks_max     = COUNT_64;
                        bfs_max        = COUNT_1;
                    end
                    PASS_1: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_16;
                        bfs_max        = COUNT_4;
                    end
                    PASS_2: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_4;
                        bfs_max        = COUNT_16;
                    end
                    PASS_3: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = COUNT_1;
                        bfs_max        = COUNT_64;
                    end
                    default: begin
                        pass_is_radix2 = 1'b0;
                        blocks_max     = ZERO6;
                        bfs_max        = ZERO6;
                    end
                endcase
            end

            PE_MODE_CWM: begin
                pass_is_radix2 = 1'b0;
                blocks_max     = COUNT_64;
                bfs_max        = COUNT_1;
                pipe_lat       = PIPE_LAT_NTT_INTT;
                pass_uses_tf   = 1'b1;
                last_pass      = 1'b1;
            end

            PE_MODE_COMP,
            PE_MODE_DECOMP: begin
                pass_is_radix2 = 1'b0;
                blocks_max     = COUNT_64;
                bfs_max        = COUNT_1;
                pipe_lat       = PIPE_LAT_COMP;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end

            PE_MODE_ADDSUB: begin
                pass_is_radix2 = 1'b0;
                blocks_max     = COUNT_64;
                bfs_max        = COUNT_1;
                pipe_lat       = PIPE_LAT_ADD_SUB;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end

            default: begin
                pass_is_radix2 = 1'b0;
                blocks_max     = ZERO6;
                bfs_max        = ZERO6;
                pipe_lat       = PIPE_LAT_ADD_SUB;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end
        endcase
    end

    assign bf_last    = (bf_cnt_r    == bfs_max);
    assign block_last = (block_cnt_r == blocks_max);
    assign issue_last = bf_last && block_last;

    // =========================================================================
    // Drain detection
    // =========================================================================
    assign drain_done = (drain_cnt_r == ZERO4);

    // =========================================================================
    // CMI issue handshake
    // =========================================================================
    assign issue_fire      = (state_r == S_RUN) && cmi_ready_i;
    assign cwm_drain_issue = (state_r == S_DRAIN) &&
                             (op_r == PE_MODE_CWM) &&
                             cmi_ready_i &&
                             mac_drain_accept_i;

    // =========================================================================
    // Butterfly / stream coefficient index generation
    // =========================================================================
    always_comb begin
        idx0 = IDX_OFF_0;
        idx1 = IDX_OFF_1;
        idx2 = IDX_OFF_2;
        idx3 = IDX_OFF_3;

        base_idx = ZERO8;
        stride   = STRIDE_1;

        unique case (op_r)

            // -------------------------------------------------------------
            // NTT
            // pass 0: stride 64, blocks 1,  bfs 64
            // pass 1: stride 16, blocks 4,  bfs 16
            // pass 2: stride 4,  blocks 16, bfs 4
            // pass 3: radix-2 final stage, 64 groups of 4
            // -------------------------------------------------------------
            PE_MODE_NTT: begin
                unique case (pass_idx_r)
                    PASS_0: begin
                        stride   = STRIDE_64;
                        base_idx = bf_cnt_r;
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    PASS_1: begin
                        stride   = STRIDE_16;
                        base_idx = (block_cnt_r << 6) + bf_cnt_r; // block*64 + bf
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    PASS_2: begin
                        stride   = STRIDE_4;
                        base_idx = (block_cnt_r << 4) + bf_cnt_r; // block*16 + bf
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    PASS_3: begin
                        base_idx = (block_cnt_r << 2); // group of 4
                        // two radix-2 butterflies in parallel: (0,2) and (1,3)
                        idx0     = base_idx + IDX_OFF_0;
                        idx1     = base_idx + IDX_OFF_2;
                        idx2     = base_idx + IDX_OFF_1;
                        idx3     = base_idx + IDX_OFF_3;
                    end

                    default: begin
                        idx0 = IDX_OFF_0;
                        idx1 = IDX_OFF_1;
                        idx2 = IDX_OFF_2;
                        idx3 = IDX_OFF_3;
                    end
                endcase
            end

            // -------------------------------------------------------------
            // INTT
            // pass 0: radix-2 first stage, 64 groups of 4
            // pass 1: stride 4,  blocks 16, bfs 4
            // pass 2: stride 16, blocks 4,  bfs 16
            // pass 3: stride 64, blocks 1,  bfs 64
            // -------------------------------------------------------------
            PE_MODE_INTT: begin
                unique case (pass_idx_r)
                    PASS_0: begin
                        base_idx = (block_cnt_r << 2);
                        idx0     = base_idx + IDX_OFF_0;
                        idx1     = base_idx + IDX_OFF_2;
                        idx2     = base_idx + IDX_OFF_1;
                        idx3     = base_idx + IDX_OFF_3;
                    end

                    PASS_1: begin
                        stride   = STRIDE_4;
                        base_idx = (block_cnt_r << 4) + bf_cnt_r; // block*16 + bf
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    PASS_2: begin
                        stride   = STRIDE_16;
                        base_idx = (block_cnt_r << 6) + bf_cnt_r; // block*64 + bf
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    PASS_3: begin
                        stride   = STRIDE_64;
                        base_idx = bf_cnt_r;
                        idx0     = base_idx;
                        idx1     = base_idx + stride;
                        idx2     = base_idx + (stride << 1);
                        idx3     = base_idx + (stride * 3);
                    end

                    default: begin
                        idx0 = IDX_OFF_0;
                        idx1 = IDX_OFF_1;
                        idx2 = IDX_OFF_2;
                        idx3 = IDX_OFF_3;
                    end
                endcase
            end

            // -------------------------------------------------------------
            // Streaming / 4-coeff-per-cycle modes
            // -------------------------------------------------------------
            PE_MODE_COMP,
            PE_MODE_DECOMP,
            PE_MODE_ADDSUB: begin
                base_idx = (issue_addr_r << 2); // 4 coeffs per cycle
                idx0     = base_idx + IDX_OFF_0;
                idx1     = base_idx + IDX_OFF_1;
                idx2     = base_idx + IDX_OFF_2;
                idx3     = base_idx + IDX_OFF_3;
            end

            // -------------------------------------------------------------
            // CWM row accumulation
            //
            // NOTE:
            // The current PAU top-level still lacks the richer dual-source
            // memory interface needed to fetch A_ij and s_j independently in
            // the same cycle. The controller therefore exposes the *correct*
            // pair index schedule for the local scratch accumulator, while
            // the simple read index pattern below remains a placeholder until
            // the wider CWM memory path is finished.
            // -------------------------------------------------------------
            PE_MODE_CWM: begin
                base_idx = {issue_addr_r[6:0], 1'b0}; // pair_idx * 2
                idx0     = base_idx + IDX_OFF_0;
                idx1     = base_idx + IDX_OFF_1;
                idx2     = ZERO8;
                idx3     = ZERO8;
            end

            default: begin
                idx0 = IDX_OFF_0;
                idx1 = IDX_OFF_1;
                idx2 = IDX_OFF_2;
                idx3 = IDX_OFF_3;
            end
        endcase
    end

    // =========================================================================
    // Next-state logic
    // =========================================================================
    always_comb begin
        state_n      = state_r;
        pass_idx_n   = pass_idx_r;
        block_cnt_n  = block_cnt_r;
        bf_cnt_n     = bf_cnt_r;
        drain_cnt_n  = drain_cnt_r;
        issue_addr_n = issue_addr_r;
        cwm_drain_idx_n = cwm_drain_idx_r;

        unique case (state_r)

            S_IDLE: begin
                if (start_i)
                    state_n = S_SETUP;
            end

            S_SETUP: begin
                state_n = S_RUN;
            end

            S_RUN: begin
                if (cmi_ready_i) begin
                    if (op_r == PE_MODE_CWM) begin
                        // CWM no longer uses the old block/butterfly counters
                        // during accumulation. Instead, each read issue maps
                        // directly to one coefficient pair in the row-local
                        // scratch accumulator. Once all 128 pairs have been
                        // consumed, the controller switches into the dedicated
                        // drain phase that writes final t_hat back to memory.
                        if (issue_addr_r[6:0] == CWM_PAIR_LAST) begin
                            issue_addr_n    = ZERO8;
                            cwm_drain_idx_n = 7'd0;
                            state_n         = S_DRAIN;
                        end else begin
                            issue_addr_n = issue_addr_r + 8'd1;
                        end
                    end else begin
                        // Legacy controller behavior for the original pass
                        // scheduler. Non-CWM operations still advance through
                        // the existing block/butterfly machinery.
                        if ((op_r == PE_MODE_COMP) ||
                        (op_r == PE_MODE_DECOMP) ||
                            (op_r == PE_MODE_ADDSUB)) begin
                            issue_addr_n = issue_addr_r + 8'd1;
                        end

                        if (bf_last) begin
                            bf_cnt_n = ZERO6;

                            if (block_last) begin
                                block_cnt_n = ZERO6;
                                drain_cnt_n = pipe_lat;
                                state_n     = S_DRAIN;
                            end else begin
                                block_cnt_n = block_cnt_r + 6'd1;
                            end
                        end else begin
                            bf_cnt_n = bf_cnt_r + 6'd1;
                        end
                    end
                end
            end

            S_DRAIN: begin
                if (op_r == PE_MODE_CWM) begin
                    if (cmi_ready_i && mac_drain_accept_i) begin
                        if (cwm_drain_idx_r == CWM_PAIR_LAST) begin
                            state_n = S_DONE;
                        end else begin
                            cwm_drain_idx_n = cwm_drain_idx_r + 7'd1;
                        end
                    end
                end else begin
                    if (!drain_done)
                        drain_cnt_n = drain_cnt_r - 4'd1;
                    else
                        state_n = S_NEXT_PASS;
                end
            end

            S_NEXT_PASS: begin
                if (last_pass) begin
                    state_n = S_DONE;
                end else begin
                    pass_idx_n   = pass_idx_r + 2'd1;
                    block_cnt_n  = ZERO6;
                    bf_cnt_n     = ZERO6;
                    issue_addr_n = ZERO8;
                    state_n      = S_SETUP;
                end
            end

            S_DONE: begin
                state_n = S_IDLE;
            end

            default: state_n = S_IDLE;
        endcase
    end

    // =========================================================================
    // Sequential state/counter registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_r      <= S_IDLE;
            op_r         <= PE_MODE_NTT;
            poly_id_r    <= ZERO2;
            pass_idx_r   <= ZERO2;
            block_cnt_r  <= ZERO6;
            bf_cnt_r     <= ZERO6;
            drain_cnt_r  <= ZERO4;
            cwm_drain_idx_r <= 7'd0;
            issue_addr_r <= ZERO8;

            pe_ctrl_d1_r  <= PE_MODE_NTT;
            pe_valid_d1_r <= 1'b0;
        end else begin
            state_r      <= state_n;
            pass_idx_r   <= pass_idx_n;
            block_cnt_r  <= block_cnt_n;
            bf_cnt_r     <= bf_cnt_n;
            drain_cnt_r  <= drain_cnt_n;
            cwm_drain_idx_r <= cwm_drain_idx_n;
            issue_addr_r <= issue_addr_n;

            // Latch job parameters once at job start
            if (state_r == S_IDLE && start_i) begin
                op_r         <= op_type_i;
                poly_id_r    <= poly_id_i;
                pass_idx_r   <= ZERO2;
                block_cnt_r  <= ZERO6;
                bf_cnt_r     <= ZERO6;
                drain_cnt_r  <= ZERO4;
                cwm_drain_idx_r <= 7'd0;
                issue_addr_r <= ZERO8;
            end

            // Delay control/valid by 1 cycle to match wrapper read latency
            pe_ctrl_d1_r  <= op_r;
            pe_valid_d1_r <= issue_fire;
        end
    end

    // =========================================================================
    // Outputs
    // =========================================================================
    assign ready_o            = (state_r == S_IDLE);
    assign done_o             = (state_r == S_DONE);

    assign pe_ctrl_o          = pe_ctrl_d1_r;
    assign pe_valid_o         = pe_valid_d1_r;

    assign tf_start_o         = (state_r == S_SETUP) && pass_uses_tf;
    assign pass_idx_o         = pass_idx_r;

    assign cmi_poly_id_o = poly_id_r;

    // CWM drain needs to keep reading e_hat pairs while the row accumulator
    // emits final t_hat writeback pairs. For all other ops, CMI is active
    // only during the original S_RUN read issue phase.
    assign cmi_v_o     = (state_r == S_RUN) ||
                         ((state_r == S_DRAIN) && (op_r == PE_MODE_CWM));
    assign cmi_rd_en_o = issue_fire || cwm_drain_issue;

    // In CWM drain we read just the e_hat pair that will be fused with the
    // scratch accumulator output. The writeback side reuses the same indices,
    // allowing e_hat to be overwritten in place by the final t_hat row.
    assign cmi_coeff_idx_o[0] =
        ((state_r == S_DRAIN) && (op_r == PE_MODE_CWM)) ? {cwm_drain_idx_r, 1'b0} : idx0;
    assign cmi_coeff_idx_o[1] =
        ((state_r == S_DRAIN) && (op_r == PE_MODE_CWM)) ? ({cwm_drain_idx_r, 1'b0} + 8'd1) : idx1;
    assign cmi_coeff_idx_o[2] = idx2;
    assign cmi_coeff_idx_o[3] = idx3;

    // CWM is called out explicitly because the true dual-source CWM fetch path
    // is still a follow-on task. During accumulation we only mark the first
    // two logical lanes as valid for the row-MAC flow introduced in this
    // branch. NTT/INTT/AddSub/Comp/Decomp retain the original 4-lane behavior.
    assign cmi_coeff_valid_o =
        ((state_r == S_DRAIN) && (op_r == PE_MODE_CWM)) ? 4'b0011 :
        ((state_r == S_RUN)   && (op_r == PE_MODE_CWM)) ? 4'b0011 :
        (state_r == S_RUN)                                ? 4'b1111 :
                                                            4'b0000;

    // CWM writeback latency is now state-dependent:
    //   RUN   : no PE writeback is expected yet, so the value is unused.
    //   DRAIN : read e_hat (1cc) -> fuse/output register in mac_row_accum (1cc)
    //           -> writeback, therefore 2cc from read issue to wr_en.
    assign cmi_wb_latency_o   = ((op_r == PE_MODE_NTT) || (op_r == PE_MODE_INTT)) ?
                                 (pass_is_radix2 ? 4'd5 : 4'd9) :
                                 ((op_r == PE_MODE_CWM) && (state_r == S_DRAIN)) ? 4'd2 :
                                 (op_r == PE_MODE_CWM)    ? 4'd9 :
                                 (op_r == PE_MODE_COMP)   ? 4'd4 :
                                 (op_r == PE_MODE_DECOMP) ? 4'd4 :
                                 (op_r == PE_MODE_ADDSUB) ? 4'd2 : 4'd2;

    // Expose the row-accumulator control plane so pau_top can delay/align it
    // to the real CWM datapath latency.
    assign mac_issue_o       = (op_r == PE_MODE_CWM) && issue_fire;
    assign mac_first_term_o  = (issue_addr_r == ZERO8);
    assign mac_pair_idx_o    = issue_addr_r[6:0];
    assign mac_drain_issue_o = cwm_drain_issue;
    assign mac_drain_idx_o   = cwm_drain_idx_r;
    assign mac_fuse_e_o      = (op_r == PE_MODE_CWM) && (state_r == S_DRAIN);

    assign block_cnt_o        = block_cnt_r;
    assign bf_cnt_o           = bf_cnt_r;

endmodule
