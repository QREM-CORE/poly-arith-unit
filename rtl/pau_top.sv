/*
 * Module Name: pau_top
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
 * Top module for the whole PAU system.
 *
 * This modified version adds PE -> local row accumulator -> CMI writeback
 * plumbing for the scratchpad-backed CWM update explored on the
 * quardins-attempted-updates branch.
 *
 * Notes:
 *   - Writeback is enabled for NTT / INTT / ADDSUB / COMP / DECOMP as before.
 *   - CWM now accumulates locally through mac_row_accum and writes back during
 *     a dedicated drain phase.
 *   - The true dual-source CWM / ADD / SUB memory interface is still a
 *     follow-on task. This branch focuses on the MAC/scratchpad side of the
 *     architecture and keeps the PAU auxiliary Memory descriptor tied idle.
 *   - PE op_a inputs now come from coeff_from_cmi rather than raw memory data.
 *   - The top-level memory boundary now exposes the current Memory Subsystem
 *     PAU primary + auxiliary port set. Only the primary descriptor is driven
 *     by current PAU RTL.
 */

import poly_arith_pkg::*;

module pau_top #(
    parameter int NUM_POLYS = 32
)(
    input  logic       clk,
    input  logic       rst,

    // ---- Control Interface (From Main System) ----
    input  logic       start_i,
    input  pe_mode_e   op_type_i,

    // ---- Memory Subsystem PAU Port ----
    output logic       pau_req_o,
    output logic       pau_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_rd_poly_id_o,
    output logic [3:0][7:0] pau_rd_idx_o,
    output logic [3:0]      pau_rd_lane_valid_o,
    output logic [3:0]      pau_wr_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_wr_poly_id_o,
    output logic [3:0][7:0] pau_wr_idx_o,
    output logic [3:0][15:0] pau_wr_data_o,
    input  logic       pau_rd_valid_i,
    input  logic [$clog2(NUM_POLYS)-1:0] pau_rd_poly_id_i,
    input  logic [3:0][7:0] pau_rd_idx_i,
    input  logic [3:0]      pau_rd_lane_valid_i,
    input  logic [3:0][15:0] pau_rd_data_i,
    input  logic       pau_stall_i,

    // ---- Memory Subsystem PAU Auxiliary Port ----
    output logic       pau_aux_req_o,
    output logic       pau_aux_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_aux_rd_poly_id_o,
    output logic [3:0][7:0] pau_aux_rd_idx_o,
    output logic [3:0]      pau_aux_rd_lane_valid_o,
    output logic [3:0]      pau_aux_wr_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_aux_wr_poly_id_o,
    output logic [3:0][7:0] pau_aux_wr_idx_o,
    output logic [3:0][15:0] pau_aux_wr_data_o,
    input  logic       pau_aux_rd_valid_i,
    input  logic [$clog2(NUM_POLYS)-1:0] pau_aux_rd_poly_id_i,
    input  logic [3:0][7:0] pau_aux_rd_idx_i,
    input  logic [3:0]      pau_aux_rd_lane_valid_i,
    input  logic [3:0][15:0] pau_aux_rd_data_i
);

    // ==========================================
    // Internal Signals
    // ==========================================

    // If poly select is not exposed yet, default to poly 0.
    localparam int POLY_W = $clog2(NUM_POLYS);
    localparam logic [POLY_W-1:0] DEFAULT_POLY_ID = '0;

    // NOTE(PAU/Mem): Current Memory exposes a PAU auxiliary descriptor so PAU
    // can own both internal memory ports for legal read/read, read/write, or
    // write/write phases. This top-level intentionally drives aux idle until
    // the PAU controller/CMI grow explicit second-source and split-destination
    // descriptors.
    //
    // Safe today:
    //   - NTT/INTT-style primary in-place read/write traffic.
    //
    // Not proven by this primary-only path:
    //   - CWM dual-read of A_hat[i][j] and s_hat[j].
    //   - ADD/SUB dual-source reads for X and Y operands.
    //   - CWM row finalize that reads EI/e_hat and writes final T/t_hat.
    //
    // TODO(PAU): drive these aux ports from a real dual-source CMI path before
    // treating CWM, ADD, or SUB as Memory-interface complete.
    assign pau_aux_req_o           = 1'b0;
    assign pau_aux_rd_en_o         = 1'b0;
    assign pau_aux_rd_poly_id_o    = '0;
    assign pau_aux_rd_idx_o        = '0;
    assign pau_aux_rd_lane_valid_o = '0;
    assign pau_aux_wr_en_o         = '0;
    assign pau_aux_wr_poly_id_o    = '0;
    assign pau_aux_wr_idx_o        = '0;
    assign pau_aux_wr_data_o       = '0;

    // Controller -> TF/PE side
    logic            ctl_ready;
    logic            ctl_done;
    logic            tf_start;
    logic [1:0]      pass_idx;
    logic            pe_valid;
    pe_mode_e        pe_ctrl;
    logic            mac_issue;
    logic            mac_first_term;
    logic [6:0]      mac_pair_idx;
    logic            mac_drain_issue;
    logic [6:0]      mac_drain_idx;
    logic            mac_fuse_e;
    logic            mac_drain_accept;

    // Controller -> CMI side
    logic            cmi_v;
    logic            cmi_rd_en;
    logic [POLY_W-1:0] cmi_poly_id;
    logic [3:0][7:0] cmi_coeff_idx;
    logic [3:0]      cmi_coeff_v;
    logic [3:0]      cmi_wb_latency;
    logic            cmi_ready;

    // CMI -> PE side
    logic [3:0][15:0]             coeff_from_cmi;

    // PE outputs
    coeff_t z0_o;
    coeff_t z1_o;
    coeff_t z2_o;
    coeff_t z3_o;
    logic   pe_wb_valid;
    logic [3:0]                   pe_wb_en;
    logic [3:0][15:0]  pe_wb_data;

    // Scratchpad-backed row accumulator signals
    coeff_t          cwm_z1_aligned;
    logic            cwm_valid_aligned;
    logic            cwm_first_term_aligned;
    logic [6:0]      cwm_pair_idx_aligned;
    coeff_t          w0_cwm_aligned;
    logic            drain_issue_d1;
    logic [6:0]      drain_idx_d1;
    logic            fuse_e_d1;
    coeff_t          acc_drain0;
    coeff_t          acc_drain1;
    logic            acc_drain_valid;
    logic [6:0]      acc_drain_pair_idx;

    // Address Generator & Twiddle Factor Signals
    logic [5:0]      tf_addr;
    logic            is_radix2;
    logic            is_radix2_pe;
    logic [1:0]      pass_out;
    logic            is_intt;
    logic            is_cwm;

    coeff_t          w0;
    coeff_t          w1;
    coeff_t          w2;
    coeff_t          w3;

    // ==========================================
    // Operational Logic
    // ==========================================

    always_comb begin
        case (op_type_i)
            PE_MODE_NTT  : begin
                is_intt = 1'b0;
                is_cwm  = 1'b0;
            end
            PE_MODE_INTT : begin
                is_intt = 1'b1;
                is_cwm  = 1'b0;
            end
            PE_MODE_CWM  : begin
                is_intt = 1'b0;
                is_cwm  = 1'b1;
            end
            default      : begin
                is_intt = 1'b0;
                is_cwm  = 1'b0;
            end
        endcase
    end

    // ----------------------------------------------------------
    // PE writeback packing
    // ----------------------------------------------------------
    always_comb begin
        pe_wb_data[0] = {4'b0000, z0_o};
        pe_wb_data[1] = {4'b0000, z1_o};
        pe_wb_data[2] = {4'b0000, z2_o};
        pe_wb_data[3] = {4'b0000, z3_o};
        pe_wb_en      = 4'b0000;

        // CWM now has a dedicated writeback path during the row drain phase.
        // The scratch accumulator emits the final t_hat pair after fusing
        // +e_hat, so only lanes 0/1 participate in this path.
        if (acc_drain_valid) begin
            pe_wb_data[0] = {4'b0000, acc_drain0};
            pe_wb_data[1] = {4'b0000, acc_drain1};
            pe_wb_en      = 4'b0011;
        end else if (pe_wb_valid) begin
            unique case (pe_ctrl)
                PE_MODE_NTT,
                PE_MODE_INTT,
                PE_MODE_ADDSUB,
                PE_MODE_COMP,
                PE_MODE_DECOMP: pe_wb_en = 4'b1111;
                PE_MODE_CWM:    pe_wb_en = 4'b0000;
                default:        pe_wb_en = 4'b0000;
            endcase
        end
    end

    // ==========================================
    // Sub-Module Instantiations
    // ==========================================

    // ---- Controller ----
    pau_controller #(
        .NUM_POLYS(NUM_POLYS)
    ) u_controller (
        .clk                (clk),
        .rst                (rst),
        .start_i            (start_i),
        .op_type_i          (op_type_i),
        .poly_id_i          (DEFAULT_POLY_ID),
        .ready_o            (ctl_ready),
        .done_o             (ctl_done),
        .tf_start_o         (tf_start),
        .pass_idx_o         (pass_idx),
        .pe_ctrl_o          (pe_ctrl),
        .pe_valid_o         (pe_valid),
        .mac_issue_o        (mac_issue),
        .mac_first_term_o   (mac_first_term),
        .mac_pair_idx_o     (mac_pair_idx),
        .mac_drain_issue_o  (mac_drain_issue),
        .mac_drain_idx_o    (mac_drain_idx),
        .mac_fuse_e_o       (mac_fuse_e),
        .mac_drain_accept_i (mac_drain_accept),
        .cmi_ready_i        (cmi_ready),
        .cmi_v_o            (cmi_v),
        .cmi_rd_en_o        (cmi_rd_en),
        .cmi_poly_id_o      (cmi_poly_id),
        .cmi_coeff_idx_o    (cmi_coeff_idx),
        .cmi_coeff_valid_o  (cmi_coeff_v),
        .cmi_wb_latency_o   (cmi_wb_latency),
        .block_cnt_o        (),
        .bf_cnt_o           ()
    );

    // ---- CMI ----
    // NOTE(PAU/Mem): u_cmi currently drives only the Memory primary PAU
    // descriptor. Memory can accept a PAU-owned auxiliary descriptor in the
    // same cycle, but that path is deliberately tied idle above.
    // TODO(PAU): add a second operand/source descriptor for ADD/SUB and CWM.
    cmi #(
        .NUM_POLYS(NUM_POLYS)
    ) u_cmi (
        .clk                    (clk),
        .rst                    (rst),
        .coeff_idx_i            (cmi_coeff_idx),
        .coeff_valid_i          (cmi_coeff_v),
        .poly_id_i              (cmi_poly_id),
        .v_i                    (cmi_v),
        .rd_en_i                (cmi_rd_en),
        .wb_latency_i           (cmi_wb_latency),
        .wr_en_i                (pe_wb_en),
        .wr_data_i              (pe_wb_data),
        .coeff_o                (coeff_from_cmi),
        .ready_o                (cmi_ready),
        .pau_req_o              (pau_req_o),
        .pau_rd_en_o            (pau_rd_en_o),
        .pau_rd_poly_id_o       (pau_rd_poly_id_o),
        .pau_rd_idx_o           (pau_rd_idx_o),
        .pau_rd_lane_valid_o    (pau_rd_lane_valid_o),
        .pau_wr_en_o            (pau_wr_en_o),
        .pau_wr_poly_id_o       (pau_wr_poly_id_o),
        .pau_wr_idx_o           (pau_wr_idx_o),
        .pau_wr_data_o          (pau_wr_data_o),
        .pau_rd_valid_i         (pau_rd_valid_i),
        .pau_rd_poly_id_i       (pau_rd_poly_id_i),
        .pau_rd_idx_i           (pau_rd_idx_i),
        .pau_rd_lane_valid_i    (pau_rd_lane_valid_i),
        .pau_rd_data_i          (pau_rd_data_i),
        .pau_stall_i            (pau_stall_i)
    );

    // ---- Twiddle Factor Address Generator ----
    tf_addr_gen u_tf_addr_gen (
        .clk                (clk),
        .rst                (rst),
        .start_i            (tf_start),
        .ctrl_i             (op_type_i),
        .pass_idx_i         (pass_idx),
        .tf_addr_o          (tf_addr),
        .is_radix2_o        (is_radix2),
        .valid_o            (),
        .pass_o             (pass_out)
    );

    // ---- Twiddle Factor ROM ----
    tf_rom u_tf_rom (
        .clk                (clk),
        .rst                (rst),
        .is_intt_i          (is_intt),
        .is_radix2_i        (is_radix2),
        .is_cwm_i           (is_cwm),
        .tf_addr_i          (tf_addr),
        .w0_o               (w0),
        .w1_o               (w1),
        .w2_o               (w2),
        .w3_o               (w3)
    );

    // Align the radix selector with the delayed controller/data path that
    // drives pe_unit on the cycle after the memory read request is issued.
    always_ff @(posedge clk) begin
        if (rst) begin
            is_radix2_pe <= 1'b0;
        end else begin
            is_radix2_pe <= is_radix2;
        end
    end

    // ==========================================================
    // CWM alignment helpers for the new scratch-backed row MAC
    // ==========================================================
    // These delays mirror the integration testbench findings:
    //   - z1 needs +1 cycle to align with z2
    //   - valid needs +4 cycles to align with the true CWM data
    //   - w0/zeta needs +3 cycles on the CWM path into PE3
    //
    // The existing PAU top previously omitted this and therefore did not
    // present a trustworthy CWM integration path. The goal here is to make
    // the new branch explicit about that timing.
    delay_n #(
        .DWIDTH (COEFF_WIDTH),
        .DEPTH  (1)
    ) u_cwm_align_z1 (
        .clk    (clk),
        .rst    (rst),
        .data_i (z1_o),
        .data_o (cwm_z1_aligned)
    );

    delay_n #(
        .DWIDTH (1),
        .DEPTH  (4)
    ) u_cwm_align_valid (
        .clk    (clk),
        .rst    (rst),
        .data_i (pe_wb_valid),
        .data_o (cwm_valid_aligned)
    );

    delay_n #(
        .DWIDTH (COEFF_WIDTH),
        .DEPTH  (3)
    ) u_cwm_align_w0 (
        .clk    (clk),
        .rst    (rst),
        .data_i (w0),
        .data_o (w0_cwm_aligned)
    );

    // Row-accumulator bookkeeping is delayed from the controller's issue cycle
    // to the true CWM output cycle seen by the scratch accumulator.
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (9)
    ) u_cwm_align_first_term (
        .clk    (clk),
        .rst    (rst),
        .data_i (mac_issue && mac_first_term),
        .data_o (cwm_first_term_aligned)
    );

    delay_n #(
        .DWIDTH (7),
        .DEPTH  (9)
    ) u_cwm_align_pair_idx (
        .clk    (clk),
        .rst    (rst),
        .data_i (mac_pair_idx),
        .data_o (cwm_pair_idx_aligned)
    );

    // DRAIN reads e_hat through the existing memory path. The response returns
    // one cycle after the controller issues the request, so delay the drain
    // bookkeeping by one cycle before pushing it into the row accumulator.
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (1)
    ) u_drain_issue_d1 (
        .clk    (clk),
        .rst    (rst),
        .data_i (mac_drain_issue),
        .data_o (drain_issue_d1)
    );

    delay_n #(
        .DWIDTH (7),
        .DEPTH  (1)
    ) u_drain_idx_d1 (
        .clk    (clk),
        .rst    (rst),
        .data_i (mac_drain_idx),
        .data_o (drain_idx_d1)
    );

    delay_n #(
        .DWIDTH (1),
        .DEPTH  (1)
    ) u_drain_fuse_d1 (
        .clk    (clk),
        .rst    (rst),
        .data_i (mac_fuse_e),
        .data_o (fuse_e_d1)
    );

    // ---- Processing Element (PE) Unit ----
    // NOTE(PAU): op_b is currently sourced from the twiddle/constant path.
    // That is correct for NTT/INTT, but not for ADD/SUB. ADD/SUB requires a
    // second memory-source vector Y[0..3], most naturally returned through the
    // Memory auxiliary PAU read channel once CMI grows that path.
    // TODO(PAU): route aux read data into op_b for ADD/SUB; do not assume the
    // NTT twiddle pattern generalizes to pointwise arithmetic.
    pe_unit u_pe_unit (
        .clk                (clk),
        .rst                (rst),
        .valid_i            (pe_valid),
        .ctrl_i             (pe_ctrl),
        .mode_i             (is_radix2_pe),
        .op_a0_i            (coeff_from_cmi[0][COEFF_WIDTH-1:0]),
        .op_a1_i            (coeff_from_cmi[1][COEFF_WIDTH-1:0]),
        .op_a2_i            (coeff_from_cmi[2][COEFF_WIDTH-1:0]),
        .op_a3_i            (coeff_from_cmi[3][COEFF_WIDTH-1:0]),
        .op_b0_i            (is_cwm ? w0_cwm_aligned : w0),
        .op_b1_i            (w1),
        .op_b2_i            (w2),
        .op_b3_i            (w3),
        .z0_o               (z0_o),
        .z1_o               (z1_o),
        .z2_o               (z2_o),
        .z3_o               (z3_o),
        .valid_o            (pe_wb_valid)
    );

    // ---- Scratch-backed row accumulator for CWM ----
    // The drain path intentionally reuses lanes 0/1 from coeff_from_cmi as
    // the e_hat pair being fused into the final t_hat writeback.
    mac_row_accum u_row_accum (
        .clk            (clk),
        .rst_n          (~rst),
        .acc_fire_i     (cwm_valid_aligned && (pe_ctrl == PE_MODE_CWM)),
        .first_term_i   (cwm_first_term_aligned),
        .pair_idx_i     (cwm_pair_idx_aligned),
        .cwm0_i         (cwm_z1_aligned),
        .cwm1_i         (z2_o),
        // The row accumulator should only drain once the e_hat pair being
        // fused into the final t_hat writeback has actually returned from the
        // memory subsystem on the PAU read-response channel.
        //
        // TODO(PAU, correctness): the present primary-only CMI uses one
        // poly_id for both read and write. A real row-finalize phase must read
        // EI/e_hat and write T0..T3/t_hat, so it needs either split read/write
        // poly IDs or an aux-assisted request shape. Do not treat this drain
        // plumbing as a complete Memory contract for KeyGen row commit.
        .drain_req_i    (drain_issue_d1 && pau_rd_valid_i),
        .drain_idx_i    (drain_idx_d1),
        .fuse_e_i       (fuse_e_d1),
        .e0_i           (coeff_from_cmi[0][COEFF_WIDTH-1:0]),
        .e1_i           (coeff_from_cmi[1][COEFF_WIDTH-1:0]),
        // Current top-level does not yet close writeback backpressure all the
        // way through CMI for the new drain path. Hold this at 1 for now and
        // document the limitation in the branch note.
        .drain_ready_i  (1'b1),
        .drain_accept_o (mac_drain_accept),
        .drain_valid_o  (acc_drain_valid),
        .drain_pair_idx_o(acc_drain_pair_idx),
        .drain0_o       (acc_drain0),
        .drain1_o       (acc_drain1)
    );

endmodule
