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
 *   - The true dual-source CWM memory interface is still a follow-on task.
 *     This branch focuses on the MAC/scratchpad side of the architecture.
 *   - PE op_a inputs now come from coeff_from_cmi rather than raw mem_rd_data.
 */

import poly_arith_pkg::*;

module pau_top (
    input  logic       clk,
    input  logic       rst,

    // ---- Control Interface (From Main System) ----
    input  logic       start_i,
    input  pe_mode_e   op_type_i

    // output logic            ready_o,
    // output logic            done_o,

    // // ---- Memory Interface (To SRAM / CMI) ----
    // output logic            mem_wr_read_en_o,
    // output logic            mem_wr_write_en_o,
    // output logic [7:0]      rAddr_o,
    // output logic [7:0]      wAddr_o,

    // // Input Data Bus (From SRAM)
    // input  coeff_t          a0_i, b0_i,
    // input  coeff_t          a1_i, b1_i, c0_i, c1_i,
    // input  coeff_t          a2_i, b2_i,
    // input  coeff_t          a3_i, b3_i, w3_i,

    // // Output Data Bus (To SRAM)
    // output coeff_t          u0_o, v0_o,
    // output coeff_t          u1_o, v1_o,
    // output coeff_t          u2_o, v2_o, m_o,
    // output coeff_t          u3_o, v3_o
);

    // ==========================================
    // Internal Signals
    // ==========================================

    // If poly select is not exposed yet, default to poly 0.
    localparam logic [1:0] DEFAULT_POLY_ID = 2'd0;

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
    logic [1:0]      cmi_poly_id;
    logic [3:0][7:0] cmi_coeff_idx;
    logic [3:0]      cmi_coeff_v;
    logic [3:0]      cmi_wb_latency;
    logic            cmi_ready;

    // CMI -> memory wrapper side
    logic [1:0]                   mem_poly_id;
    logic                         mem_v;
    logic                         mem_rd_en;
    logic [3:0][7:0]              mem_rd_idx;
    logic [3:0]                   mem_rd_lane_valid;
    logic [3:0]                   mem_wr_en;
    logic [3:0][7:0]              mem_wr_idx;
    logic [3:0][15:0]             mem_wr_data;
    logic                         mem_ready;

    // Memory wrapper -> CMI side
    logic                         mem_rd_valid;
    logic [1:0]                   mem_rd_poly_id;
    logic [3:0][7:0]              mem_rd_idx_rsp;
    logic [3:0]                   mem_rd_lane_valid_rsp;
    logic [3:0][15:0]             mem_rd_data;

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
    pau_controller u_controller (
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
    // NEED TO ADD POLY BANK B FOR ADD/SUB
    cmi u_cmi (
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
        .mem_poly_id_o          (mem_poly_id),
        .mem_v_o                (mem_v),
        .mem_rd_en_o            (mem_rd_en),
        .mem_rd_idx_o           (mem_rd_idx),
        .mem_rd_lane_valid_o    (mem_rd_lane_valid),
        .mem_wr_en_o            (mem_wr_en),
        .mem_wr_idx_o           (mem_wr_idx),
        .mem_wr_data_o          (mem_wr_data),
        .mem_rd_valid_i         (mem_rd_valid),
        .mem_rd_poly_id_i       (mem_rd_poly_id),
        .mem_rd_idx_i           (mem_rd_idx_rsp),
        .mem_rd_lane_valid_i    (mem_rd_lane_valid_rsp),
        .mem_rd_data_i          (mem_rd_data),
        .mem_ready_i            (mem_ready)
    );

    // ---- 4-bank polynomial memory wrapper ----
    poly_mem_wrapper_4bank u_poly_mem_wrapper (
        .clk                (clk),
        .rst_n              (rst),
        .poly_id_i          (mem_poly_id),
        .v_i                (mem_v),
        .rd_en_i            (mem_rd_en),
        .ready_o            (mem_ready),
        .rd_idx_i           (mem_rd_idx),
        .rd_lane_valid_i    (mem_rd_lane_valid),
        .rd_valid_o         (mem_rd_valid),
        .rd_poly_id_o       (mem_rd_poly_id),
        .rd_idx_o           (mem_rd_idx_rsp),
        .rd_lane_valid_o    (mem_rd_lane_valid_rsp),
        .rd_data_o          (mem_rd_data),
        .wr_en_i            (mem_wr_en),
        .wr_idx_i           (mem_wr_idx),
        .wr_data_i          (mem_wr_data)
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
        .DWIDTH (1),
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
    // NEED TO ADD MUXING FOR ADD/SUB FOR OP_B
    // CURRENTLY WIRED FOR NTT ONLY.
    pe_unit u_pe_unit (
        .clk                (clk),
        .rst                (rst),
        .valid_i            (pe_valid),
        .ctrl_i             (pe_ctrl),
        .mode_i             (is_radix2),
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
        .drain_req_i    (drain_issue_d1 && mem_rd_valid),
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
