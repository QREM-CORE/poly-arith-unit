/*
 * Module Name: unipam_top
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
 * Top module for the whole UNIPAM system.
 *
 * Wire fixes in this version:
 *   - fixed missing comma in top-level port list
 *   - fixed controller -> CMI ready handshake (no longer tied to 1'b1)
 *   - fixed CMI instance to use current mem_* port names
 *   - fixed output-output short between cmi.ready_o and wrapper.ready_o
 *   - fixed cmi_coeff_valid width to 4 lanes
 */

import poly_arith_pkg::*;

module unipam_top (
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

    // CMI -> PE side (kept as internal for now)
    logic [3:0][15:0]             coeff_from_cmi;

    // PE
    coeff_t z0_o;
    coeff_t z1_o;
    coeff_t z2_o;
    coeff_t z3_o;

    // Address Generator & Twiddle Factor Signals
    logic [5:0]      tf_addr;
    logic            is_radix2;
    logic [1:0]      pass_out;
    logic            is_intt;

    coeff_t          w0;
    coeff_t          w1;
    coeff_t          w2;
    coeff_t          w3;

    // ==========================================
    // Operational Logic
    // ==========================================

    always_comb begin
        case (op_type_i)
            PE_MODE_NTT  : is_intt = 1'b0;
            PE_MODE_INTT : is_intt = 1'b1;
            default      : is_intt = 1'b0;
        endcase
    end

    // ==========================================
    // Sub-Module Instantiations
    // ==========================================

    // ---- Controller ----
    unipam_controller u_controller (
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
        .wr_en_i                (4'b0000),
        .wr_data_i              ('0),
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
        .tf_addr_i          (tf_addr),
        .w0_o               (w0),
        .w1_o               (w1),
        .w2_o               (w2),
        .w3_o               (w3)
    );

    // ---- Processing Element (PE) Unit ----
    // NEED TO ADD MUXING FOR ADD/SUB FOR OP_B
    // CURRENTLY WIRED FOR NTT ONLY..

    pe_unit u_pe_unit (
        .clk                (clk),
        .rst                (rst),
        .valid_i            (pe_valid),
        .ctrl_i             (pe_ctrl),
        .mode_i             (is_radix2),
        .op_a0_i            (mem_rd_data[0][COEFF_WIDTH-1:0]),
        .op_a1_i            (mem_rd_data[1][COEFF_WIDTH-1:0]),
        .op_a2_i            (mem_rd_data[2][COEFF_WIDTH-1:0]),
        .op_a3_i            (mem_rd_data[3][COEFF_WIDTH-1:0]),
        .op_b0_i            (w0),
        .op_b1_i            (w1),
        .op_b2_i            (w2),
        .op_b3_i            (w3),
        .z0_o               (z0_o),
        .z1_o               (z1_o),
        .z2_o               (z2_o),
        .z3_o               (z3_o),
        .valid_o            ()
    );

endmodule
