/*
 * Module Name: cmi (Conflict-Free Memory Interface)
 * Author(s): Mai Komar, Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 *   "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard"
 *   H. Jung, Q. D. Truong, H. Lee - IEEE OJCAS 2025
 *
 * Description:
 *   Adapter between the PAU controller / AU writeback path and the
 *   Memory Subsystem PAU port.
 *
 *   This CMI belongs to the PAU. It is intentionally kept with the PAU RTL so
 *   the PAU team owns the request timing, writeback alignment, and interface
 *   contract into memory.
 *
 * Responsibilities:
 *   - Forward 4-lane read requests (coefficient indices + valid flags) to the
 *     memory subsystem, which handles CMI bank mapping internally
 *   - Consume the subsystem's 1-cycle read response and present aligned
 *     coefficient data to the PE / arithmetic path
 *   - Align writeback indices via configurable delay pipelines so write
 *     addresses arrive at the memory wrapper at the same time as PAU result
 *     data
 *   - Allow write-only cycles (no concurrent read) for drain/final writeback
 *     phases
 *
 * Current Memory alignment:
 *   - NTT / INTT keep using the primary PAU descriptor for in-place traffic.
 *   - CWM accumulation can now mirror the accepted primary read beat onto the
 *     PAU auxiliary descriptor so the PE sees {A_hat, s_hat} in one cycle.
 *   - CWM drain still uses the primary response path for +e_hat fusion while
 *     writeback is redirected into the fixed T-slot map.
 *
 * Notes:
 *   - Reset is active-high and synchronous.
 *   - Memory handles bank mapping + read-response ownership.
 *   - Writes are allowed even when no new read is being issued.
 */

import qrem_mem_map_pkg::*;

module cmi #(
    parameter int N          = 256,
    parameter int W          = 16,
    parameter int NUM_POLYS  = 32,
    parameter int MAX_WB_LAT = 10
)(
    input  logic clk,
    input  logic rst,

    // ------------------------------------------------------------
    // From controller / rd_wr_addr_gen
    // ------------------------------------------------------------
    input  logic [3:0][7:0]               coeff_idx_i,
    input  logic [3:0]                    coeff_valid_i,

    // ------------------------------------------------------------
    // Control interface
    // ------------------------------------------------------------
    input  logic [$clog2(NUM_POLYS)-1:0]  poly_id_i,
    input  logic                          v_i,
    input  logic                          rd_en_i,
    input  logic [3:0]                    wb_latency_i,
    input  logic                          cwm_mode_i,
    input  logic                          cwm_issue_i,
    input  logic                          cwm_drain_issue_i,

    // ------------------------------------------------------------
    // From AU writeback path
    // ------------------------------------------------------------
    input  logic [3:0]                    wr_en_i,
    input  logic [3:0][W-1:0]             wr_data_i,

    // ------------------------------------------------------------
    // Coefficient output to AU
    // ------------------------------------------------------------
    output logic [3:0][W-1:0]             coeff_o,

    // ------------------------------------------------------------
    // Status back to controller
    // ------------------------------------------------------------
    output logic                          ready_o,

    // ------------------------------------------------------------
    // To Memory Subsystem PAU primary port
    // ------------------------------------------------------------
    output logic                          pau_req_o,
    output logic                          pau_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0]  pau_rd_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]     pau_rd_idx_o,
    output logic [3:0]                    pau_rd_lane_valid_o,
    output logic [3:0]                    pau_wr_en_o,
    output logic [$clog2(NUM_POLYS)-1:0]  pau_wr_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]     pau_wr_idx_o,
    output logic [3:0][W-1:0]             pau_wr_data_o,

    // ------------------------------------------------------------
    // From Memory Subsystem PAU primary port
    // ------------------------------------------------------------
    input  logic                          pau_rd_valid_i,
    input  logic [$clog2(NUM_POLYS)-1:0]  pau_rd_poly_id_i,
    input  logic [3:0][$clog2(N)-1:0]     pau_rd_idx_i,
    input  logic [3:0]                    pau_rd_lane_valid_i,
    input  logic [3:0][W-1:0]             pau_rd_data_i,
    input  logic                          pau_stall_i,

    // ------------------------------------------------------------
    // To / from Memory Subsystem PAU auxiliary port
    // ------------------------------------------------------------
    output logic                          pau_aux_req_o,
    output logic                          pau_aux_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0]  pau_aux_rd_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]     pau_aux_rd_idx_o,
    output logic [3:0]                    pau_aux_rd_lane_valid_o,
    output logic [3:0]                    pau_aux_wr_en_o,
    output logic [$clog2(NUM_POLYS)-1:0]  pau_aux_wr_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]     pau_aux_wr_idx_o,
    output logic [3:0][W-1:0]             pau_aux_wr_data_o,
    input  logic                          pau_aux_rd_valid_i,
    input  logic [$clog2(NUM_POLYS)-1:0]  pau_aux_rd_poly_id_i,
    input  logic [3:0][$clog2(N)-1:0]     pau_aux_rd_idx_i,
    input  logic [3:0]                    pau_aux_rd_lane_valid_i,
    input  logic [3:0][W-1:0]             pau_aux_rd_data_i
);

    localparam int POLY_W = $clog2(NUM_POLYS);

    logic [POLY_W-1:0] cwm_slot_sel;
    logic [POLY_W-1:0] primary_rd_poly_id_sel;
    logic [POLY_W-1:0] primary_wr_poly_id_sel;
    logic [POLY_W-1:0] aux_rd_poly_id_sel;
    logic              cwm_issue_d1_r;

    logic [3:0][$clog2(N)-1:0] wr_idx_pipe   [0:MAX_WB_LAT];
    logic [3:0]                valid_pipe    [0:MAX_WB_LAT];
    logic [3:0][$clog2(N)-1:0] wr_idx_sel;
    logic [3:0]                coeff_valid_sel;

    assign cwm_slot_sel = {{(POLY_W-2){1'b0}}, poly_id_i[1:0]};

    always_ff @(posedge clk) begin
        if (rst)
            cwm_issue_d1_r <= 1'b0;
        else
            cwm_issue_d1_r <= cwm_issue_i;
    end

    always_comb begin
        primary_rd_poly_id_sel = poly_id_i;
        primary_wr_poly_id_sel = poly_id_i;
        aux_rd_poly_id_sel     = '0;

        if (cwm_issue_i) begin
            primary_rd_poly_id_sel = POLY_W'(POLY_ID_A0) + cwm_slot_sel;
            aux_rd_poly_id_sel     = POLY_W'(POLY_ID_S0) + cwm_slot_sel;
        end else if (cwm_drain_issue_i) begin
            primary_rd_poly_id_sel = POLY_W'(POLY_ID_EI);
        end

        if (cwm_mode_i && (|wr_en_i)) begin
            primary_wr_poly_id_sel = POLY_W'(POLY_ID_T0) + cwm_slot_sel;
        end
    end

    // ============================================================
    // READ REQUEST PATH
    // ============================================================
    assign pau_req_o           = v_i | (|wr_en_i);
    assign pau_rd_en_o         = rd_en_i;
    assign pau_rd_poly_id_o    = primary_rd_poly_id_sel;
    assign pau_rd_idx_o        = coeff_idx_i;
    assign pau_rd_lane_valid_o = coeff_valid_i;
    assign pau_wr_poly_id_o    = primary_wr_poly_id_sel;

    assign pau_aux_req_o           = cwm_issue_i;
    assign pau_aux_rd_en_o         = cwm_issue_i;
    assign pau_aux_rd_poly_id_o    = aux_rd_poly_id_sel;
    assign pau_aux_rd_idx_o        = cwm_issue_i ? coeff_idx_i : '0;
    assign pau_aux_rd_lane_valid_o = cwm_issue_i ? coeff_valid_i : 4'b0000;
    assign pau_aux_wr_en_o         = 4'b0000;
    assign pau_aux_wr_poly_id_o    = '0;
    assign pau_aux_wr_idx_o        = '0;
    assign pau_aux_wr_data_o       = '0;

    // ============================================================
    // READ RESPONSE PATH
    // ============================================================
    always_comb begin
        coeff_o = '0;

        if (cwm_issue_d1_r) begin
            for (int i = 0; i < 2; i++) begin
                if (pau_rd_valid_i && pau_rd_lane_valid_i[i])
                    coeff_o[i] = pau_rd_data_i[i];
                if (pau_aux_rd_valid_i && pau_aux_rd_lane_valid_i[i])
                    coeff_o[i+2] = pau_aux_rd_data_i[i];
            end
        end else if (pau_rd_valid_i) begin
            for (int i = 0; i < 4; i++) begin
                if (pau_rd_lane_valid_i[i])
                    coeff_o[i] = pau_rd_data_i[i];
            end
        end
    end

    // ============================================================
    // WRITEBACK ALIGNMENT
    // ============================================================
    generate
        for (genvar i0 = 0; i0 < 4; i0++) begin : G_WB_PIPE_HEAD
            assign wr_idx_pipe[0][i0] = coeff_idx_i[i0];
            assign valid_pipe[0][i0]  = coeff_valid_i[i0];
        end
    endgenerate

    generate
        for (genvar d = 0; d < MAX_WB_LAT; d++) begin : G_WB_PIPE_STAGE
            for (genvar i = 0; i < 4; i++) begin : G_WB_PIPE_LANE
                delay_n #(
                    .DWIDTH ($clog2(N)),
                    .DEPTH  (d+1)
                ) u_idx_delay (
                    .clk    (clk),
                    .rst    (rst),
                    .data_i (coeff_idx_i[i]),
                    .data_o (wr_idx_pipe[d+1][i])
                );

                delay_n #(
                    .DWIDTH (1),
                    .DEPTH  (d+1)
                ) u_valid_delay (
                    .clk    (clk),
                    .rst    (rst),
                    .data_i (coeff_valid_i[i]),
                    .data_o (valid_pipe[d+1][i])
                );
            end
        end
    endgenerate

    always_comb begin
        wr_idx_sel      = wr_idx_pipe[2];
        coeff_valid_sel = valid_pipe[2];

        unique case (wb_latency_i)
            4'd2: begin wr_idx_sel = wr_idx_pipe[2]; coeff_valid_sel = valid_pipe[2]; end
            4'd4: begin wr_idx_sel = wr_idx_pipe[4]; coeff_valid_sel = valid_pipe[4]; end
            4'd5: begin wr_idx_sel = wr_idx_pipe[5]; coeff_valid_sel = valid_pipe[5]; end
            4'd6: begin wr_idx_sel = wr_idx_pipe[6]; coeff_valid_sel = valid_pipe[6]; end
            4'd9: begin wr_idx_sel = wr_idx_pipe[9]; coeff_valid_sel = valid_pipe[9]; end
            4'd10: begin wr_idx_sel = wr_idx_pipe[10]; coeff_valid_sel = valid_pipe[10]; end
            default: begin wr_idx_sel = wr_idx_pipe[2]; coeff_valid_sel = valid_pipe[2]; end
        endcase
    end

    assign pau_wr_en_o   = wr_en_i & coeff_valid_sel;
    assign pau_wr_idx_o  = wr_idx_sel;
    assign pau_wr_data_o = wr_data_i;

    // ============================================================
    // READY
    // ============================================================
    assign ready_o = ~pau_stall_i;

endmodule
