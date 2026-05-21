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

import poly_arith_pkg::*;

module cmi #(
    parameter int N          = 256,
    parameter int W          = 16,
    parameter int NUM_POLYS  = qrem_global_pkg::NUM_POLYS,
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
    input  logic [$clog2(NUM_POLYS)-1:0]  aux_poly_id_i,
    input  logic                          v_i,
    input  logic                          aux_v_i,
    input  logic                          rd_en_i,
    input  logic                          aux_rd_en_i,
    input  logic [3:0]                    wb_latency_i,
    input  logic                          cwm_mode_i,
    input  logic                          cwm_issue_i,
    input  logic                          cwm_drain_issue_i,
    input  logic [1:0]                    pass_idx_i,
    input  logic                          is_radix2_i,

    // ------------------------------------------------------------
    // From AU writeback path
    // ------------------------------------------------------------
    input  logic [3:0]                    wr_en_i,
    input  logic [3:0][W-1:0]             wr_data_i,

    // ------------------------------------------------------------
    // Coefficient output to AU
    // ------------------------------------------------------------
    output logic [3:0][W-1:0]             coeff_o,
    output logic [3:0][W-1:0]             aux_coeff_o,

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
    logic              primary_rd_accept;
    logic              aux_rd_accept;
    logic              primary_rd_pending_r;
    logic              aux_rd_pending_r;
    logic [POLY_W-1:0] primary_req_poly_id_r;
    logic [POLY_W-1:0] aux_req_poly_id_r;
    logic [3:0][$clog2(N)-1:0] primary_req_idx_r;
    logic [3:0][$clog2(N)-1:0] aux_req_idx_r;
    logic [3:0]                primary_req_valid_r;
    logic [3:0]                aux_req_valid_r;
    logic [3:0][W-1:0]         primary_coeff_reordered;
    logic [3:0][W-1:0]         aux_coeff_reordered;
    logic [3:0]                primary_req_match;
    logic [3:0]                primary_rsp_match;
    logic [3:0]                aux_req_match;
    logic [3:0]                aux_rsp_match;

    logic [3:0][$clog2(N)-1:0] wr_idx_pipe   [MAX_WB_LAT+1];
    logic [3:0]                valid_pipe    [MAX_WB_LAT+1];
    logic [1:0]                pass_idx_pipe [MAX_WB_LAT+1];
    logic                      is_radix2_pipe [MAX_WB_LAT+1];
    logic [3:0][$clog2(N)-1:0] wr_idx_sel;
    logic [3:0]                coeff_valid_sel;
    logic [1:0]                pass_idx_sel;
    logic                      is_radix2_sel;
    logic [3:0][$clog2(N)-1:0] wb_idx_head;
    logic [3:0]                wb_valid_head;

    assign cwm_slot_sel = {{(POLY_W-2){1'b0}}, poly_id_i[1:0]};

    always_comb begin
        primary_rd_poly_id_sel = poly_id_i;
        primary_wr_poly_id_sel = poly_id_i;
        aux_rd_poly_id_sel     = aux_poly_id_i;

        if (cwm_issue_i) begin
            // For KeyGen CWM, use the fixed slot structure if those IDs are
            // valid in the map. Otherwise, fall back to the IDs passed by the
            // job controller.
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

    assign pau_aux_req_o           = aux_v_i | (|pau_aux_wr_en_o);
    assign pau_aux_rd_en_o         = aux_rd_en_i;
    assign pau_aux_rd_poly_id_o    = aux_rd_poly_id_sel;
    assign pau_aux_rd_idx_o        = aux_rd_en_i ? coeff_idx_i : '0;
    assign pau_aux_rd_lane_valid_o = aux_rd_en_i ? coeff_valid_i : 4'b0000;
    assign pau_aux_wr_en_o         = 4'b0000;
    assign pau_aux_wr_poly_id_o    = '0;
    assign pau_aux_wr_idx_o        = '0;
    assign pau_aux_wr_data_o       = '0;

    assign primary_rd_accept = pau_req_o &&
                               pau_rd_en_o &&
                               (|pau_rd_lane_valid_o) &&
                               !pau_stall_i;
    assign aux_rd_accept     = pau_aux_req_o &&
                               pau_aux_rd_en_o &&
                               (|pau_aux_rd_lane_valid_o) &&
                               !pau_stall_i;

    always_ff @(posedge clk) begin
        if (rst) begin
            primary_rd_pending_r <= 1'b0;
            aux_rd_pending_r     <= 1'b0;
            primary_req_poly_id_r <= '0;
            aux_req_poly_id_r     <= '0;
            primary_req_idx_r     <= '0;
            aux_req_idx_r         <= '0;
            primary_req_valid_r   <= '0;
            aux_req_valid_r       <= '0;
        end else begin
            primary_rd_pending_r <= primary_rd_accept;
            aux_rd_pending_r     <= aux_rd_accept;

            if (primary_rd_accept) begin
                primary_req_poly_id_r <= pau_rd_poly_id_o;
                primary_req_idx_r     <= pau_rd_idx_o;
                primary_req_valid_r   <= pau_rd_lane_valid_o;
            end

            if (aux_rd_accept) begin
                aux_req_poly_id_r <= pau_aux_rd_poly_id_o;
                aux_req_idx_r     <= pau_aux_rd_idx_o;
                aux_req_valid_r   <= pau_aux_rd_lane_valid_o;
            end
        end
    end

    // ============================================================
    // READ RESPONSE PATH
    // ============================================================
    always_comb begin
        coeff_o = '0;
        aux_coeff_o = '0;
        primary_coeff_reordered = '0;
        aux_coeff_reordered     = '0; // Note: aux_coeff_reordered redefined as [3:0] below
        primary_req_match       = '0;
        primary_rsp_match       = '0;
        aux_req_match           = '0;
        aux_rsp_match           = '0;

        if (pau_rd_valid_i && primary_rd_pending_r) begin
            for (int dst = 0; dst < 4; dst++) begin
                if (primary_req_valid_r[dst]) begin
                    for (int src = 0; src < 4; src++) begin
                        if (pau_rd_lane_valid_i[src] &&
                            (pau_rd_idx_i[src] == primary_req_idx_r[dst])) begin
                            primary_coeff_reordered[dst] = pau_rd_data_i[src];
                            primary_req_match[dst]       = 1'b1;
                            primary_rsp_match[src]       = 1'b1;
                        end
                    end
                end
            end
            coeff_o = primary_coeff_reordered;
        end

        if (pau_aux_rd_valid_i && aux_rd_pending_r) begin
            for (int dst = 0; dst < 4; dst++) begin
                if (aux_req_valid_r[dst]) begin
                    for (int src = 0; src < 4; src++) begin
                        if (pau_aux_rd_lane_valid_i[src] &&
                            (pau_aux_rd_idx_i[src] == aux_req_idx_r[dst])) begin
                            aux_coeff_reordered[dst] = pau_aux_rd_data_i[src];
                            aux_req_match[dst]       = 1'b1;
                            aux_rsp_match[src]       = 1'b1;
                        end
                    end
                end
            end
            aux_coeff_o = aux_coeff_reordered;
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!rst) begin
            if (aux_rd_accept && (|pau_aux_rd_lane_valid_o[3:2])) begin
                $error("CMI aux read currently expects only request lanes 0/1 to be active, got lane_valid=%b",
                       pau_aux_rd_lane_valid_o);
            end

            if (pau_rd_valid_i) begin
                if (!primary_rd_pending_r) begin
                    $error("CMI received a primary read response without a stored accepted request");
                end else begin
                    if (pau_rd_poly_id_i !== primary_req_poly_id_r) begin
                        $error("CMI primary poly_id mismatch: req=%0d rsp=%0d",
                               primary_req_poly_id_r, pau_rd_poly_id_i);
                    end

                    for (int dst = 0; dst < 4; dst++) begin
                        if (primary_req_valid_r[dst] && !primary_req_match[dst]) begin
                            $error("CMI primary lane %0d missing match for requested idx=%0d", dst,
                                   primary_req_idx_r[dst]);
                        end
                    end

                    for (int src = 0; src < 4; src++) begin
                        if (pau_rd_lane_valid_i[src] && !primary_rsp_match[src]) begin
                            $error("CMI primary response lane %0d idx=%0d did not match any requested idx",
                                   src, pau_rd_idx_i[src]);
                        end
                    end

                    if (is_radix2_i || (pass_idx_i == 2'd1) || (pass_idx_i == 2'd2)) begin
                        $display("CMI primary resp pass=%0d radix2=%0b req_idx={%0d,%0d,%0d,%0d} ret_idx={%0d,%0d,%0d,%0d} coeff_o={%0h,%0h,%0h,%0h}",
                                 pass_idx_i, is_radix2_i,
                                 primary_req_idx_r[0], primary_req_idx_r[1],
                                 primary_req_idx_r[2], primary_req_idx_r[3],
                                 pau_rd_idx_i[0], pau_rd_idx_i[1],
                                 pau_rd_idx_i[2], pau_rd_idx_i[3],
                                 coeff_o[0], coeff_o[1], coeff_o[2], coeff_o[3]);
                    end
                end
            end

            if (pau_aux_rd_valid_i) begin
                if (!aux_rd_pending_r) begin
                    $error("CMI received an auxiliary read response without a stored accepted request");
                end else begin
                    if (pau_aux_rd_poly_id_i !== aux_req_poly_id_r) begin
                        $error("CMI auxiliary poly_id mismatch: req=%0d rsp=%0d",
                               aux_req_poly_id_r, pau_aux_rd_poly_id_i);
                    end

                    for (int dst = 0; dst < 2; dst++) begin
                        if (aux_req_valid_r[dst] && !aux_req_match[dst]) begin
                            $error("CMI auxiliary lane %0d missing match for requested idx=%0d", dst,
                                   aux_req_idx_r[dst]);
                        end
                    end

                    for (int src = 0; src < 4; src++) begin
                        if (pau_aux_rd_lane_valid_i[src] && !aux_rsp_match[src]) begin
                            $error("CMI auxiliary response lane %0d idx=%0d did not match any requested idx",
                                   src, pau_aux_rd_idx_i[src]);
                        end
                    end
                end
            end
        end
    end
`endif

    // INTT pass 0 reads Memory in the interleaved order {0,2,1,3} so PE0/PE2
    // consume the two radix-2 butterflies in parallel. The writeback side must
    // restore natural coefficient order {0,1,2,3}; later passes keep the
    // request order.
    always_comb begin
        wb_idx_head   = coeff_idx_i;
        wb_valid_head = coeff_valid_i;

        if (is_radix2_i && (pass_idx_i == 2'd0)) begin
            wb_idx_head[0]   = coeff_idx_i[0];
            wb_idx_head[1]   = coeff_idx_i[2];
            wb_idx_head[2]   = coeff_idx_i[1];
            wb_idx_head[3]   = coeff_idx_i[3];
            wb_valid_head[0] = coeff_valid_i[0];
            wb_valid_head[1] = coeff_valid_i[2];
            wb_valid_head[2] = coeff_valid_i[1];
            wb_valid_head[3] = coeff_valid_i[3];
        end
    end

    // ============================================================
    // WRITEBACK ALIGNMENT
    // ============================================================
    generate
        for (genvar i0 = 0; i0 < 4; i0++) begin : G_WB_PIPE_HEAD
            assign wr_idx_pipe[0][i0] = wb_idx_head[i0];
            assign valid_pipe[0][i0]  = wb_valid_head[i0];
        end
        assign pass_idx_pipe[0]   = pass_idx_i;
        assign is_radix2_pipe[0]  = is_radix2_i;
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
                    .data_i (wb_idx_head[i]),
                    .data_o (wr_idx_pipe[d+1][i])
                );

                delay_n #(
                    .DWIDTH (1),
                    .DEPTH  (d+1)
                ) u_valid_delay (
                    .clk    (clk),
                    .rst    (rst),
                    .data_i (wb_valid_head[i]),
                    .data_o (valid_pipe[d+1][i])
                );
            end

            delay_n #(
                .DWIDTH (2),
                .DEPTH  (d+1)
            ) u_pass_delay (
                .clk    (clk),
                .rst    (rst),
                .data_i (pass_idx_i),
                .data_o (pass_idx_pipe[d+1])
            );

            delay_n #(
                .DWIDTH (1),
                .DEPTH  (d+1)
            ) u_r2_delay (
                .clk    (clk),
                .rst    (rst),
                .data_i (is_radix2_i),
                .data_o (is_radix2_pipe[d+1])
            );
        end
    endgenerate

    always_comb begin
        wr_idx_sel      = wr_idx_pipe[2];
        coeff_valid_sel = valid_pipe[2];
        pass_idx_sel    = pass_idx_pipe[2];
        is_radix2_sel   = is_radix2_pipe[2];

        unique case (wb_latency_i)
            4'd2: begin
                wr_idx_sel = wr_idx_pipe[2]; coeff_valid_sel = valid_pipe[2];
                pass_idx_sel = pass_idx_pipe[2]; is_radix2_sel = is_radix2_pipe[2];
            end
            4'd4: begin
                wr_idx_sel = wr_idx_pipe[4]; coeff_valid_sel = valid_pipe[4];
                pass_idx_sel = pass_idx_pipe[4]; is_radix2_sel = is_radix2_pipe[4];
            end
            4'd5: begin
                wr_idx_sel = wr_idx_pipe[5]; coeff_valid_sel = valid_pipe[5];
                pass_idx_sel = pass_idx_pipe[5]; is_radix2_sel = is_radix2_pipe[5];
            end
            4'd6: begin
                wr_idx_sel = wr_idx_pipe[6]; coeff_valid_sel = valid_pipe[6];
                pass_idx_sel = pass_idx_pipe[6]; is_radix2_sel = is_radix2_pipe[6];
            end
            4'd9: begin
                wr_idx_sel = wr_idx_pipe[9]; coeff_valid_sel = valid_pipe[9];
                pass_idx_sel = pass_idx_pipe[9]; is_radix2_sel = is_radix2_pipe[9];
            end
            4'd10: begin
                wr_idx_sel = wr_idx_pipe[10]; coeff_valid_sel = valid_pipe[10];
                pass_idx_sel = pass_idx_pipe[10]; is_radix2_sel = is_radix2_pipe[10];
            end
            default: begin
                wr_idx_sel = wr_idx_pipe[2]; coeff_valid_sel = valid_pipe[2];
                pass_idx_sel = pass_idx_pipe[2]; is_radix2_sel = is_radix2_pipe[2];
            end
        endcase
    end

    // INTT radix-2 pass 0 writeback must reorder data/enables to match the
    // restored natural index order {0,1,2,3}.
    logic [3:0]        wr_en_reordered;
    logic [3:0][W-1:0] wr_data_reordered;

    always_comb begin
        wr_en_reordered   = wr_en_i;
        wr_data_reordered = wr_data_i;

        if (is_radix2_sel && (pass_idx_sel == 2'd0)) begin
            wr_en_reordered[1]   = wr_en_i[2];
            wr_en_reordered[2]   = wr_en_i[1];
            wr_data_reordered[1] = wr_data_i[2];
            wr_data_reordered[2] = wr_data_i[1];
        end
    end

    assign pau_wr_en_o   = wr_en_reordered & coeff_valid_sel;
    assign pau_wr_idx_o  = wr_idx_sel;
    assign pau_wr_data_o = wr_data_reordered;


    // ============================================================
    // READY
    // ============================================================
    assign ready_o = ~pau_stall_i;

endmodule
