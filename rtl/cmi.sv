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
 * Notes:
 *   - Reset is active-high and synchronous.
 *   - Memory handles bank mapping + read-response ownership.
 *   - Writes are allowed even when no new read is being issued.
 */

module cmi #(
    parameter int N          = 256,
    parameter int W          = 16,
    parameter int NUM_POLYS  = 32,
    parameter int MAX_WB_LAT = 9
)(
    input  logic clk,
    input  logic rst,

    // ------------------------------------------------------------
    // From controller / rd_wr_addr_gen
    // ------------------------------------------------------------
    input  logic [3:0][7:0]              coeff_idx_i,
    input  logic [3:0]                   coeff_valid_i,

    // ------------------------------------------------------------
    // Control interface
    // ------------------------------------------------------------
    input  logic [$clog2(NUM_POLYS)-1:0] poly_id_i,
    input  logic                         v_i,
    input  logic                         rd_en_i,
    input  logic [3:0]                   wb_latency_i,

    // ------------------------------------------------------------
    // From AU writeback path
    // ------------------------------------------------------------
    input  logic [3:0]                   wr_en_i,
    input  logic [3:0][W-1:0]            wr_data_i,

    // ------------------------------------------------------------
    // Coefficient output to AU
    // ------------------------------------------------------------
    output logic [3:0][W-1:0]            coeff_o,

    // ------------------------------------------------------------
    // Status back to controller
    // ------------------------------------------------------------
    output logic                         ready_o,

    // ------------------------------------------------------------
    // To Memory Subsystem PAU port
    // ------------------------------------------------------------
    output logic                         pau_req_o,
    output logic                         pau_rd_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_rd_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]    pau_rd_idx_o,
    output logic [3:0]                   pau_rd_lane_valid_o,
    output logic [3:0]                   pau_wr_en_o,
    output logic [$clog2(NUM_POLYS)-1:0] pau_wr_poly_id_o,
    output logic [3:0][$clog2(N)-1:0]    pau_wr_idx_o,
    output logic [3:0][W-1:0]            pau_wr_data_o,

    // ------------------------------------------------------------
    // From Memory Subsystem PAU port
    // ------------------------------------------------------------
    input  logic                         pau_rd_valid_i,
    input  logic [$clog2(NUM_POLYS)-1:0] pau_rd_poly_id_i,
    input  logic [3:0][$clog2(N)-1:0]    pau_rd_idx_i,
    input  logic [3:0]                   pau_rd_lane_valid_i,
    input  logic [3:0][W-1:0]            pau_rd_data_i,
    input  logic                         pau_stall_i
);

    // ============================================================
    // READ REQUEST PATH
    // ============================================================
    assign pau_req_o           = v_i | (|wr_en_i);
    assign pau_rd_en_o         = rd_en_i;
    assign pau_rd_poly_id_o    = poly_id_i;
    assign pau_rd_idx_o        = coeff_idx_i;
    assign pau_rd_lane_valid_o = coeff_valid_i;
    assign pau_wr_poly_id_o    = poly_id_i;

    // ============================================================
    // READ RESPONSE PATH
    // ============================================================
    always_comb begin
        coeff_o = '0;

        if (pau_rd_valid_i) begin
            for (int i = 0; i < 4; i++) begin
                if (pau_rd_lane_valid_i[i]) begin
                    coeff_o[i] = pau_rd_data_i[i];
                end
            end
        end
    end

    // ============================================================
    // WRITEBACK ALIGNMENT
    // ============================================================
    logic [3:0][$clog2(N)-1:0] wr_idx_pipe   [0:MAX_WB_LAT];
    logic [3:0]                valid_pipe    [0:MAX_WB_LAT];

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

    logic [3:0][$clog2(N)-1:0] wr_idx_sel;
    logic [3:0]                coeff_valid_sel;

    always_comb begin
        wr_idx_sel      = wr_idx_pipe[2];
        coeff_valid_sel = valid_pipe[2];

        unique case (wb_latency_i)
            4'd2: begin wr_idx_sel = wr_idx_pipe[2]; coeff_valid_sel = valid_pipe[2]; end
            4'd4: begin wr_idx_sel = wr_idx_pipe[4]; coeff_valid_sel = valid_pipe[4]; end
            4'd5: begin wr_idx_sel = wr_idx_pipe[5]; coeff_valid_sel = valid_pipe[5]; end
            4'd9: begin wr_idx_sel = wr_idx_pipe[9]; coeff_valid_sel = valid_pipe[9]; end
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
