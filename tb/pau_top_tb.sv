`timescale 1ns/1ps

import poly_arith_pkg::*;

module pau_top_tb;

    localparam int NUM_POLYS = 32;
    localparam int POLY_W    = $clog2(NUM_POLYS);

    logic       clk;
    logic       rst;
    logic       start_i;
    pe_mode_e   op_type_i;

    logic       pau_req_o;
    logic       pau_rd_en_o;
    logic [POLY_W-1:0] pau_rd_poly_id_o;
    logic [3:0][7:0] pau_rd_idx_o;
    logic [3:0]      pau_rd_lane_valid_o;
    logic [3:0]      pau_wr_en_o;
    logic [POLY_W-1:0] pau_wr_poly_id_o;
    logic [3:0][7:0] pau_wr_idx_o;
    logic [3:0][15:0] pau_wr_data_o;
    logic       pau_rd_valid_i;
    logic [POLY_W-1:0] pau_rd_poly_id_i;
    logic [3:0][7:0] pau_rd_idx_i;
    logic [3:0]      pau_rd_lane_valid_i;
    logic [3:0][15:0] pau_rd_data_i;
    logic       pau_stall_i;

    logic [POLY_W-1:0] mem_poly_id_mux;
    logic       mem_v_mux;
    logic       mem_rd_en_mux;
    logic       mem_ready_mux;

    // Correctly mapped all ports
    pau_top #(
        .NUM_POLYS(NUM_POLYS)
    ) DUT (
        .clk        (clk),
        .rst        (rst),
        .start_i    (start_i),
        .op_type_i  (op_type_i),
        .pau_req_o  (pau_req_o),
        .pau_rd_en_o(pau_rd_en_o),
        .pau_rd_poly_id_o(pau_rd_poly_id_o),
        .pau_rd_idx_o(pau_rd_idx_o),
        .pau_rd_lane_valid_o(pau_rd_lane_valid_o),
        .pau_wr_en_o(pau_wr_en_o),
        .pau_wr_poly_id_o(pau_wr_poly_id_o),
        .pau_wr_idx_o(pau_wr_idx_o),
        .pau_wr_data_o(pau_wr_data_o),
        .pau_rd_valid_i(pau_rd_valid_i),
        .pau_rd_poly_id_i(pau_rd_poly_id_i),
        .pau_rd_idx_i(pau_rd_idx_i),
        .pau_rd_lane_valid_i(pau_rd_lane_valid_i),
        .pau_rd_data_i(pau_rd_data_i),
        .pau_stall_i(pau_stall_i)
    );

    // Minimal local memory harness for the top-level smoke test.
    assign mem_poly_id_mux = pau_rd_en_o ? pau_rd_poly_id_o : pau_wr_poly_id_o;
    assign mem_v_mux       = pau_req_o;
    assign mem_rd_en_mux   = pau_rd_en_o;

    poly_mem_wrapper_4bank #(
        .N(256),
        .W(16),
        .NUM_POLYS(NUM_POLYS)
    ) u_local_mem (
        .clk(clk),
        .rst_n(rst),
        .poly_id_i(mem_poly_id_mux),
        .v_i(mem_v_mux),
        .rd_en_i(mem_rd_en_mux),
        .ready_o(mem_ready_mux),
        .rd_idx_i(pau_rd_idx_o),
        .rd_lane_valid_i(pau_rd_lane_valid_o),
        .rd_valid_o(pau_rd_valid_i),
        .rd_poly_id_o(pau_rd_poly_id_i),
        .rd_idx_o(pau_rd_idx_i),
        .rd_lane_valid_o(pau_rd_lane_valid_i),
        .rd_data_o(pau_rd_data_i),
        .wr_en_i(pau_wr_en_o),
        .wr_idx_i(pau_wr_idx_o),
        .wr_data_i(pau_wr_data_o)
    );

    assign pau_stall_i = ~mem_ready_mux;

    // ---------------------------------------------------------------------
    // Clock Generation (10ns period -> 100MHz)
    // ---------------------------------------------------------------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ---------------------------------------------------------------------
    // Stimulus
    // ---------------------------------------------------------------------
    initial begin
        // Initialize signals
        rst       = 1;
        start_i   = 0;
        op_type_i = PE_MODE_NTT;

        // Hold reset for a couple of clocks
        #20;
        rst = 0;
        #10;

        // Pulse start_i for 1 clock cycle
        start_i = 1;
        #10;
        start_i = 0;

        // Pass 1 processes 1 block of 64 butterflies (64 clock cycles).
        // 64 cycles * 10ns = 640ns. We'll wait 700ns to be safe.

        #2800;
        $display("Simulation Complete.");
        $finish;
    end

 // OKAY SO FOR NTT IT WILL GIVE A START SIGNAL AND A PASS INDEX EACH TIME
endmodule
