`timescale 1ns/1ps

import qrem_mem_map_pkg::*;
import qrem_global_pkg::*;

module cmi_tb;

    localparam int N         = 256;
    localparam int W         = 16;
    localparam int NUM_POLYS = qrem_global_pkg::NUM_POLYS;
    localparam int POLY_W    = $clog2(NUM_POLYS);
    localparam int COEFF_W   = $clog2(N);

    logic clk;
    logic rst;

    logic [3:0][7:0]           coeff_idx_i;
    logic [3:0]                coeff_valid_i;
    logic [POLY_W-1:0]         poly_id_i;
    logic                      v_i;
    logic                      rd_en_i;
    logic [3:0]                wb_latency_i;
    logic                      cwm_mode_i;
    logic                      cwm_issue_i;
    logic                      cwm_drain_issue_i;
    logic [1:0]                pass_idx_i;
    logic                      is_radix2_i;
    logic [3:0]                wr_en_i;
    logic [3:0][W-1:0]         wr_data_i;
    logic [3:0][W-1:0]         coeff_o;
    logic                      ready_o;

    logic                      pau_req_o;
    logic                      pau_rd_en_o;
    logic [POLY_W-1:0]         pau_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0]   pau_rd_idx_o;
    logic [3:0]                pau_rd_lane_valid_o;
    logic [3:0]                pau_wr_en_o;
    logic [POLY_W-1:0]         pau_wr_poly_id_o;
    logic [3:0][COEFF_W-1:0]   pau_wr_idx_o;
    logic [3:0][W-1:0]         pau_wr_data_o;

    logic                      pau_rd_valid_i;
    logic [POLY_W-1:0]         pau_rd_poly_id_i;
    logic [3:0][COEFF_W-1:0]   pau_rd_idx_i;
    logic [3:0]                pau_rd_lane_valid_i;
    logic [3:0][W-1:0]         pau_rd_data_i;
    logic                      pau_stall_i;
    logic                      pau_aux_req_o;
    logic                      pau_aux_rd_en_o;
    logic [POLY_W-1:0]         pau_aux_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0]   pau_aux_rd_idx_o;
    logic [3:0]                pau_aux_rd_lane_valid_o;
    logic [3:0]                pau_aux_wr_en_o;
    logic [POLY_W-1:0]         pau_aux_wr_poly_id_o;
    logic [3:0][COEFF_W-1:0]   pau_aux_wr_idx_o;
    logic [3:0][W-1:0]         pau_aux_wr_data_o;
    logic                      pau_aux_rd_valid_i;
    logic [POLY_W-1:0]         pau_aux_rd_poly_id_i;
    logic [3:0][COEFF_W-1:0]   pau_aux_rd_idx_i;
    logic [3:0]                pau_aux_rd_lane_valid_i;
    logic [3:0][W-1:0]         pau_aux_rd_data_i;

    cmi #(
        .N(N),
        .W(W),
        .NUM_POLYS(NUM_POLYS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .coeff_idx_i(coeff_idx_i),
        .coeff_valid_i(coeff_valid_i),
        .poly_id_i(poly_id_i),
        .v_i(v_i),
        .rd_en_i(rd_en_i),
        .wb_latency_i(wb_latency_i),
        .cwm_mode_i(cwm_mode_i),
        .cwm_issue_i(cwm_issue_i),
        .cwm_drain_issue_i(cwm_drain_issue_i),
        .pass_idx_i(pass_idx_i),
        .is_radix2_i(is_radix2_i),
        .wr_en_i(wr_en_i),
        .wr_data_i(wr_data_i),
        .coeff_o(coeff_o),
        .ready_o(ready_o),
        .pau_req_o(pau_req_o),
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
        .pau_stall_i(pau_stall_i),
        .pau_aux_req_o(pau_aux_req_o),
        .pau_aux_rd_en_o(pau_aux_rd_en_o),
        .pau_aux_rd_poly_id_o(pau_aux_rd_poly_id_o),
        .pau_aux_rd_idx_o(pau_aux_rd_idx_o),
        .pau_aux_rd_lane_valid_o(pau_aux_rd_lane_valid_o),
        .pau_aux_wr_en_o(pau_aux_wr_en_o),
        .pau_aux_wr_poly_id_o(pau_aux_wr_poly_id_o),
        .pau_aux_wr_idx_o(pau_aux_wr_idx_o),
        .pau_aux_wr_data_o(pau_aux_wr_data_o),
        .pau_aux_rd_valid_i(pau_aux_rd_valid_i),
        .pau_aux_rd_poly_id_i(pau_aux_rd_poly_id_i),
        .pau_aux_rd_idx_i(pau_aux_rd_idx_i),
        .pau_aux_rd_lane_valid_i(pau_aux_rd_lane_valid_i),
        .pau_aux_rd_data_i(pau_aux_rd_data_i)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task automatic tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task automatic clear_inputs;
        begin
            coeff_idx_i         = '0;
            coeff_valid_i       = '0;
            poly_id_i           = '0;
            v_i                 = 1'b0;
            rd_en_i             = 1'b0;
            wb_latency_i        = 4'd2;
            cwm_mode_i          = 1'b0;
            cwm_issue_i         = 1'b0;
            cwm_drain_issue_i   = 1'b0;
            pass_idx_i          = '0;
            is_radix2_i         = 1'b0;
            wr_en_i             = '0;
            wr_data_i           = '0;
            pau_rd_valid_i      = 1'b0;
            pau_rd_poly_id_i    = '0;
            pau_rd_idx_i        = '0;
            pau_rd_lane_valid_i = '0;
            pau_rd_data_i       = '0;
            pau_stall_i         = 1'b0;
            pau_aux_rd_valid_i      = 1'b0;
            pau_aux_rd_poly_id_i    = '0;
            pau_aux_rd_idx_i        = '0;
            pau_aux_rd_lane_valid_i = '0;
            pau_aux_rd_data_i       = '0;
        end
    endtask

    initial begin
        rst = 1'b1;
        clear_inputs();
        repeat (2) tick();
        rst = 1'b0;
        tick();

        // ------------------------------------------------------
        // 1) Read request forwarding
        // ------------------------------------------------------
        poly_id_i           = POLY_W'(2);
        v_i                 = 1'b1;
        rd_en_i             = 1'b1;
        coeff_idx_i[0]      = COEFF_W'(8);
        coeff_idx_i[1]      = COEFF_W'(9);
        coeff_idx_i[2]      = COEFF_W'(10);
        coeff_idx_i[3]      = COEFF_W'(11);
        coeff_valid_i       = 4'b1111;
        #1;

        if (pau_rd_poly_id_o !== POLY_W'(2))
            $fatal(1, "CMI did not forward read poly_id correctly");
        if (!pau_req_o || !pau_rd_en_o)
            $fatal(1, "CMI did not forward read request correctly");
        if (pau_rd_idx_o[0] !== COEFF_W'(8) || pau_rd_idx_o[3] !== COEFF_W'(11))
            $fatal(1, "CMI read indices mismatch");
        if (pau_rd_lane_valid_o !== 4'b1111)
            $fatal(1, "CMI read lane-valid mismatch");

        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 2) Read response routing follows returned idx, not raw lane order
        // ------------------------------------------------------
        pau_rd_valid_i      = 1'b1;
        pau_rd_poly_id_i    = POLY_W'(2);
        pau_rd_lane_valid_i = 4'b1111;
        pau_rd_idx_i[0]     = COEFF_W'(11);
        pau_rd_idx_i[1]     = COEFF_W'(8);
        pau_rd_idx_i[2]     = COEFF_W'(10);
        pau_rd_idx_i[3]     = COEFF_W'(9);
        pau_rd_data_i[0]    = 16'h4444;
        pau_rd_data_i[1]    = 16'h1111;
        pau_rd_data_i[2]    = 16'h3333;
        pau_rd_data_i[3]    = 16'h2222;
        #1;
        if (coeff_o[0] !== 16'h1111 || coeff_o[1] !== 16'h2222 ||
            coeff_o[2] !== 16'h3333 || coeff_o[3] !== 16'h4444)
            $fatal(1, "CMI primary read response reorder mismatch");
        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 3) CWM dual-read response routing also follows returned idx
        // ------------------------------------------------------
        poly_id_i           = '0;
        v_i                 = 1'b1;
        rd_en_i             = 1'b1;
        cwm_mode_i          = 1'b1;
        cwm_issue_i         = 1'b1;
        coeff_idx_i[0]      = COEFF_W'(20);
        coeff_idx_i[1]      = COEFF_W'(21);
        coeff_valid_i       = 4'b0011;
        tick();
        clear_inputs();

        pau_rd_valid_i          = 1'b1;
        pau_rd_poly_id_i        = POLY_W'(POLY_ID_A0);
        pau_rd_lane_valid_i     = 4'b0011;
        pau_rd_idx_i[0]         = COEFF_W'(21);
        pau_rd_idx_i[1]         = COEFF_W'(20);
        pau_rd_data_i[0]        = 16'hA1A1;
        pau_rd_data_i[1]        = 16'hA0A0;
        pau_aux_rd_valid_i      = 1'b1;
        pau_aux_rd_poly_id_i    = POLY_W'(POLY_ID_S0);
        pau_aux_rd_lane_valid_i = 4'b0011;
        pau_aux_rd_idx_i[0]     = COEFF_W'(21);
        pau_aux_rd_idx_i[1]     = COEFF_W'(20);
        pau_aux_rd_data_i[0]    = 16'hB1B1;
        pau_aux_rd_data_i[1]    = 16'hB0B0;
        #1;
        if (coeff_o[0] !== 16'hA0A0 || coeff_o[1] !== 16'hA1A1 ||
            coeff_o[2] !== 16'hB0B0 || coeff_o[3] !== 16'hB1B1)
            $fatal(1, "CMI CWM primary/aux response reorder mismatch");
        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 4) INTT radix-2 writeback restores natural {0,1,2,3} order
        // ------------------------------------------------------
        pass_idx_i          = 2'd0;
        is_radix2_i         = 1'b1;
        coeff_idx_i[0]      = COEFF_W'(40);
        coeff_idx_i[1]      = COEFF_W'(42);
        coeff_idx_i[2]      = COEFF_W'(41);
        coeff_idx_i[3]      = COEFF_W'(43);
        coeff_valid_i       = 4'b1111;
        wb_latency_i        = 4'd2;
        wr_en_i             = 4'b1111;
        wr_data_i[0]        = 16'h0100;
        wr_data_i[1]        = 16'h0200;
        wr_data_i[2]        = 16'h0300;
        wr_data_i[3]        = 16'h0400;
        tick();
        clear_inputs();
        #1;
        if (pau_wr_idx_o[0] !== COEFF_W'(40) || pau_wr_idx_o[1] !== COEFF_W'(41) ||
            pau_wr_idx_o[2] !== COEFF_W'(42) || pau_wr_idx_o[3] !== COEFF_W'(43))
            $fatal(1, "CMI INTT radix-2 writeback index reorder mismatch");

        // ------------------------------------------------------
        // 5) Ready mirrors downstream memory readiness
        // ------------------------------------------------------
        pau_stall_i = 1'b1;
        #1;
        if (ready_o !== 1'b0)
            $fatal(1, "CMI ready_o should reflect the PAU stall input");
        pau_stall_i = 1'b0;
        #1;
        if (ready_o !== 1'b1)
            $fatal(1, "CMI ready_o failed to return high");

        // ------------------------------------------------------
        // 6) Write-only drain cycle is allowed
        // ------------------------------------------------------
        clear_inputs();
        wr_en_i          = 4'b0011;
        wr_data_i[0]     = 16'hAAAA;
        wr_data_i[1]     = 16'hBBBB;
        #1;
        if (!pau_req_o)
            $fatal(1, "CMI must assert pau_req_o for write-only cycles");
        if (pau_wr_poly_id_o !== '0)
            $fatal(1, "CMI write poly_id should default from the input poly_id");

        // ------------------------------------------------------
        // 7) Writeback alignment for latency=2
        // ------------------------------------------------------
        clear_inputs();
        coeff_idx_i[0]   = COEFF_W'(20);
        coeff_idx_i[1]   = COEFF_W'(21);
        coeff_idx_i[2]   = COEFF_W'(22);
        coeff_idx_i[3]   = COEFF_W'(23);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd2;
        tick();
        clear_inputs();
        tick();

        wr_en_i          = 4'b0011;
        wr_data_i[0]     = 16'hCAFE;
        wr_data_i[1]     = 16'hBEEF;
        wb_latency_i     = 4'd2;
        #1;
        if (pau_wr_en_o !== 4'b0011)
            $fatal(1, "CMI latency-2 write enable mismatch");
        if (pau_wr_idx_o[0] !== COEFF_W'(20) || pau_wr_idx_o[1] !== COEFF_W'(21))
            $fatal(1, "CMI latency-2 write index mismatch");
        if (pau_wr_data_o[0] !== 16'hCAFE || pau_wr_data_o[1] !== 16'hBEEF)
            $fatal(1, "CMI latency-2 write data mismatch");
        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 8) Writeback alignment for latency=4
        // ------------------------------------------------------
        coeff_idx_i[0]   = COEFF_W'(40);
        coeff_idx_i[1]   = COEFF_W'(41);
        coeff_idx_i[2]   = COEFF_W'(42);
        coeff_idx_i[3]   = COEFF_W'(43);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd4;
        tick();
        clear_inputs();
        repeat (3) tick();

        wr_en_i          = 4'b1100;
        wr_data_i[2]     = 16'h1234;
        wr_data_i[3]     = 16'h5678;
        wb_latency_i     = 4'd4;
        #1;
        if (pau_wr_en_o !== 4'b1100)
            $fatal(1, "CMI latency-4 write enable mismatch");
        if (pau_wr_idx_o[2] !== COEFF_W'(42) || pau_wr_idx_o[3] !== COEFF_W'(43))
            $fatal(1, "CMI latency-4 write index mismatch");
        if (pau_wr_data_o[2] !== 16'h1234 || pau_wr_data_o[3] !== 16'h5678)
            $fatal(1, "CMI latency-4 write data mismatch");

        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 9) Writeback alignment for latency=6
        // ------------------------------------------------------
        coeff_idx_i[0]   = COEFF_W'(60);
        coeff_idx_i[1]   = COEFF_W'(61);
        coeff_idx_i[2]   = COEFF_W'(62);
        coeff_idx_i[3]   = COEFF_W'(63);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd6;
        tick();
        clear_inputs();
        repeat (5) tick();

        wr_en_i          = 4'b1111;
        wr_data_i[0]     = 16'h5000;
        wr_data_i[1]     = 16'h5001;
        wr_data_i[2]     = 16'h5002;
        wr_data_i[3]     = 16'h5003;
        wb_latency_i     = 4'd6;
        #1;
        if (pau_wr_en_o !== 4'b1111)
            $fatal(1, "CMI latency-6 write enable mismatch");
        if (pau_wr_idx_o[0] !== COEFF_W'(60) || pau_wr_idx_o[3] !== COEFF_W'(63))
            $fatal(1, "CMI latency-6 write index mismatch");
        if (pau_wr_data_o[0] !== 16'h5000 || pau_wr_data_o[3] !== 16'h5003)
            $fatal(1, "CMI latency-6 write data mismatch");

        tick();
        clear_inputs();

        // ------------------------------------------------------
        // 10) Writeback alignment for latency=10
        // ------------------------------------------------------
        coeff_idx_i[0]   = COEFF_W'(80);
        coeff_idx_i[1]   = COEFF_W'(81);
        coeff_idx_i[2]   = COEFF_W'(82);
        coeff_idx_i[3]   = COEFF_W'(83);
        coeff_valid_i    = 4'b1111;
        wb_latency_i     = 4'd10;
        tick();
        clear_inputs();
        repeat (9) tick();

        wr_en_i          = 4'b0110;
        wr_data_i[1]     = 16'h9001;
        wr_data_i[2]     = 16'h9002;
        wb_latency_i     = 4'd10;
        #1;
        if (pau_wr_en_o !== 4'b0110)
            $fatal(1, "CMI latency-10 write enable mismatch");
        if (pau_wr_idx_o[1] !== COEFF_W'(81) || pau_wr_idx_o[2] !== COEFF_W'(82))
            $fatal(1, "CMI latency-10 write index mismatch");
        if (pau_wr_data_o[1] !== 16'h9001 || pau_wr_data_o[2] !== 16'h9002)
            $fatal(1, "CMI latency-10 write data mismatch");

        $display("TB PASS");
        $finish;
    end

endmodule
