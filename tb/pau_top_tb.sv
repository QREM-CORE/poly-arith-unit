`timescale 1ns/1ps

import poly_arith_pkg::*;
import qrem_mem_map_pkg::*;
import qrem_seed_map_pkg::*;

module pau_top_tb;

    localparam int NUM_POLYS  = 32;
    localparam int NCOEFF     = 256;
    localparam int W          = 16;
    localparam int SEED_DEPTH = 32;
    localparam int SEED_W     = 64;
    localparam int POLY_W     = $clog2(NUM_POLYS);
    localparam int COEFF_W    = $clog2(NCOEFF);
    localparam int SEED_IDX_W = $clog2(QREM_SEED_BEATS);

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

    logic       pau_aux_req_o;
    logic       pau_aux_rd_en_o;
    logic [POLY_W-1:0] pau_aux_rd_poly_id_o;
    logic [3:0][7:0] pau_aux_rd_idx_o;
    logic [3:0]      pau_aux_rd_lane_valid_o;
    logic [3:0]      pau_aux_wr_en_o;
    logic [POLY_W-1:0] pau_aux_wr_poly_id_o;
    logic [3:0][7:0] pau_aux_wr_idx_o;
    logic [3:0][15:0] pau_aux_wr_data_o;
    logic       pau_aux_rd_valid_i;
    logic [POLY_W-1:0] pau_aux_rd_poly_id_i;
    logic [3:0][7:0] pau_aux_rd_idx_i;
    logic [3:0]      pau_aux_rd_lane_valid_i;
    logic [3:0][15:0] pau_aux_rd_data_i;

    logic wipe_i;
    logic wipe_busy_o;
    logic wipe_done_o;
    logic mem_fault_o;
    logic [2:0] mem_fault_code_o;

    logic hsu_hash_ek_read_en;
    logic hsu_req;
    logic hsu_rd_en;
    logic [POLY_W-1:0] hsu_rd_poly_id;
    logic [3:0][COEFF_W-1:0] hsu_rd_idx;
    logic [3:0] hsu_rd_lane_valid;
    logic [3:0] hsu_wr_en;
    logic [POLY_W-1:0] hsu_wr_poly_id;
    logic [3:0][COEFF_W-1:0] hsu_wr_idx;
    logic [3:0][W-1:0] hsu_wr_data;
    logic hsu_rd_valid;
    logic [POLY_W-1:0] hsu_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0] hsu_rd_idx_o;
    logic [3:0] hsu_rd_lane_valid_o;
    logic [3:0][W-1:0] hsu_rd_data;
    logic hsu_stall;

    logic tr_req;
    logic tr_rd_en;
    logic [POLY_W-1:0] tr_rd_poly_id;
    logic [3:0][COEFF_W-1:0] tr_rd_idx;
    logic [3:0] tr_rd_lane_valid;
    logic [3:0] tr_wr_en;
    logic [POLY_W-1:0] tr_wr_poly_id;
    logic [3:0][COEFF_W-1:0] tr_wr_idx;
    logic [3:0][W-1:0] tr_wr_data;
    logic tr_rd_valid;
    logic [POLY_W-1:0] tr_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0] tr_rd_idx_o;
    logic [3:0] tr_rd_lane_valid_o;
    logic [3:0][W-1:0] tr_rd_data;
    logic tr_stall;

    logic hsu_seed_req;
    logic hsu_seed_we;
    seed_id_e hsu_seed_id;
    logic [SEED_IDX_W-1:0] hsu_seed_idx;
    logic [SEED_W-1:0] hsu_seed_wdata;
    logic hsu_seed_ready;
    logic hsu_seed_rvalid;
    logic [SEED_W-1:0] hsu_seed_rdata;

    logic tr_seed_req;
    logic tr_seed_we;
    seed_id_e tr_seed_id;
    logic [SEED_IDX_W-1:0] tr_seed_idx;
    logic [SEED_W-1:0] tr_seed_wdata;
    logic tr_seed_ready;
    logic tr_seed_rvalid;
    logic [SEED_W-1:0] tr_seed_rdata;

    bit saw_primary_req;
    bit saw_primary_read;
    bit saw_primary_write;
    bit saw_primary_response;

    pau_top #(
        .NUM_POLYS(NUM_POLYS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start_i),
        .op_type_i(op_type_i),
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

    poly_mem_subsystem #(
        .NUM_POLYS  (NUM_POLYS),
        .NCOEFF     (NCOEFF),
        .W          (W),
        .SEED_DEPTH (SEED_DEPTH),
        .SEED_W     (SEED_W)
    ) u_mem (
        .clk(clk),
        .rst(rst),
        .wipe_i(wipe_i),
        .wipe_busy_o(wipe_busy_o),
        .wipe_done_o(wipe_done_o),
        .mem_fault_o(mem_fault_o),
        .mem_fault_code_o(mem_fault_code_o),
        .pau_req(pau_req_o),
        .pau_rd_en(pau_rd_en_o),
        .pau_rd_poly_id(pau_rd_poly_id_o),
        .pau_rd_idx(pau_rd_idx_o),
        .pau_rd_lane_valid(pau_rd_lane_valid_o),
        .pau_wr_en(pau_wr_en_o),
        .pau_wr_poly_id(pau_wr_poly_id_o),
        .pau_wr_idx(pau_wr_idx_o),
        .pau_wr_data(pau_wr_data_o),
        .pau_rd_valid(pau_rd_valid_i),
        .pau_rd_poly_id_o(pau_rd_poly_id_i),
        .pau_rd_idx_o(pau_rd_idx_i),
        .pau_rd_lane_valid_o(pau_rd_lane_valid_i),
        .pau_rd_data(pau_rd_data_i),
        .pau_stall(pau_stall_i),
        .pau_aux_req(pau_aux_req_o),
        .pau_aux_rd_en(pau_aux_rd_en_o),
        .pau_aux_rd_poly_id(pau_aux_rd_poly_id_o),
        .pau_aux_rd_idx(pau_aux_rd_idx_o),
        .pau_aux_rd_lane_valid(pau_aux_rd_lane_valid_o),
        .pau_aux_wr_en(pau_aux_wr_en_o),
        .pau_aux_wr_poly_id(pau_aux_wr_poly_id_o),
        .pau_aux_wr_idx(pau_aux_wr_idx_o),
        .pau_aux_wr_data(pau_aux_wr_data_o),
        .pau_aux_rd_valid(pau_aux_rd_valid_i),
        .pau_aux_rd_poly_id_o(pau_aux_rd_poly_id_i),
        .pau_aux_rd_idx_o(pau_aux_rd_idx_i),
        .pau_aux_rd_lane_valid_o(pau_aux_rd_lane_valid_i),
        .pau_aux_rd_data(pau_aux_rd_data_i),
        .hsu_hash_ek_read_en(hsu_hash_ek_read_en),
        .hsu_req(hsu_req),
        .hsu_rd_en(hsu_rd_en),
        .hsu_rd_poly_id(hsu_rd_poly_id),
        .hsu_rd_idx(hsu_rd_idx),
        .hsu_rd_lane_valid(hsu_rd_lane_valid),
        .hsu_wr_en(hsu_wr_en),
        .hsu_wr_poly_id(hsu_wr_poly_id),
        .hsu_wr_idx(hsu_wr_idx),
        .hsu_wr_data(hsu_wr_data),
        .hsu_rd_valid(hsu_rd_valid),
        .hsu_rd_poly_id_o(hsu_rd_poly_id_o),
        .hsu_rd_idx_o(hsu_rd_idx_o),
        .hsu_rd_lane_valid_o(hsu_rd_lane_valid_o),
        .hsu_rd_data(hsu_rd_data),
        .hsu_stall(hsu_stall),
        .tr_req(tr_req),
        .tr_rd_en(tr_rd_en),
        .tr_rd_poly_id(tr_rd_poly_id),
        .tr_rd_idx(tr_rd_idx),
        .tr_rd_lane_valid(tr_rd_lane_valid),
        .tr_wr_en(tr_wr_en),
        .tr_wr_poly_id(tr_wr_poly_id),
        .tr_wr_idx(tr_wr_idx),
        .tr_wr_data(tr_wr_data),
        .tr_rd_valid(tr_rd_valid),
        .tr_rd_poly_id_o(tr_rd_poly_id_o),
        .tr_rd_idx_o(tr_rd_idx_o),
        .tr_rd_lane_valid_o(tr_rd_lane_valid_o),
        .tr_rd_data(tr_rd_data),
        .tr_stall(tr_stall),
        .hsu_seed_req(hsu_seed_req),
        .hsu_seed_we(hsu_seed_we),
        .hsu_seed_id(hsu_seed_id),
        .hsu_seed_idx(hsu_seed_idx),
        .hsu_seed_wdata(hsu_seed_wdata),
        .hsu_seed_ready(hsu_seed_ready),
        .hsu_seed_rvalid(hsu_seed_rvalid),
        .hsu_seed_rdata(hsu_seed_rdata),
        .tr_seed_req(tr_seed_req),
        .tr_seed_we(tr_seed_we),
        .tr_seed_id(tr_seed_id),
        .tr_seed_idx(tr_seed_idx),
        .tr_seed_wdata(tr_seed_wdata),
        .tr_seed_ready(tr_seed_ready),
        .tr_seed_rvalid(tr_seed_rvalid),
        .tr_seed_rdata(tr_seed_rdata)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task automatic tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task automatic expect_aux_idle;
        begin
            #1;
            if (pau_aux_req_o || pau_aux_rd_en_o || (|pau_aux_rd_lane_valid_o) ||
                (|pau_aux_wr_en_o) || pau_aux_rd_valid_i)
                $fatal(1, "PAU auxiliary Memory path must remain tied idle in this revision");
            if (pau_aux_rd_poly_id_o !== '0 || pau_aux_rd_idx_o !== '0 ||
                pau_aux_wr_poly_id_o !== '0 || pau_aux_wr_idx_o !== '0 ||
                pau_aux_wr_data_o !== '0)
                $fatal(1, "PAU auxiliary descriptor/data should be zero while idle");
        end
    endtask

    task automatic clear_other_clients;
        begin
            wipe_i = 1'b0;
            hsu_hash_ek_read_en = 1'b0;
            hsu_req = 1'b0;
            hsu_rd_en = 1'b0;
            hsu_rd_poly_id = '0;
            hsu_rd_idx = '0;
            hsu_rd_lane_valid = '0;
            hsu_wr_en = '0;
            hsu_wr_poly_id = '0;
            hsu_wr_idx = '0;
            hsu_wr_data = '0;
            tr_req = 1'b0;
            tr_rd_en = 1'b0;
            tr_rd_poly_id = '0;
            tr_rd_idx = '0;
            tr_rd_lane_valid = '0;
            tr_wr_en = '0;
            tr_wr_poly_id = '0;
            tr_wr_idx = '0;
            tr_wr_data = '0;
            hsu_seed_req = 1'b0;
            hsu_seed_we = 1'b0;
            hsu_seed_id = SEED_ID_D;
            hsu_seed_idx = '0;
            hsu_seed_wdata = '0;
            tr_seed_req = 1'b0;
            tr_seed_we = 1'b0;
            tr_seed_id = SEED_ID_D;
            tr_seed_idx = '0;
            tr_seed_wdata = '0;
        end
    endtask

    always_ff @(posedge clk) begin
        if (rst) begin
            saw_primary_req      <= 1'b0;
            saw_primary_read     <= 1'b0;
            saw_primary_write    <= 1'b0;
            saw_primary_response <= 1'b0;
        end else begin
            if (pau_req_o)
                saw_primary_req <= 1'b1;
            if (pau_req_o && pau_rd_en_o && (|pau_rd_lane_valid_o) && !pau_stall_i)
                saw_primary_read <= 1'b1;
            if (pau_req_o && (|pau_wr_en_o) && !pau_stall_i)
                saw_primary_write <= 1'b1;
            if (pau_rd_valid_i)
                saw_primary_response <= 1'b1;
        end
    end

    initial begin
        rst       = 1'b1;
        start_i   = 1'b0;
        op_type_i = PE_MODE_NTT;
        clear_other_clients();
        repeat (3) tick();
        expect_aux_idle();

        rst = 1'b0;
        tick();
        expect_aux_idle();

        start_i = 1'b1;
        tick();
        start_i = 1'b0;

        repeat (450) begin
            tick();
            expect_aux_idle();
            if (mem_fault_o)
                $fatal(1, "Unexpected Memory fault during PAU NTT smoke");
        end

        if (!saw_primary_req)
            $fatal(1, "PAU top did not issue primary Memory requests");
        if (!saw_primary_read)
            $fatal(1, "PAU top did not issue primary Memory reads");
        if (!saw_primary_write)
            $fatal(1, "PAU top did not issue primary Memory writeback");
        if (!saw_primary_response)
            $fatal(1, "PAU top did not receive primary Memory read data");

        $display("TB PASS");
        $finish;
    end

endmodule
