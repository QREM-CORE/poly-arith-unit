`timescale 1ns/1ps

import poly_arith_pkg::*;
import qrem_mem_map_pkg::*;
import qrem_seed_map_pkg::*;

module pau_top_intt_tb;

    localparam int NUM_POLYS  = 32;
    localparam int NCOEFF     = 256;
    localparam int W          = 16;
    localparam int SEED_DEPTH = 32;
    localparam int SEED_W     = 64;
    localparam int MEM_WORD_W = STORE_WIDTH;
    localparam int POLY_W     = $clog2(NUM_POLYS);
    localparam int COEFF_W    = $clog2(NCOEFF);
    localparam int SEED_IDX_W = $clog2(QREM_SEED_BEATS);
    localparam int MAX_CYCLES = 1200;

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
    logic [3:0][MEM_WORD_W-1:0] pau_wr_data_o;
    logic       pau_rd_valid_i;
    logic [POLY_W-1:0] pau_rd_poly_id_i;
    logic [3:0][7:0] pau_rd_idx_i;
    logic [3:0]      pau_rd_lane_valid_i;
    logic [3:0][MEM_WORD_W-1:0] pau_rd_data_i;
    logic       pau_stall_i;

    logic       pau_aux_req_o;
    logic       pau_aux_rd_en_o;
    logic [POLY_W-1:0] pau_aux_rd_poly_id_o;
    logic [3:0][7:0] pau_aux_rd_idx_o;
    logic [3:0]      pau_aux_rd_lane_valid_o;
    logic [3:0]      pau_aux_wr_en_o;
    logic [POLY_W-1:0] pau_aux_wr_poly_id_o;
    logic [3:0][7:0] pau_aux_wr_idx_o;
    logic [3:0][MEM_WORD_W-1:0] pau_aux_wr_data_o;
    logic       pau_aux_rd_valid_i;
    logic [POLY_W-1:0] pau_aux_rd_poly_id_i;
    logic [3:0][7:0] pau_aux_rd_idx_i;
    logic [3:0]      pau_aux_rd_lane_valid_i;
    logic [3:0][MEM_WORD_W-1:0] pau_aux_rd_data_i;

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
    logic [3:0][MEM_WORD_W-1:0] hsu_wr_data;
    logic hsu_rd_valid;
    logic [POLY_W-1:0] hsu_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0] hsu_rd_idx_o;
    logic [3:0] hsu_rd_lane_valid_o;
    logic [3:0][MEM_WORD_W-1:0] hsu_rd_data;
    logic hsu_stall;

    logic tr_req;
    logic tr_rd_en;
    logic [POLY_W-1:0] tr_rd_poly_id;
    logic [3:0][COEFF_W-1:0] tr_rd_idx;
    logic [3:0] tr_rd_lane_valid;
    logic [3:0] tr_wr_en;
    logic [POLY_W-1:0] tr_wr_poly_id;
    logic [3:0][COEFF_W-1:0] tr_wr_idx;
    logic [3:0][MEM_WORD_W-1:0] tr_wr_data;
    logic tr_rd_valid;
    logic [POLY_W-1:0] tr_rd_poly_id_o;
    logic [3:0][COEFF_W-1:0] tr_rd_idx_o;
    logic [3:0] tr_rd_lane_valid_o;
    logic [3:0][MEM_WORD_W-1:0] tr_rd_data;
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

    logic [MEM_WORD_W-1:0] model_mem [0:NUM_POLYS-1][0:NCOEFF-1];
    logic [MEM_WORD_W-1:0] init_mem  [0:NUM_POLYS-1][0:NCOEFF-1];

    logic                  exp_rd_pending_r;
    logic [POLY_W-1:0]     exp_rd_poly_id_r;
    logic [3:0][7:0]       exp_rd_idx_r;
    logic [3:0]            exp_rd_lane_valid_r;
    logic [3:0][MEM_WORD_W-1:0] exp_rd_data_r;

    int cycle_count;
    int fail_count;
    int stall_count;
    int read_count;
    int write_count;
    int write_issue_count_by_pass [0:3];
    int issue_count_by_pass [0:3];
    int wb_count_by_pass [0:3];
    bit wb_seen_by_pass [0:3];
    int wb_gap_count_by_pass [0:3];

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
        .NUM_POLYS         (NUM_POLYS),
        .NCOEFF            (NCOEFF),
        .W                 (W),
        .COEFF_W           (MEM_WORD_W),
        .SEED_DEPTH        (SEED_DEPTH),
        .SEED_W            (SEED_W),
        .POLY_PRELOAD_EN   (1'b1),
        .POLY_PRELOAD_MODE (0)
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

    task automatic clear_other_clients;
        begin
            wipe_i             = 1'b0;
            hsu_hash_ek_read_en = 1'b0;
            hsu_req            = 1'b0;
            hsu_rd_en          = 1'b0;
            hsu_rd_poly_id     = '0;
            hsu_rd_idx         = '0;
            hsu_rd_lane_valid  = '0;
            hsu_wr_en          = '0;
            hsu_wr_poly_id     = '0;
            hsu_wr_idx         = '0;
            hsu_wr_data        = '0;
            tr_req             = 1'b0;
            tr_rd_en           = 1'b0;
            tr_rd_poly_id      = '0;
            tr_rd_idx          = '0;
            tr_rd_lane_valid   = '0;
            tr_wr_en           = '0;
            tr_wr_poly_id      = '0;
            tr_wr_idx          = '0;
            tr_wr_data         = '0;
            hsu_seed_req       = 1'b0;
            hsu_seed_we        = 1'b0;
            hsu_seed_id        = SEED_ID_RHO;
            hsu_seed_idx       = '0;
            hsu_seed_wdata     = '0;
            tr_seed_req        = 1'b0;
            tr_seed_we         = 1'b0;
            tr_seed_id         = SEED_ID_RHO;
            tr_seed_idx        = '0;
            tr_seed_wdata      = '0;
        end
    endtask

    task automatic init_model_mem;
        int pid;
        int order;
        begin
            for (pid = 0; pid < NUM_POLYS; pid++) begin
                for (order = 0; order < NCOEFF; order++) begin
                    model_mem[pid][order] = MEM_WORD_W'((pid * NCOEFF) + order);
                    init_mem[pid][order]  = MEM_WORD_W'((pid * NCOEFF) + order);
                end
            end
        end
    endtask

    task automatic run_op(input pe_mode_e mode, input string name);
        int start_cycle;
        begin
            op_type_i   = mode;
            start_i     = 1'b1;
            @(posedge clk);
            start_i     = 1'b0;

            start_cycle = cycle_count;
            while (!dut.ctl_done && ((cycle_count - start_cycle) < MAX_CYCLES))
                @(posedge clk);

            if (!dut.ctl_done)
                fail($sformatf("%s run timed out after %0d local cycles",
                               name, cycle_count - start_cycle));

            @(posedge clk);
        end
    endtask

    task automatic fail(input string msg);
        begin
            fail_count++;
            $display("FAIL @%0t: %s", $time, msg);
        end
    endtask

    logic                  next_exp_rd_pending;
    logic [POLY_W-1:0]     next_exp_rd_poly_id;
    logic [3:0][7:0]       next_exp_rd_idx;
    logic [3:0]            next_exp_rd_lane_valid;
    logic [3:0][MEM_WORD_W-1:0] next_exp_rd_data;

    always_ff @(posedge clk) begin
        int lane;
        int pass_q;

        if (rst) begin
            for (int pass = 0; pass < 4; pass++) begin
                issue_count_by_pass[pass]  <= 0;
                wb_count_by_pass[pass]     <= 0;
                write_issue_count_by_pass[pass] <= 0;
                wb_seen_by_pass[pass]      <= 1'b0;
                wb_gap_count_by_pass[pass] <= 0;
            end
            exp_rd_pending_r    <= 1'b0;
            exp_rd_poly_id_r    <= '0;
            exp_rd_idx_r        <= '0;
            exp_rd_lane_valid_r <= '0;
            exp_rd_data_r       <= '0;
            cycle_count         <= 0;
            stall_count         <= 0;
            read_count          <= 0;
            write_count         <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
            pass_q = dut.pass_idx;

            if (pau_stall_i)
                stall_count <= stall_count + 1;

            if (dut.pe_valid)
                issue_count_by_pass[pass_q] <= issue_count_by_pass[pass_q] + 1;

            if (dut.pe_wb_valid) begin
                wb_seen_by_pass[pass_q]  <= 1'b1;
                wb_count_by_pass[pass_q] <= wb_count_by_pass[pass_q] + 1;

                if (|pau_wr_en_o) begin
                    write_issue_count_by_pass[pass_q] <= write_issue_count_by_pass[pass_q] + 1;
                end else begin
                    $display("WB drop @%0t pass=%0d state=%0d block=%0d bf=%0d idx=%p",
                             $time, dut.pass_idx, dut.u_controller.state_r,
                             dut.u_controller.block_cnt_o, dut.u_controller.bf_cnt_o,
                             pau_wr_idx_o);
                end
            end else if (wb_seen_by_pass[pass_q] &&
                         (wb_count_by_pass[pass_q] < 64) &&
                         (dut.u_controller.state_r != dut.u_controller.S_NEXT_PASS) &&
                         (dut.u_controller.state_r != dut.u_controller.S_DONE)) begin
                wb_gap_count_by_pass[pass_q] <= wb_gap_count_by_pass[pass_q] + 1;
            end

            if (exp_rd_pending_r !== pau_rd_valid_i) begin
                fail($sformatf(
                    "read-valid mismatch exp=%0b got=%0b pass=%0d block=%0d bf=%0d",
                    exp_rd_pending_r, pau_rd_valid_i, dut.pass_idx, dut.u_controller.block_cnt_o,
                    dut.u_controller.bf_cnt_o));
            end

            if (pau_rd_valid_i) begin
                read_count <= read_count + 1;

                if (pau_rd_poly_id_i !== exp_rd_poly_id_r)
                    fail($sformatf("read poly mismatch exp=%0d got=%0d",
                                   exp_rd_poly_id_r, pau_rd_poly_id_i));

                if (pau_rd_idx_i !== exp_rd_idx_r)
                    fail($sformatf("read idx mismatch exp=%p got=%p",
                                   exp_rd_idx_r, pau_rd_idx_i));

                if (pau_rd_lane_valid_i !== exp_rd_lane_valid_r)
                    fail($sformatf("read lane-valid mismatch exp=%b got=%b",
                                   exp_rd_lane_valid_r, pau_rd_lane_valid_i));

                for (lane = 0; lane < 4; lane++) begin
                    if (exp_rd_lane_valid_r[lane] &&
                        (pau_rd_data_i[lane] !== exp_rd_data_r[lane])) begin
                        fail($sformatf(
                            "read data mismatch lane=%0d exp=%0h got=%0h pass=%0d block=%0d bf=%0d idx=%0d",
                            lane, exp_rd_data_r[lane], pau_rd_data_i[lane], dut.pass_idx,
                            dut.u_controller.block_cnt_o, dut.u_controller.bf_cnt_o, exp_rd_idx_r[lane]));
                    end

                    if (exp_rd_lane_valid_r[lane] &&
                        (dut.coeff_from_cmi[lane] !== exp_rd_data_r[lane])) begin
                        fail($sformatf(
                            "CMI coeff mismatch lane=%0d exp=%0h got=%0h pass=%0d block=%0d bf=%0d idx=%0d",
                            lane, exp_rd_data_r[lane], dut.coeff_from_cmi[lane], dut.pass_idx,
                            dut.u_controller.block_cnt_o, dut.u_controller.bf_cnt_o, exp_rd_idx_r[lane]));
                    end
                end
            end

            next_exp_rd_pending    = 1'b0;
            next_exp_rd_poly_id    = '0;
            next_exp_rd_idx        = '0;
            next_exp_rd_lane_valid = '0;
            next_exp_rd_data       = '0;

            if (pau_req_o && pau_rd_en_o && (|pau_rd_lane_valid_o) && !pau_stall_i) begin
                next_exp_rd_pending    = 1'b1;
                next_exp_rd_poly_id    = pau_rd_poly_id_o;
                next_exp_rd_idx        = pau_rd_idx_o;
                next_exp_rd_lane_valid = pau_rd_lane_valid_o;
                for (lane = 0; lane < 4; lane++) begin
                    if (pau_rd_lane_valid_o[lane])
                        next_exp_rd_data[lane] = model_mem[pau_rd_poly_id_o][pau_rd_idx_o[lane]];
                end
            end

            for (lane = 0; lane < 4; lane++) begin
                if (pau_wr_en_o[lane] && !pau_stall_i) begin
                    model_mem[pau_wr_poly_id_o][pau_wr_idx_o[lane]] <= pau_wr_data_o[lane];
                    write_count <= write_count + 1;
                end
            end

            exp_rd_pending_r    <= next_exp_rd_pending;
            exp_rd_poly_id_r    <= next_exp_rd_poly_id;
            exp_rd_idx_r        <= next_exp_rd_idx;
            exp_rd_lane_valid_r <= next_exp_rd_lane_valid;
            exp_rd_data_r       <= next_exp_rd_data;
        end
    end

    initial begin
        int pass;
        int idx;

        fail_count = 0;
        init_model_mem();
        clear_other_clients();
        rst       = 1'b1;
        start_i   = 1'b0;
        op_type_i = PE_MODE_NTT;
        repeat (3) @(posedge clk);

        rst = 1'b0;
        @(posedge clk);

        run_op(PE_MODE_NTT, "NTT");
        run_op(PE_MODE_INTT, "INTT");

        if (mem_fault_o)
            fail($sformatf("Memory fault code=%0d", mem_fault_code_o));

        for (idx = 0; idx < NCOEFF; idx++) begin
            if (model_mem[0][idx] !== init_mem[0][idx]) begin
                fail($sformatf(
                    "NTT->INTT mismatch at coeff %0d exp=%0h got=%0h",
                    idx, init_mem[0][idx], model_mem[0][idx]));
                if (idx > 31)
                    break;
            end
        end

        $display("Final poly0 [0:31]:");
        for (idx = 0; idx < 32; idx = idx + 4) begin
            $display("  [%0d:%0d] = %0h %0h %0h %0h",
                     idx, idx + 3,
                     model_mem[0][idx + 0], model_mem[0][idx + 1],
                     model_mem[0][idx + 2], model_mem[0][idx + 3]);
        end

        for (pass = 0; pass < 4; pass++) begin
            if (issue_count_by_pass[pass] != 128)
                fail($sformatf("pass %0d issue count exp=128 got=%0d",
                               pass, issue_count_by_pass[pass]));
            if (wb_count_by_pass[pass] != 128)
                fail($sformatf("pass %0d wb-valid count exp=128 got=%0d",
                               pass, wb_count_by_pass[pass]));
        end

        $display("NTT->INTT summary: cycles=%0d stalls=%0d reads=%0d writes=%0d fails=%0d",
                 cycle_count, stall_count, read_count, write_count, fail_count);
        $display("NTT->INTT per-pass: issue=%p wb=%p wr=%p wb_gaps=%p",
                 issue_count_by_pass, wb_count_by_pass, write_issue_count_by_pass,
                 wb_gap_count_by_pass);

        if (fail_count == 0)
            $display("TB PASS");
        else
            $display("TB FAIL");
        $finish;
    end

endmodule
