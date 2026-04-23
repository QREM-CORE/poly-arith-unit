`timescale 1ns/1ps

import poly_arith_pkg::*;

module mac_row_accum_tb;

    localparam int NUM_PAIRS = 4;
    localparam logic [6:0] PAIR_0 = 7'd0;
    localparam logic [6:0] PAIR_P = 7'd1;
    localparam logic [6:0] PAIR_Q = 7'd2;
    localparam logic [6:0] PAIR_LAST = 7'd3;

    logic       clk;
    logic       rst;
    logic       acc_fire_i;
    logic       first_term_i;
    logic [6:0] pair_idx_i;
    coeff_t     cwm0_i;
    coeff_t     cwm1_i;
    logic       drain_req_i;
    logic [6:0] drain_idx_i;
    logic       fuse_e_i;
    coeff_t     e0_i;
    coeff_t     e1_i;
    logic       drain_ready_i;
    logic       drain_accept_o;
    logic       drain_valid_o;
    logic [6:0] drain_pair_idx_o;
    coeff_t     drain0_o;
    coeff_t     drain1_o;

    int error_count = 0;

    mac_row_accum #(
        .NUM_PAIRS(NUM_PAIRS)
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .acc_fire_i      (acc_fire_i),
        .first_term_i    (first_term_i),
        .pair_idx_i      (pair_idx_i),
        .cwm0_i          (cwm0_i),
        .cwm1_i          (cwm1_i),
        .drain_req_i     (drain_req_i),
        .drain_idx_i     (drain_idx_i),
        .fuse_e_i        (fuse_e_i),
        .e0_i            (e0_i),
        .e1_i            (e1_i),
        .drain_ready_i   (drain_ready_i),
        .drain_accept_o  (drain_accept_o),
        .drain_valid_o   (drain_valid_o),
        .drain_pair_idx_o(drain_pair_idx_o),
        .drain0_o        (drain0_o),
        .drain1_o        (drain1_o)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    function automatic coeff_t gm_mod_add(input coeff_t a, input coeff_t b);
        logic [12:0] sum;
        sum = a + b;
        return (sum >= Q) ? coeff_t'(sum - Q) : coeff_t'(sum);
    endfunction

    task automatic clear_inputs;
        begin
            acc_fire_i    = 1'b0;
            first_term_i  = 1'b0;
            pair_idx_i    = '0;
            cwm0_i        = '0;
            cwm1_i        = '0;
            drain_req_i   = 1'b0;
            drain_idx_i   = '0;
            fuse_e_i      = 1'b0;
            e0_i          = '0;
            e1_i          = '0;
        end
    endtask

    task automatic expect_coeff(
        input string label,
        input coeff_t got,
        input coeff_t exp
    );
        begin
            if (got !== exp) begin
                $display("[FAIL] %s exp=%0d got=%0d", label, exp, got);
                error_count++;
            end
        end
    endtask

    task automatic expect_bit(
        input string label,
        input logic got,
        input logic exp
    );
        begin
            if (got !== exp) begin
                $display("[FAIL] %s exp=%0b got=%0b", label, exp, got);
                error_count++;
            end
        end
    endtask

    task automatic check_scratch(
        input logic [6:0] idx,
        input coeff_t exp0,
        input coeff_t exp1,
        input string label
    );
        begin
            expect_coeff({label, " acc0_mem"}, dut.acc0_mem[idx], exp0);
            expect_coeff({label, " acc1_mem"}, dut.acc1_mem[idx], exp1);
        end
    endtask

    task automatic dump_acc_debug(
        input string      label,
        input logic [6:0] test_idx
    );
        begin
            $display("[DBG] %s pair_idx=%0d first_term=%0b acc_fire=%0b cwm0=%0d cwm1=%0d acc0_old=%0d acc1_old=%0d acc0_sum=%0d acc1_sum=%0d acc0_mem[test_idx]=%0d acc1_mem[test_idx]=%0d",
                     label, pair_idx_i, first_term_i, acc_fire_i, cwm0_i, cwm1_i,
                     dut.acc0_old, dut.acc1_old, dut.acc0_sum, dut.acc1_sum,
                     dut.acc0_mem[test_idx], dut.acc1_mem[test_idx]);
        end
    endtask


    task automatic drive_acc(
        input logic       first_term,
        input logic [6:0] idx,
        input coeff_t     c0,
        input coeff_t     c1,
        input logic       check_old,
        input coeff_t     exp_old0,
        input coeff_t     exp_old1,
        input coeff_t     exp_new0,
        input coeff_t     exp_new1,
        input string      label
    );
        begin
            @(negedge clk);
            acc_fire_i   = 1'b1;
            first_term_i = first_term;
            pair_idx_i   = idx;
            cwm0_i       = c0;
            cwm1_i       = c1;

            #1;
            dump_acc_debug({label, " pre"}, idx);
            if (check_old) begin
                expect_coeff({label, " acc0_old"}, dut.acc0_old, exp_old0);
                expect_coeff({label, " acc1_old"}, dut.acc1_old, exp_old1);
            end

            @(posedge clk);
            #1;
            dump_acc_debug({label, " post"}, idx);
            check_scratch(idx, exp_new0, exp_new1, label);

            clear_inputs();
        end
    endtask

    task automatic drive_drain(
        input logic [6:0] idx,
        input logic       fuse,
        input coeff_t     e0,
        input coeff_t     e1,
        input coeff_t     exp0,
        input coeff_t     exp1,
        input string      label
    );
        begin
            @(negedge clk);
            drain_req_i = 1'b1;
            drain_idx_i = idx;
            fuse_e_i    = fuse;
            e0_i        = e0;
            e1_i        = e1;

            #1;
            expect_bit({label, " drain_accept"}, drain_accept_o, 1'b1);

            @(posedge clk);
            #1;
            expect_bit({label, " drain_valid"}, drain_valid_o, 1'b1);
            expect_coeff({label, " drain_pair_idx"}, coeff_t'({5'd0, drain_pair_idx_o}), coeff_t'({5'd0, idx}));
            expect_coeff({label, " drain0"}, drain0_o, exp0);
            expect_coeff({label, " drain1"}, drain1_o, exp1);

            clear_inputs();

            @(posedge clk);
            #1;
            expect_bit({label, " drain_valid_clear"}, drain_valid_o, 1'b0);
        end
    endtask

    initial begin
        rst           = 1'b1;
        drain_ready_i = 1'b1;
        clear_inputs();

        repeat (3) @(posedge clk);
        rst = 1'b0;
        repeat (1) @(posedge clk);

        // first_term_i is now a 1-cycle "start new row" pulse. The module
        // keeps seeding until the first sweep reaches PAIR_LAST.
        drive_acc(1'b1, PAIR_0,   12'd10, 12'd20, 1'b1, '0, '0, 12'd10, 12'd20,
                  "test1_seed_pair0_start");
        drive_acc(1'b0, PAIR_P,   12'd7,  12'd8,  1'b1, '0, '0, 12'd7,  12'd8,
                  "test1_seed_pair1_mid");
        drive_acc(1'b0, PAIR_Q,   12'd9,  12'd10, 1'b1, '0, '0, 12'd9,  12'd10,
                  "test1_seed_pair2_mid");
        drive_acc(1'b0, PAIR_LAST,12'd11, 12'd12, 1'b1, '0, '0, 12'd11, 12'd12,
                  "test1_seed_pair3_last");

        // Once the first sweep has touched every pair slot, revisits must use
        // the stored partial sums instead of reseeding from scratch.
        drive_acc(1'b0, PAIR_P, 12'd3, 12'd4, 1'b1, 12'd7, 12'd8, 12'd10, 12'd12,
                  "test2_acc_pair1_revisit");
        drive_acc(1'b0, PAIR_Q, 12'd5, 12'd6, 1'b1, 12'd9, 12'd10, 12'd14, 12'd16,
                  "test3_acc_pair2_revisit");
        check_scratch(PAIR_0,    12'd10, 12'd20, "test4_pair0_unchanged");
        check_scratch(PAIR_LAST, 12'd11, 12'd12, "test4_pair3_unchanged");

        // Prove drain can immediately see the accumulated pair and optional +e_hat fuse.
        drive_drain(PAIR_Q, 1'b0, '0, '0, 12'd14, 12'd16, "test5_drain_q_plain");
        drive_drain(PAIR_P, 1'b0, '0, '0, 12'd10, 12'd12, "test6_drain_p_plain");
        drive_drain(PAIR_P, 1'b1, 12'd3315, 12'd3305,
                    gm_mod_add(12'd10, 12'd3315),
                    gm_mod_add(12'd12, 12'd3305),
                    "test6_drain_p_fused");

        if (error_count == 0) begin
            $display("[PASS] mac_row_accum directed tests passed");
        end else begin
            $display("[FAIL] mac_row_accum directed tests failed: %0d errors", error_count);
            $fatal(1);
        end

        $finish;
    end

endmodule
