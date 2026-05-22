`timescale 1ns/1ps

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pau_controller_tb;

    localparam int NUM_POLYS          = qrem_global_pkg::NUM_POLYS;
    localparam int POLY_W             = $clog2(NUM_POLYS);
    localparam int CWM_NUM_TERMS      = 3;
    localparam int CWM_PAIRS_PER_TERM = 128;

    logic       clk;
    logic       rst;
    logic       start_i;
    pe_mode_e   op_type_i;
    logic [POLY_W-1:0] poly_id_i;

    logic       ready_o;
    logic       done_o;
    logic       tf_start_o;
    logic [1:0] pass_idx_o;
    pe_mode_e   pe_ctrl_o;
    logic       pe_valid_o;

    logic       mac_issue_o;
    logic       mac_first_term_o;
    logic [6:0] mac_pair_idx_o;
    logic       mac_drain_issue_o;
    logic [6:0] mac_drain_idx_o;
    logic       mac_fuse_e_o;
    logic       mac_drain_accept_i;

    logic       cmi_ready_i;
    logic       cmi_v_o;
    logic       cmi_rd_en_o;
    logic [POLY_W-1:0] cmi_poly_id_o;
    logic [3:0][7:0] cmi_coeff_idx_o;
    logic [3:0]      cmi_coeff_valid_o;
    logic [3:0]      cmi_wb_latency_o;

    logic [5:0] block_cnt_o;
    logic [5:0] bf_cnt_o;

    logic [POLY_W-1:0] aux_poly_id_i;
    logic [POLY_W-1:0] cwm_num_terms_i;
    logic tf_step_o;
    logic pass_is_radix2_o;
    logic [POLY_W-1:0] cmi_aux_poly_id_o;
    logic cmi_aux_v_o;
    logic cmi_aux_rd_en_o;

    integer pass_count = 0;
    integer fail_count = 0;

    pau_controller #(
        .NUM_POLYS(NUM_POLYS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start_i(start_i),
        .op_type_i(op_type_i),
        .poly_id_i(poly_id_i),
        .aux_poly_id_i(aux_poly_id_i),
        .cwm_num_terms_i(cwm_num_terms_i),
        .ready_o(ready_o),
        .done_o(done_o),
        .tf_start_o(tf_start_o),
        .tf_step_o(tf_step_o),
        .pass_is_radix2_o(pass_is_radix2_o),
        .pass_idx_o(pass_idx_o),
        .pe_ctrl_o(pe_ctrl_o),
        .pe_valid_o(pe_valid_o),
        .mac_issue_o(mac_issue_o),
        .mac_first_term_o(mac_first_term_o),
        .mac_pair_idx_o(mac_pair_idx_o),
        .mac_drain_issue_o(mac_drain_issue_o),
        .mac_drain_idx_o(mac_drain_idx_o),
        .mac_fuse_e_o(mac_fuse_e_o),
        .mac_drain_accept_i(mac_drain_accept_i),
        .cmi_ready_i(cmi_ready_i),
        .cmi_v_o(cmi_v_o),
        .cmi_rd_en_o(cmi_rd_en_o),
        .cmi_poly_id_o(cmi_poly_id_o),
        .cmi_aux_poly_id_o(cmi_aux_poly_id_o),
        .cmi_coeff_idx_o(cmi_coeff_idx_o),
        .cmi_coeff_valid_o(cmi_coeff_valid_o),
        .cmi_wb_latency_o(cmi_wb_latency_o),
        .cmi_aux_v_o(cmi_aux_v_o),
        .cmi_aux_rd_en_o(cmi_aux_rd_en_o),
        .block_cnt_o(block_cnt_o),
        .bf_cnt_o(bf_cnt_o)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task automatic check(input logic cond, input string msg);
        begin
            if (cond) begin
                $display("[%0t] PASS: %s", $time, msg);
                pass_count++;
            end else begin
                $display("[%0t] FAIL: %s", $time, msg);
                fail_count++;
            end
        end
    endtask

    task automatic reset_dut;
        begin
            rst                = 1'b1;
            start_i            = 1'b0;
            op_type_i          = PE_MODE_NTT;
            poly_id_i          = POLY_W'(3);
            aux_poly_id_i      = POLY_W'(4);
            cwm_num_terms_i    = POLY_W'(3);
            cmi_ready_i        = 1'b1;
            mac_drain_accept_i = 1'b1;
            repeat (3) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
            #1;
        end
    endtask

    task automatic start_op(input pe_mode_e mode);
        begin
            @(negedge clk);
            op_type_i = mode;
            start_i   = 1'b1;
            @(negedge clk);
            start_i   = 1'b0;
        end
    endtask

    task automatic wait_done(input integer max_cycles, input string opname);
        integer cyc;
        begin
            cyc = 0;
            while ((done_o !== 1'b1) && (cyc < max_cycles)) begin
                @(posedge clk);
                cyc++;
            end
            check(done_o === 1'b1, {opname, " completed before timeout"});
            if (done_o === 1'b1)
                $display("[%0t] INFO: %s finished in %0d cycles", $time, opname, cyc);
            @(posedge clk);
            #1;
            check(ready_o === 1'b1, {opname, " returned to IDLE"});
        end
    endtask

    task automatic test_reset_idle;
        begin
            $display("\n=== TEST 1: RESET / IDLE ===");
            reset_dut();

            check(ready_o === 1'b1, "ready_o is high in IDLE after reset");
            check(done_o === 1'b0, "done_o is low after reset");
            check(pe_valid_o === 1'b0, "pe_valid_o is low in IDLE");
            check(cmi_v_o === 1'b0, "cmi_v_o is low in IDLE");
            check(cmi_rd_en_o === 1'b0, "cmi_rd_en_o is low in IDLE");
        end
    endtask

    task automatic test_ntt;
        integer cycle_idx;
        logic saw_issue;
        logic saw_nonzero_idx;
        begin
            $display("\n=== TEST 2: NTT PRIMARY CMI ISSUE ===");
            reset_dut();

            saw_issue = 1'b0;
            saw_nonzero_idx = 1'b0;
            start_op(PE_MODE_NTT);

            #1;
            check(tf_start_o === 1'b1, "tf_start_o pulses during NTT setup");
            check(cmi_poly_id_o === POLY_W'(3), "poly_id is forwarded to CMI");

            for (cycle_idx = 0; cycle_idx < 32; cycle_idx++) begin
                @(posedge clk);
                #1;
                if (cmi_v_o && cmi_rd_en_o && cmi_coeff_valid_o == 4'b1111)
                    saw_issue = 1'b1;
                if (cmi_coeff_idx_o[0] != 8'd0)
                    saw_nonzero_idx = 1'b1;
            end

            check(saw_issue, "NTT issues primary CMI reads");
            check(saw_nonzero_idx, "NTT coefficient index stream advances");
            check(pe_valid_o === 1'b1, "pe_valid_o follows accepted CMI reads");

            wait_done(5000, "NTT");
        end
    endtask

    task automatic test_intt;
        begin
            $display("\n=== TEST 3: INTT PRIMARY CMI ISSUE ===");
            reset_dut();
            start_op(PE_MODE_INTT);

            #1;
            check(tf_start_o === 1'b1, "tf_start_o pulses during INTT setup");
            @(posedge clk);
            #1;
            check(pe_ctrl_o == PE_MODE_INTT, "pe_ctrl_o latches INTT");
            check(cmi_v_o && cmi_rd_en_o, "INTT issues primary CMI reads");

            wait_done(5000, "INTT");
        end
    endtask

    task automatic test_addsub_primary_only_marker;
        begin
            $display("\n=== TEST 4: ADDSUB PRIMARY-ONLY MARKER ===");
            reset_dut();
            start_op(PE_MODE_ADDSUB);

            @(posedge clk);
            #1;
            check(tf_start_o === 1'b0, "tf_start_o stays low for ADDSUB");
            @(posedge clk);
            #1;
            check(pe_ctrl_o == PE_MODE_ADDSUB, "pe_ctrl_o latches ADDSUB");
            check(cmi_v_o && cmi_rd_en_o, "ADDSUB currently issues only primary reads");
            check(cmi_coeff_valid_o === 4'b1111, "ADDSUB requests four primary lanes");

            wait_done(1000, "ADDSUB");
        end
    endtask

    task automatic test_cwm_marker;
        integer cycle_idx;
        integer accum_issue_count;
        integer accum_term_idx;
        integer accum_pair_idx;
        logic [6:0] expected_pair;
        logic [POLY_W-1:0] expected_term_slot;
        logic saw_accum_issue;
        logic saw_drain_issue;
        logic saw_done;
        begin
            $display("\n=== TEST 5: CWM MULTI-TERM CONTROL ===");
            reset_dut();

            saw_accum_issue   = 1'b0;
            saw_drain_issue   = 1'b0;
            saw_done          = 1'b0;
            accum_issue_count = 0;
            start_op(PE_MODE_CWM);

            for (cycle_idx = 0; cycle_idx < 900; cycle_idx++) begin
                @(posedge clk);
                #1;

                if (mac_issue_o && cmi_v_o && cmi_rd_en_o &&
                    cmi_coeff_valid_o == 4'b0011) begin
                    saw_accum_issue    = 1'b1;
                    accum_term_idx     = accum_issue_count / CWM_PAIRS_PER_TERM;
                    accum_pair_idx     = accum_issue_count % CWM_PAIRS_PER_TERM;
                    expected_pair      = 7'(accum_pair_idx);
                    expected_term_slot = POLY_W'(accum_term_idx);

                    if ((accum_term_idx < 3) &&
                        ((accum_pair_idx < 2) || (accum_pair_idx == (CWM_PAIRS_PER_TERM-1)))) begin
                        $display("[DBG %0t] CWM term=%0d pair=%0d first_term=%0b poly_sel=%0d issue=%0b",
                                 $time, accum_term_idx, mac_pair_idx_o, mac_first_term_o,
                                 cmi_poly_id_o, mac_issue_o);
                    end

                    if (mac_pair_idx_o !== expected_pair) begin
                        $display("[%0t] FAIL: CWM pair index mismatch exp=%0d got=%0d",
                                 $time, expected_pair, mac_pair_idx_o);
                        fail_count++;
                    end
                    if (mac_first_term_o !== (accum_issue_count == 0)) begin
                        $display("[%0t] FAIL: CWM first_term mismatch issue=%0d term=%0d got=%0b",
                                 $time, accum_issue_count, accum_term_idx, mac_first_term_o);
                        fail_count++;
                    end
                    if (cmi_poly_id_o !== expected_term_slot) begin
                        $display("[%0t] FAIL: CWM term slot mismatch exp=%0d got=%0d",
                                 $time, expected_term_slot, cmi_poly_id_o);
                        fail_count++;
                    end

                    accum_issue_count++;
                end

                if (mac_drain_issue_o && mac_fuse_e_o && cmi_v_o &&
                    cmi_rd_en_o && cmi_coeff_valid_o == 4'b0011)
                    saw_drain_issue = 1'b1;
                if (done_o) begin
                    saw_done = 1'b1;
                    cycle_idx = 900;
                end
            end

            check(saw_accum_issue, "CWM accumulation issues lane-0/1 primary reads");
            check(accum_issue_count == (CWM_NUM_TERMS * CWM_PAIRS_PER_TERM),
                  "CWM issues one full pair sweep per term before drain");
            check(saw_drain_issue, "CWM drain issues lane-0/1 primary reads");
            check(saw_done, "CWM completed before timeout");
            @(posedge clk);
            #1;
            check(ready_o === 1'b1, "CWM returned to IDLE");
        end
    endtask

    task automatic test_backpressure_hold;
        begin
            $display("\n=== TEST 6: CMI BACKPRESSURE ===");
            reset_dut();
            start_op(PE_MODE_NTT);

            @(posedge clk);
            @(posedge clk);
            #1;
            cmi_ready_i = 1'b0;
            repeat (3) @(posedge clk);
            #1;
            check(cmi_v_o === 1'b1, "controller keeps request visible while CMI is not ready");
            check(cmi_rd_en_o === 1'b0, "controller does not fire reads while stalled");

            cmi_ready_i = 1'b1;
            @(posedge clk);
            #1;
            check(cmi_rd_en_o === 1'b1, "controller resumes reads after CMI ready");
        end
    endtask

    initial begin
        $display("==================================================");
        $display("Starting pau_controller testbench");
        $display("==================================================");

        test_reset_idle();
        test_ntt();
        test_intt();
        test_addsub_primary_only_marker();
        test_cwm_marker();
        test_backpressure_hold();

        $display("--------------------------------------------------");
        $display("TEST SUMMARY: PASS=%0d  FAIL=%0d", pass_count, fail_count);
        $display("--------------------------------------------------");
        if (fail_count != 0)
            $fatal(1, "pau_controller_tb failed");

        $display("TB PASS");
        $finish;
    end

endmodule
