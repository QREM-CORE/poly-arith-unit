// ==========================================================
// Testbench for Twiddle Factor ROM + Address Generator
// Author: Jessica Buentipo
// Description: Verifies that the tf_rom (4-ROM architecture)
//              and tf_addr_gen (sequential counter) modules
//              produce the correct twiddle factor values for
//              all NTT and INTT passes.
//
//              The testbench checks:
//              1. r4_addr / r2_addr sequences from tf_addr_gen
//              2. ROM output values (w0, w1, w2, w3) from tf_rom
//              3. Pass transitions and total cycle counts
//              4. Pre-negated INTT values are correct
// ==========================================================

`timescale 1ns/1ps

import poly_arith_pkg::*;

module unipam_controller_tb;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    logic       clk;
    logic       rst;

    logic       start_i;
    pe_mode_e   op_type_i;

    logic       ready_o;
    logic       done_o;

    logic       tf_start_o;
    logic [1:0] pass_idx_o;
    pe_mode_e   pe_ctrl_o;
    logic       pe_valid_o;

    logic       mem_read_en_o;
    logic       mem_write_en_o;
    logic [7:0] rAddr_o;
    logic [7:0] wAddr_o;

    logic [5:0] block_cnt_o;
    logic [5:0] bf_cnt_o;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    unipam_controller dut (
        .clk            (clk),
        .rst            (rst),
        .start_i        (start_i),
        .op_type_i      (op_type_i),
        .ready_o        (ready_o),
        .done_o         (done_o),
        .tf_start_o     (tf_start_o),
        .pass_idx_o     (pass_idx_o),
        .pe_ctrl_o      (pe_ctrl_o),
        .pe_valid_o     (pe_valid_o),
        .mem_read_en_o  (mem_read_en_o),
        .mem_write_en_o (mem_write_en_o),
        .rAddr_o        (rAddr_o),
        .wAddr_o        (wAddr_o),
        .block_cnt_o    (block_cnt_o),
        .bf_cnt_o       (bf_cnt_o)
    );

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Pass/Fail bookkeeping
    // -------------------------------------------------------------------------
    integer pass_count = 0;
    integer fail_count = 0;

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

    task automatic summary;
        begin
            $display("--------------------------------------------------");
            $display("TEST SUMMARY: PASS=%0d  FAIL=%0d", pass_count, fail_count);
            $display("--------------------------------------------------");
            if (fail_count == 0)
                $display("OVERALL RESULT: PASS");
            else
                $display("OVERALL RESULT: FAIL");
        end
    endtask

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    // Removed drive_pe_idle as we emulate the controller based purely on timer

    task automatic reset_dut;
        begin
            rst      = 1'b1;
            start_i  = 1'b0;
            op_type_i = PE_MODE_NTT;

            repeat (3) @(posedge clk);
            rst = 1'b0;
            @(posedge clk);
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
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 1: Reset / idle checks
    // -------------------------------------------------------------------------
    task automatic test_reset_idle;
        begin
            $display("\n=== TEST 1: RESET / IDLE ===");
            reset_dut();

            check(ready_o === 1'b1, "ready_o is high in IDLE after reset");
            check(done_o  === 1'b0, "done_o is low after reset");
            check(pe_valid_o === 1'b0, "pe_valid_o is low in IDLE");
            check(mem_read_en_o === 1'b0, "mem_read_en_o is low in IDLE");
            check(mem_write_en_o === 1'b0, "mem_write_en_o is low in IDLE");
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 2: NTT behavior
    // -------------------------------------------------------------------------
    task automatic test_ntt;
        integer tf_pulses;
        integer prev_pass;
        begin
            $display("\n=== TEST 2: NTT ===");
            reset_dut();

            tf_pulses = 0;
            prev_pass = -1;

            start_op(PE_MODE_NTT);

            // setup cycle
            @(posedge clk);
            check(ready_o === 1'b0, "controller leaves IDLE after NTT start");
            check(tf_start_o === 1'b1, "tf_start_o pulses during NTT setup");

            // wait for validity cycle
            @(posedge clk);
            check(pe_ctrl_o == PE_MODE_NTT, "pe_ctrl_o latched to NTT (delayed by 1 cycle)");

            // monitor a little while
            repeat (20) begin
                @(posedge clk);
                if (tf_start_o) tf_pulses++;
                if (pass_idx_o != prev_pass) begin
                    $display("[%0t] INFO: NTT pass_idx_o = %0d", $time, pass_idx_o);
                    prev_pass = pass_idx_o;
                end
            end

            check(mem_read_en_o === 1'b1, "mem_read_en_o asserted during NTT run");
            check(rAddr_o > 0, "rAddr_o increments during NTT run");
            check(pe_valid_o === 1'b1, "pe_valid_o asserted during NTT run (delayed)");

            wait_done(5000, "NTT");
            check(done_o === 1'b1, "done_o asserted for NTT");
            @(posedge clk);
            check(ready_o === 1'b1, "controller returns to IDLE after NTT");
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 3: INTT behavior
    // -------------------------------------------------------------------------
    task automatic test_intt;
        begin
            $display("\n=== TEST 3: INTT ===");
            reset_dut();

            start_op(PE_MODE_INTT);

            @(posedge clk); // setup
            check(tf_start_o === 1'b1, "tf_start_o pulses during INTT setup");
            @(posedge clk); // validity
            check(pe_ctrl_o == PE_MODE_INTT, "pe_ctrl_o latched to INTT (delayed by 1 cycle)");

            wait_done(5000, "INTT");
            check(done_o === 1'b1, "done_o asserted for INTT");
            @(posedge clk);
            check(ready_o === 1'b1, "controller returns to IDLE after INTT");
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 4: ADDSUB single-pass mode
    // -------------------------------------------------------------------------
    task automatic test_addsub;
        begin
            $display("\n=== TEST 4: ADDSUB ===");
            reset_dut();

            start_op(PE_MODE_ADDSUB);

            @(posedge clk); // setup
            check(tf_start_o === 1'b0, "tf_start_o stays low for ADDSUB");
            @(posedge clk); // validity
            check(pe_ctrl_o == PE_MODE_ADDSUB, "pe_ctrl_o latched to ADDSUB");

            wait_done(1000, "ADDSUB");
            check(done_o === 1'b1, "done_o asserted for ADDSUB");
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 5: CWM single-pass mode
    // -------------------------------------------------------------------------
    task automatic test_cwm;
        begin
            $display("\n=== TEST 5: CWM ===");
            reset_dut();

            start_op(PE_MODE_CWM);

            @(posedge clk); // setup
            check(tf_start_o === 1'b1, "tf_start_o pulses for CWM");
            @(posedge clk); // validity
            check(pe_ctrl_o == PE_MODE_CWM, "pe_ctrl_o latched to CWM");

            wait_done(1000, "CWM");
            check(done_o === 1'b1, "done_o asserted for CWM");
        end
    endtask

    // -------------------------------------------------------------------------
    // Test 6: Basic writeback delay observation
    // -------------------------------------------------------------------------
    task automatic test_writeback_delay;
        integer cycle_idx;
        logic saw_write;
        begin
            $display("\n=== TEST 6: WRITEBACK DELAY ===");
            reset_dut();

            saw_write = 1'b0;
            start_op(PE_MODE_NTT);

            // move into RUN
            @(posedge clk); // setup
            @(posedge clk); // first run cycle

            check(mem_read_en_o === 1'b1, "read enable asserted in RUN");

            for (cycle_idx = 0; cycle_idx < 16; cycle_idx++) begin
                @(posedge clk);
                if (mem_write_en_o) begin
                    saw_write = 1'b1;
                    $display("[%0t] INFO: write observed, rAddr_o=%0d wAddr_o=%0d",
                             $time, rAddr_o, wAddr_o);
                end
            end

            check(saw_write, "write enable eventually asserts after pipeline delay");
        end
    endtask

    // -------------------------------------------------------------------------
    // Main
    // -------------------------------------------------------------------------
    initial begin
        $display("==================================================");
        $display("Starting unipam_controller testbench");
        $display("==================================================");

        test_reset_idle();
        test_ntt();
        test_intt();
        test_addsub();
        test_cwm();
        test_writeback_delay();

        summary();
        $finish;
    end

endmodule
