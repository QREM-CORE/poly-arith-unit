// ==========================================================
// Testbench for Modular Unified Adder/Subtractor
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM)
// Verified Latency: 0 Clock Cycles (Combinational)
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module mod_uni_add_sub_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;

    // Inputs
    coeff_t         op1;
    coeff_t         op2;
    logic           is_sub; // 0 = Add, 1 = Sub
    logic           rst;

    // Outputs
    coeff_t         result_o;

    logic           data_valid_in = 0;
    logic           data_valid_out = 0;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Checking)
    // ------------------------------------------------------
    typedef struct {
        coeff_t a;
        coeff_t b;
        logic   sub;
        coeff_t expected;
        string  name;
    } transaction_t;

    transaction_t expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    mod_uni_add_sub dut (
        .clk      (clk),
        .rst      (rst),
        .op1_i    (op1),
        .op2_i    (op2),
        .is_sub_i (is_sub),
        .result_o (result_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        rst = 1;
        #15 rst = 0;
    end
    always #5 clk = ~clk; // 100MHz Clock

    // ------------------------------------------------------
    // Golden Model (Pure Math)
    // ------------------------------------------------------
    function automatic coeff_t get_expected(input coeff_t a, input coeff_t b, input logic sub);
        longint res;
        if (sub) begin
            // (a - b) mod Q (Ensure positive)
            res = (longint'(a) + 3329 - longint'(b)) % 3329;
        end else begin
            // (a + b) mod Q
            res = (longint'(a) + longint'(b)) % 3329;
        end
        return coeff_t'(res);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Inputs
    // ------------------------------------------------------
    task automatic drive_input(input coeff_t a, input coeff_t b, input logic sub, input string name);
        transaction_t trans;

        // Drive inputs (at Posedge)
        data_valid_in <= 1'b1;
        op1    <= a;
        op2    <= b;
        is_sub <= sub;

        // Prepare Transaction
        trans.a = a;
        trans.b = b;
        trans.sub = sub;
        trans.expected = get_expected(a, b, sub);
        trans.name = name;

        // Push to queue
        expected_queue.push_back(trans);
        sent_count++;

        // Wait for next cycle
        @(posedge clk);
    endtask

    always @(posedge clk) begin
        data_valid_out <= data_valid_in;
    end

    // ------------------------------------------------------
    // Monitor Process
    // ------------------------------------------------------
    always @(negedge clk) begin
        if (data_valid_out) begin
            transaction_t trans;
            if (expected_queue.size() > 0) begin
                trans = expected_queue.pop_front();

                // ONLY print if there is a mismatch
                if (result_o !== trans.expected) begin
                    $display("\n==================================================");
                    $display("[FAIL] Mismatch on %s", trans.name);
                    $display("  Op:       %s", (trans.sub ? "Subtraction" : "Addition"));
                    $display("  Input A:  %0d", trans.a);
                    $display("  Input B:  %0d", trans.b);
                    $display("  Expected: %0d", trans.expected);
                    $display("  Received: %0d", result_o);
                    $display("==================================================\n");
                    $error("Test Failed");
                    error_count++;
                end else begin
                    recv_count++;
                end
            end
        end
    end

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize
        op1 = 0; op2 = 0; is_sub = 0;
        data_valid_in = 0;

        // Wait for startup
        repeat(5) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Adder/Subtractor Verification (Combinational)");
        $display("==========================================================");

        // --------------------------------------------------
        // 1. ADDITION Corner Cases
        // --------------------------------------------------
        drive_input(0, 0,       0, "Add Zero");
        drive_input(100, 200,   0, "Add Standard");
        drive_input(3328, 1,    0, "Add Boundary");
        drive_input(3328, 3328, 0, "Add Max");

        // --------------------------------------------------
        // 2. SUBTRACTION Corner Cases
        // --------------------------------------------------
        drive_input(500, 200,   1, "Sub Standard");
        drive_input(500, 500,   1, "Sub Zero");
        drive_input(0, 1,       1, "Sub Underflow");
        drive_input(10, 20,     1, "Sub Wrap");

        // --------------------------------------------------
        // 3. Random Stress Testing
        // --------------------------------------------------
        $display("Starting Random Stress Loop (500 vectors)...");

        for (int i = 0; i < 500; i++) begin
            coeff_t rand_a, rand_b;
            logic   rand_sub;

            rand_a   = $urandom_range(0, 3328);
            rand_b   = $urandom_range(0, 3328);
            rand_sub = $urandom_range(0, 1);

            drive_input(rand_a, rand_b, rand_sub, "Random");
        end

        // Stop Driving
        // wait for one cycle to ensure last non-blocking assignment propagated
        // wait, drive_input already waits for a posedge clk at the end.
        data_valid_in <= 1'b0;

        // Wait for Queue to Drain (Pipeline Drain)
        repeat (5) @(posedge clk);
        if (expected_queue.size() != 0) begin
            $error("[FAIL] Pipeline failed to drain! %0d results missing.", expected_queue.size());
            error_count++;
        end

        // --------------------------------------------------
        // Final Report
        // --------------------------------------------------
        $display("==========================================================");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED");
            $display("Vectors Processed: %0d", recv_count);
        end else begin
            $display("TEST FAILED: %0d Errors Found", error_count);
            $fatal(1, "mod_uni_add_sub_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
