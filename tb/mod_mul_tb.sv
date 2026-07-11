// ==========================================================
// Testbench for Table-Based Modular Multiplier
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - 12-bit Modular Arithmetic
// Verified Latency: 3 Clock Cycles
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module mod_mul_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         op1;
    coeff_t         op2;
    logic           valid_i;

    // Outputs
    coeff_t         result_o;
    logic           valid_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Pipelined Checking)
    // ------------------------------------------------------
    // Stores expected results to compare against valid_o
    // The queue depth naturally handles the 3-cycle latency
    coeff_t expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    mod_mul dut (
        .clk(clk),
        .rst(rst),
        .op1_i(op1),
        .op2_i(op2),
        .valid_i(valid_i),
        .result_o(result_o),
        .valid_o(valid_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock (10ns period)
    end

    // ------------------------------------------------------
    // Golden Model (Pure Math)
    // ------------------------------------------------------
    // Calculates (A * B) % 3329 using safe 64-bit arithmetic
    function automatic coeff_t get_expected(input coeff_t a, input coeff_t b);
        longint product;
        product = longint'(a) * longint'(b); // Force 64-bit calc
        return coeff_t'(product % 3329);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Inputs
    // ------------------------------------------------------
    task automatic drive_input(input coeff_t a, input coeff_t b, input string name);
        @(posedge clk);
        valid_i <= 1'b1;
        op1     <= a;
        op2     <= b;

        // Push expected result to queue
        expected_queue.push_back(get_expected(a, b));
        sent_count++;
    endtask

    // ------------------------------------------------------
    // Monitor Process
    // ------------------------------------------------------
    always @(posedge clk) begin
        // Checks valid_o; works for any pipeline depth (now 3)
        if (valid_o) begin
            coeff_t expected_val;

            if (expected_queue.size() == 0) begin
                $error("[FAIL] Unexpected output valid_o! Queue is empty.");
                error_count++;
            end else begin
                expected_val = expected_queue.pop_front();

                if (result_o !== expected_val) begin
                    $error("==================================================");
                    $error("[FAIL] Mismatch!");
                    $error("Expected: %0d", expected_val);
                    $error("Received: %0d", result_o);
                    $error("==================================================");
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
        rst = 1;
        valid_i = 0;
        op1 = 0;
        op2 = 0;

        // Reset Sequence
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Multiplier Verification (3-Cycle Latency)");
        $display("==========================================================");

        // --------------------------------------------------
        // 1. Corner Cases
        // --------------------------------------------------
        // Zero
        drive_input(0, 0,    "Zero * Zero");
        drive_input(0, 1234, "Zero * Rand");

        // Identity
        drive_input(1, 1,    "One * One");
        drive_input(1, 3328, "One * Max");

        // Max Values (The Overflow Stress Test)
        // 3328 = -1 mod 3329. So (-1)*(-1) should be 1.
        drive_input(3328, 3328, "Max * Max");

        // --------------------------------------------------
        // 2. Random Stress Testing
        // --------------------------------------------------
        $display("Starting Random Stress Loop (500 vectors)...");

        for (int i = 0; i < 500; i++) begin
            coeff_t rand_a, rand_b;
            rand_a = $urandom_range(0, 3328);
            rand_b = $urandom_range(0, 3328);

            drive_input(rand_a, rand_b, "Random");
        end

        // Stop Driving
        @(posedge clk);
        valid_i <= 0;
        op1 <= 0; op2 <= 0;

        // Wait for Pipeline to Drain (3 cycles + safety)
        repeat(10) @(posedge clk);
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
            $fatal(1, "mod_mul_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
