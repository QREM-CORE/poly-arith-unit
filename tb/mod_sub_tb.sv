// ==========================================================
// Testbench for Modular Subtractor (Combinational)
// Author: Jessica Buentipo, Kiet Le
// Target: FIPS 203 (ML-KEM) - 0-Cycle Latency
// ==========================================================

`timescale 1ns/1ps

module mod_sub_tb();

    import poly_arith_pkg::*;

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic          clk;
    logic          rst;
    coeff_t        op1_i;
    coeff_t        op2_i;
    coeff_t        result_o;

    // ------------------------------------------------------
    // Verification Stats
    // ------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    // Struct to hold expected values
    typedef struct {
        coeff_t a;
        coeff_t b;
        coeff_t expected;
    } transaction_t;

    transaction_t expected_q[$];

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        rst = 1;
        #15 rst = 0;
    end
    always #5 clk = ~clk; // 100MHz

    // ------------------------------------------------------
    // Instantiate DUT (Combinational)
    // ------------------------------------------------------
    mod_sub dut (
        .clk      (clk),
        .rst      (rst),
        .op1_i    (op1_i),
        .op2_i    (op2_i),
        .result_o (result_o)
    );

    // ------------------------------------------------------
    // Task: Drive Inputs & Queue Expectation
    // ------------------------------------------------------
    task send_sub(input [11:0] a, input [11:0] b);
        int signed diff;
        coeff_t expected_val;
        transaction_t trans;
        begin
            @(posedge clk);
            op1_i <= a;
            op2_i <= b;

            // Golden Model Logic for Subtraction
            // (A - B) mod 3329
            diff = $signed({1'b0, a}) - $signed({1'b0, b});

            if (diff < 0) begin
                // Handle negative wrap (underflow)
                expected_val = coeff_t'(diff + Q);
            end else begin
                expected_val = coeff_t'(diff);
            end

            // Prepare Transaction
            trans.a = a;
            trans.b = b;
            trans.expected = expected_val;

            expected_q.push_back(trans);
        end
    endtask

    transaction_t expected_q_delayed[$];

    always @(posedge clk) begin
        if (expected_q.size() > 0) begin
            expected_q_delayed.push_back(expected_q.pop_front());
        end
    end

    // ------------------------------------------------------
    // Monitor / Checker (At Negedge)
    // ------------------------------------------------------
    // Checks result 1 cycle later
    always @(negedge clk) begin
        if (expected_q_delayed.size() > 0) begin
            transaction_t trans;
            trans = expected_q_delayed.pop_front();

            if (result_o === trans.expected) begin
                pass_count++;
            end else begin
                $display("[FAIL] Time:%0t | Input: %d - %d", $time, trans.a, trans.b);
                $display("       Expected:%d | Got:%d", trans.expected, result_o);
                fail_count++;
            end
        end
    end

    // ------------------------------------------------------
    // Main Stimulus
    // ------------------------------------------------------
    initial begin
        // Initialize
        op1_i = 0;
        op2_i = 0;

        $display("==================================================");
        $display("Starting Modular Subtractor Verification (Combinational)");
        $display("==================================================");

        // Wait for startup
        repeat (5) @(posedge clk);

        // --- Test Cases ---
        send_sub(12'd50, 12'd20);      // Simple positive result: 30
        send_sub(12'd20, 12'd50);      // Underflow: 20 - 50 = -30 -> 3299
        send_sub(12'd0, 12'd1);        // Smallest underflow: -1 -> 3328
        send_sub(12'd3328, 12'd0);     // Max positive: 3328
        send_sub(12'd100, 12'd100);    // Zero case
        send_sub(12'd0, 12'd3328);     // Max underflow: -3328 -> 1
        send_sub(12'd1664, 12'd1665);  // Mid-range underflow

        // Random testing
        repeat(50) begin
            coeff_t r1, r2;
            r1 = $urandom_range(0, 3328);
            r2 = $urandom_range(0, 3328);
            send_sub(r1, r2);
        end

        // Wait for checker queue to empty
        wait(expected_q.size() == 0);
        repeat (2) @(posedge clk);

        // --- Final Report ---
        $display("\n--- FINAL REPORT ---");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", fail_count);

        if (fail_count == 0 && pass_count > 0)
            $display("SIMULATION RESULT: SUCCESS\n");
        else begin
            $display("SIMULATION RESULT: FAILURE\n");
            $fatal(1, "mod_sub_tb: Testbench failed.");
        end

        $finish;
    end

endmodule
