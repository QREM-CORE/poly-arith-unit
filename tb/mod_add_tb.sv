// ==========================================================
// Testbench for Modular Adder (Combinational)
// Author: Jessica Buentipo, Kiet Le
// Target: FIPS 203 (ML-KEM) - 0-Cycle Latency
// ==========================================================

`timescale 1ns/1ps

module mod_add_tb();

    import poly_arith_pkg::*;

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic          clk;
    logic          rst;
    coeff_t        op1_i;
    coeff_t        op2_i;
    coeff_t        result_o;

    // ------------------------------------------------------/
    // Verification Stats
    // ------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    // Struct to hold expected values for checking
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
    mod_add dut (
        .clk      (clk),
        .rst      (rst),
        .op1_i    (op1_i),
        .op2_i    (op2_i),
        .result_o (result_o)
    );

    // ------------------------------------------------------
    // Task: Drive Inputs & Queue Expectation
    // ------------------------------------------------------
    task send_add(input [11:0] a, input [11:0] b);
        logic [12:0] local_sum;
        transaction_t trans;
        begin
            @(posedge clk);
            op1_i <= a;
            op2_i <= b;

            // Golden Model Logic
            local_sum = a + b;

            // Prepare transaction
            trans.a = a;
            trans.b = b;
            // Modulo reduction: If sum >= 3329, subtract 3329
            trans.expected = (local_sum >= Q) ? (local_sum - Q) : local_sum[11:0];

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
    // Checks the result 1 cycle later
    always @(negedge clk) begin
        if (expected_q_delayed.size() > 0) begin
            transaction_t trans;
            trans = expected_q_delayed.pop_front();

            if (result_o === trans.expected) begin
                pass_count++;
            end else begin
                $display("[FAIL] Time:%0t | Input: %d + %d", $time, trans.a, trans.b);
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
        $display("Starting Modular Adder Verification (Combinational)");
        $display("==================================================");

        // Wait for startup
        repeat (5) @(posedge clk);

        // --- Test Cases ---
        send_add(12'd10, 12'd20);      // No reduction
        send_add(Q-1, 12'd1);          // Exact wrap (3328 + 1 = 3329 -> 0)
        send_add(12'd3000, 12'd3000);  // Large reduction (6000 - 3329 = 2671)
        send_add(12'd0, 12'd0);        // Zeros
        send_add(12'd3328, 12'd3328);  // Max possible (6656 - 3329 = 3327)

        // Random Stress Test
        repeat(50) begin
            coeff_t r1, r2;
            r1 = $urandom_range(0, 3328);
            r2 = $urandom_range(0, 3328);
            send_add(r1, r2);
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
            $fatal(1, "mod_add_tb: Testbench failed.");
        end

        $finish;
    end

endmodule
