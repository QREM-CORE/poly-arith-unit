// ==========================================================
// Testbench for MAC Adder (Standalone Unit Test)
// Author(s): Salwan Aldhahab
// Target: FIPS 203 (ML-KEM) - Multiply-Accumulate Adder
// Mode: Pipelined Verification (1 CC Latency)
//
// Description:
// Comprehensive standalone verification of the mac_adder module.
// Tests both lanes independently and together across:
//   - First-term mode (passthrough, j=0)
//   - Accumulate mode (modular addition, j>0)
//   - Boundary values (0, Q-1, wraparound)
//   - Multi-step accumulation sequences (simulating k iterations)
//   - Random stress testing
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module mac_adder_tb();

    // ------------------------------------------------------
    // Constants
    // ------------------------------------------------------
    localparam int MODULUS = 3329;

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic       clk;
    logic       rst;

    // DUT Inputs
    logic       first_term_i;
    logic       valid_i;
    coeff_t     a0_i, b0_i;
    coeff_t     a1_i, b1_i;

    // DUT Outputs
    coeff_t     z0_o, z1_o;
    logic       valid_o;

    // Verification Stats
    int pass_count  = 0;
    int error_count = 0;

    // ------------------------------------------------------
    // Expected Result Structure
    // ------------------------------------------------------
    typedef struct {
        coeff_t     z0;
        coeff_t     z1;
        string      name;
    } expected_result_t;

    expected_result_t expected_queue [$];

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    mac_adder dut (
        .clk        (clk),
        .rst        (rst),
        .first_term_i(first_term_i),
        .valid_i    (valid_i),
        .a0_i       (a0_i),
        .b0_i       (b0_i),
        .a1_i       (a1_i),
        .b1_i       (b1_i),
        .z0_o       (z0_o),
        .z1_o       (z1_o),
        .valid_o    (valid_o)
    );

    // ------------------------------------------------------
    // Golden Model Helper
    // ------------------------------------------------------
    function automatic coeff_t gm_mod_add(input coeff_t a, input coeff_t b);
        logic [12:0] sum;
        sum = a + b;
        return (sum >= MODULUS) ? coeff_t'(sum - MODULUS) : coeff_t'(sum);
    endfunction

    // ------------------------------------------------------
    // Task: Drive a single MAC operation (1 clock cycle)
    // Does NOT block waiting for output.
    // ------------------------------------------------------
    task automatic drive_mac(
        input coeff_t   a0,     // New CWM result lane 0
        input coeff_t   b0,     // Partial sum lane 0
        input coeff_t   a1,     // New CWM result lane 1
        input coeff_t   b1,     // Partial sum lane 1
        input logic     first_term,   // 1 = passthrough, 0 = accumulate
        input string    name
    );
        expected_result_t exp;

        // Golden model: first_term selects passthrough vs accumulate.
        if (first_term) begin
            exp.z0 = a0;
            exp.z1 = a1;
        end else begin
            exp.z0 = gm_mod_add(a0, b0);
            exp.z1 = gm_mod_add(a1, b1);
        end
        exp.name = name;

        expected_queue.push_back(exp);

        // Drive inputs
        @(posedge clk);
        valid_i <= 1'b1;
        first_term_i <= first_term;
        a0_i    <= a0;
        b0_i    <= b0;
        a1_i    <= a1;
        b1_i    <= b1;
    endtask

    // ------------------------------------------------------
    // Task: Deassert valid and wait for pipeline to drain
    // ------------------------------------------------------
    task automatic flush_pipeline();
        @(posedge clk);
        valid_i <= 1'b0;
        first_term_i <= 1'b0;
        a0_i    <= '0;
        b0_i    <= '0;
        a1_i    <= '0;
        b1_i    <= '0;

        wait(expected_queue.size() == 0);
        repeat(2) @(posedge clk);
    endtask

    // ------------------------------------------------------
    // Task: Drive a full multi-step MAC accumulation sequence
    // Simulates: acc = sum_{j=0}^{k-1} new_value[j]
    // Uses an internal "software accumulator" to generate the
    // correct b_i feedback values each cycle.
    // ------------------------------------------------------
    task automatic drive_mac_sequence(
        input int       k,      // Number of accumulation steps
        input string    name
    );
        coeff_t acc0, acc1;
        coeff_t new_a0, new_a1;

        for (int j = 0; j < k; j++) begin
            new_a0 = $urandom_range(0, MODULUS - 1);
            new_a1 = $urandom_range(0, MODULUS - 1);

            if (j == 0) begin
                // First iteration: first-term passthrough, b_i is don't-care
                drive_mac(new_a0, 12'd0, new_a1, 12'd0, 1'b1,
                          $sformatf("%s [j=%0d/%0d INIT]", name, j, k));
                acc0 = new_a0;
                acc1 = new_a1;
            end else begin
                // Subsequent iterations: accumulate with previous sum
                drive_mac(new_a0, acc0, new_a1, acc1, 1'b0,
                          $sformatf("%s [j=%0d/%0d ACC]", name, j, k));
                acc0 = gm_mod_add(new_a0, acc0);
                acc1 = gm_mod_add(new_a1, acc1);
            end
        end
    endtask

    // ------------------------------------------------------
    // Monitor / Checker (Runs continuously)
    // 1 CC latency: check on cycle after drive
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            if (expected_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_o high but expected_queue is empty! (Ghost Pulse)");
                $display("==================================================");
                error_count++;
            end else begin
                expected_result_t exp;
                exp = expected_queue.pop_front();

                if (z0_o !== exp.z0 || z1_o !== exp.z1) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp.name);
                    if (z0_o !== exp.z0) $display("   Lane 0 Mismatch! Exp: %0d, Got: %0d", exp.z0, z0_o);
                    if (z1_o !== exp.z1) $display("   Lane 1 Mismatch! Exp: %0d, Got: %0d", exp.z1, z1_o);
                    $display("==================================================");
                    error_count++;
                end else begin
                    pass_count++;
                end
            end
        end
    end

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize
        rst     = 1;
        valid_i = 0;
        first_term_i  = 0;
        a0_i = 0; b0_i = 0;
        a1_i = 0; b1_i = 0;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting MAC Adder Standalone Verification");
        $display("==========================================================");

        // --------------------------------------------------
        // Test 1: Init Mode (Passthrough)
        // Expected: z = a (bypass, ignore b)
        // --------------------------------------------------
        $display("--- Test 1: Init Mode (Passthrough) ---");
        drive_mac(12'd0,    12'd0,    12'd0,    12'd0,    1'b1, "Init: All Zeros");
        drive_mac(12'd100,  12'd999,  12'd200,  12'd888,  1'b1, "Init: Ignore B Values");
        drive_mac(12'd3328, 12'd3328, 12'd3328, 12'd3328, 1'b1, "Init: Max Values");
        drive_mac(12'd1,    12'd3328, 12'd1,    12'd3328, 1'b1, "Init: Min A, Max B");
        drive_mac(12'd1729, 12'd0,    12'd2580, 12'd0,    1'b1, "Init: Typical Zeta Values");
        flush_pipeline();

        // --------------------------------------------------
        // Test 2: Accumulate Mode (Modular Addition)
        // Expected: z = (a + b) mod 3329
        // --------------------------------------------------
        $display("--- Test 2: Accumulate Mode (Modular Add) ---");
        drive_mac(12'd0,    12'd0,    12'd0,    12'd0,    1'b0, "Acc: All Zeros");
        drive_mac(12'd10,   12'd20,   12'd30,   12'd40,   1'b0, "Acc: Simple No Wrap");
        drive_mac(12'd3328, 12'd1,    12'd3328, 12'd1,    1'b0, "Acc: Exact Wrap (3328+1=0)");
        drive_mac(12'd3000, 12'd3000, 12'd3000, 12'd3000, 1'b0, "Acc: Large Wrap (6000-3329=2671)");
        drive_mac(12'd3328, 12'd3328, 12'd3328, 12'd3328, 1'b0, "Acc: Max Both (6656-3329=3327)");
        drive_mac(12'd0,    12'd3328, 12'd3328, 12'd0,    1'b0, "Acc: One Zero Per Lane");
        drive_mac(12'd1665, 12'd1664, 12'd1664, 12'd1665, 1'b0, "Acc: Just Below Q (3329)");
        drive_mac(12'd1665, 12'd1665, 12'd1665, 12'd1665, 1'b0, "Acc: Just Above Q (3330-3329=1)");
        flush_pipeline();

        // --------------------------------------------------
        // Test 3: Lane Independence
        // Verify each lane operates independently
        // --------------------------------------------------
        $display("--- Test 3: Lane Independence ---");
        drive_mac(12'd100,  12'd200,  12'd0,    12'd0,    1'b0, "Lane Indep: L0 active, L1 zero");
        drive_mac(12'd0,    12'd0,    12'd100,  12'd200,  1'b0, "Lane Indep: L0 zero, L1 active");
        drive_mac(12'd3000, 12'd500,  12'd10,   12'd20,   1'b0, "Lane Indep: L0 wrap, L1 no wrap");
        drive_mac(12'd10,   12'd20,   12'd3000, 12'd500,  1'b0, "Lane Indep: L0 no wrap, L1 wrap");
        flush_pipeline();

        // --------------------------------------------------
        // Test 4: Init/Acc Transition (Back-to-Back)
        // Simulates the j=0 -> j=1 transition within a stream
        // --------------------------------------------------
        $display("--- Test 4: Init/Acc Transition ---");
        drive_mac(12'd500,  12'd0,    12'd600,  12'd0,    1'b1, "Trans: Init j=0");
        drive_mac(12'd700,  12'd500,  12'd800,  12'd600,  1'b0, "Trans: Acc j=1 (a+prev)");
        drive_mac(12'd100,  12'd1200, 12'd200,  12'd1400, 1'b0, "Trans: Acc j=2 (a+prev)");
        flush_pipeline();

        // --------------------------------------------------
        // Test 5: Multi-Step Accumulation (k=2, ML-KEM-512)
        // --------------------------------------------------
        $display("--- Test 5: Multi-Step Sequence k=2 ---");
        drive_mac_sequence(2, "MAC-k2");
        flush_pipeline();

        // --------------------------------------------------
        // Test 6: Multi-Step Accumulation (k=3, ML-KEM-768)
        // --------------------------------------------------
        $display("--- Test 6: Multi-Step Sequence k=3 ---");
        drive_mac_sequence(3, "MAC-k3");
        flush_pipeline();

        // --------------------------------------------------
        // Test 7: Multi-Step Accumulation (k=4, ML-KEM-1024)
        // --------------------------------------------------
        $display("--- Test 7: Multi-Step Sequence k=4 ---");
        drive_mac_sequence(4, "MAC-k4");
        flush_pipeline();

        // --------------------------------------------------
        // Test 8: Valid Signal Gating
        // Ensure output valid follows input valid with 1 CC
        // --------------------------------------------------
        $display("--- Test 8: Valid Signal Gating ---");
        // Drive one valid, then idle, then another
        drive_mac(12'd42, 12'd0, 12'd84, 12'd0, 1'b1, "Valid Gate: Pulse 1");
        flush_pipeline();

        // Insert gap with valid_i = 0
        repeat(3) @(posedge clk);

        drive_mac(12'd99, 12'd0, 12'd77, 12'd0, 1'b1, "Valid Gate: Pulse 2");
        flush_pipeline();

        // --------------------------------------------------
        // Test 9: Reset Behavior
        // Drive data, then assert reset mid-stream
        // --------------------------------------------------
        $display("--- Test 9: Reset Behavior ---");
        // Drive one valid vector. With 1 CC registered latency, the DUT
        // captures this result on the next posedge BEFORE reset takes
        // effect one cycle later. drive_mac pushes the expected value
        // so the monitor can verify the legitimate output.
        drive_mac(12'd111, 12'd0, 12'd222, 12'd0, 1'b1, "Reset: Pre-Reset Capture");

        // Assert reset — takes effect on the NEXT posedge, clearing
        // the registered outputs that were just captured above.
        @(posedge clk);
        valid_i <= 1'b0;
        rst     <= 1'b1;  // Assert reset

        repeat(3) @(posedge clk);

        // Check outputs are cleared
        if (z0_o !== 12'd0 || z1_o !== 12'd0 || valid_o !== 1'b0) begin
            $display("[FAIL] Reset: Outputs not cleared! z0=%0d z1=%0d valid=%b", z0_o, z1_o, valid_o);
            error_count++;
        end else begin
            $display("[PASS] Reset: Outputs correctly cleared.");
            pass_count++;
        end

        rst <= 1'b0;
        // Clear any stale entries that might have been pushed before reset
        expected_queue.delete();
        repeat(2) @(posedge clk);

        // Verify normal operation resumes after reset
        drive_mac(12'd500, 12'd0, 12'd600, 12'd0, 1'b1, "Post-Reset: Normal Op");
        flush_pipeline();

        // --------------------------------------------------
        // Test 10: Random Stress Test
        // --------------------------------------------------
        $display("--- Test 10: Random Stress (200 vectors) ---");
        repeat(200) begin
            coeff_t ra0, rb0, ra1, rb1;
            logic rinit;

            ra0   = $urandom_range(0, MODULUS - 1);
            rb0   = $urandom_range(0, MODULUS - 1);
            ra1   = $urandom_range(0, MODULUS - 1);
            rb1   = $urandom_range(0, MODULUS - 1);
            rinit = $urandom_range(0, 1);

            drive_mac(ra0, rb0, ra1, rb1, rinit, "Random Stress");
        end
        flush_pipeline();

        // --------------------------------------------------
        // Test 11: Random Multi-Step Sequences (x20)
        // --------------------------------------------------
        $display("--- Test 11: Random Multi-Step Sequences (x20) ---");
        repeat(20) begin
            int rk;
            rk = $urandom_range(2, 4); // Random k between 2 and 4
            drive_mac_sequence(rk, $sformatf("RandSeq-k%0d", rk));
            flush_pipeline();
        end

        // --------------------------------------------------
        // Final Report
        // --------------------------------------------------
        repeat(5) @(posedge clk);
        $display("");
        $display("==========================================================");
        $display("MAC ADDER STANDALONE TESTBENCH SUMMARY");
        $display("----------------------------------------------------------");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", error_count);
        $display("----------------------------------------------------------");

        if (error_count == 0 && pass_count > 0)
            $display("RESULT: SUCCESS");
        else begin
            $display("RESULT: FAILURE");
            $fatal(1, "mac_adder_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
