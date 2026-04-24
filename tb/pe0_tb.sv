// ==========================================================
// Testbench for PE0 (Butterfly Unit)
// Author(s): Kiet Le
// Target: FIPS 203 (ML-KEM) - Mixed Radix Butterfly
// Mode: Fully Pipelined Verification (Streaming Inputs)
// ==========================================================
`timescale 1ns/1ps

import qrem_global_pkg::*;
import poly_arith_pkg::*;

module pe0_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         a0_i;
    coeff_t         b0_i;
    coeff_t         w0_i;
    pe_mode_e       ctrl_i;
    logic           valid_i;

    // Outputs
    coeff_t         u0_o;
    coeff_t         v0_o;
    logic           valid_o;

    // Verification Stats
    int error_count = 0;
    int pass_count  = 0;

    // ------------------------------------------------------
    // Constants & Structures
    // ------------------------------------------------------
    localparam int  MODULUS = 3329;

    // Struct to hold expected results in the FIFO queue
    typedef struct {
        coeff_t     u;
        coeff_t     v;
        pe_mode_e   mode;
        string      name;
    } expected_result_t;

    expected_result_t expected_queue [$]; // FIFO Queue

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    pe0 dut (
        .clk(clk),
        .rst(rst),
        .a0_i(a0_i),
        .b0_i(b0_i),
        .w0_i(w0_i),
        .ctrl_i(ctrl_i),
        .valid_i(valid_i),
        .u0_o(u0_o),
        .v0_o(v0_o),
        .valid_o(valid_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ------------------------------------------------------
    // Golden Model Helpers
    // ------------------------------------------------------
    function automatic coeff_t mod_add(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) + longint'(b)) % MODULUS);
    endfunction

    function automatic coeff_t mod_sub(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) - longint'(b) + MODULUS) % MODULUS);
    endfunction

    function automatic coeff_t mod_mul(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) * longint'(b)) % MODULUS);
    endfunction

    function automatic coeff_t mod_div2(input coeff_t a);
        longint val = longint'(a);
        if (val % 2 != 0) val = val + MODULUS;
        return coeff_t'(val / 2);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Pipeline (Does NOT block/wait for output)
    // ------------------------------------------------------
    task automatic drive_pipeline(
        input coeff_t a,
        input coeff_t b,
        input coeff_t w,
        input pe_mode_e mode,
        input string name
    );
        expected_result_t exp;
        coeff_t t;

        // 1. Calculate Expected Values
        if (mode == PE_MODE_NTT) begin
            t = mod_mul(b, w);
            exp.u = mod_add(a, t);
            exp.v = mod_sub(a, t); // NTT: Top - Bottom
        end
        else if (mode == PE_MODE_CWM) begin
            t = mod_mul(b, w);
            exp.u = mod_add(a, t);
            exp.v = mod_sub(t, a); // CWM: Bottom - Top (The PE0 Fix!)
        end
        else if (mode == PE_MODE_INTT) begin
            exp.u = mod_div2(mod_add(a, b));
            t     = mod_sub(a, b);
            exp.v = mod_mul(t, w);
        end
        else if (mode == PE_MODE_ADDSUB) begin
            exp.u = mod_add(a, b);
            exp.v = mod_sub(a, b);
        end
        else if (mode == PE_MODE_COMP || mode == PE_MODE_DECOMP) begin
            exp.u = a;
            exp.v = mod_mul(b, w);
        end

        exp.mode = mode;
        exp.name = name;

        // 2. Push expected result to queue
        expected_queue.push_back(exp);

        // 3. Drive Inputs (1 clock cycle)
        @(posedge clk);
        valid_i <= 1'b1;
        a0_i    <= a;
        b0_i    <= b;
        w0_i    <= w;
        ctrl_i  <= mode;
    endtask

    // ------------------------------------------------------
    // Task: Flush Pipeline (Waits for data to drain)
    // ------------------------------------------------------
    // Because ctrl_i is combinational, we MUST flush before changing modes.
    task automatic flush_pipeline();
        @(posedge clk);
        valid_i <= 1'b0;
        a0_i    <= '0;
        b0_i    <= '0;
        w0_i    <= '0;
        // NOTE: ctrl_i remains held at its previous value while draining!

        // Wait until all pushed data has been processed by the monitor
        wait(expected_queue.size() == 0);
        repeat(2) @(posedge clk); // Small safety buffer before next operation
    endtask

    // ------------------------------------------------------
    // Monitor Process (Runs continuously in background)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            expected_result_t exp;

            if (expected_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_o is high, but expected_queue is empty! (Ghost Pulse)");
                $display("==================================================");
                error_count++;
            end else begin
                // Pop the oldest expected result
                exp = expected_queue.pop_front();

                if (u0_o !== exp.u || v0_o !== exp.v) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp.name);
                    $display("Mode: %s", exp.mode.name());
                    if (u0_o !== exp.u) $display("   U-Mismatch! Exp: %0d, Got: %0d", exp.u, u0_o);
                    if (v0_o !== exp.v) $display("   V-Mismatch! Exp: %0d, Got: %0d", exp.v, v0_o);
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
        pe_mode_e current_mode;

        // Initialize
        rst = 1;
        valid_i = 0;
        a0_i = 0; b0_i = 0; w0_i = 0;
        ctrl_i = PE_MODE_NTT;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE0 Verification (Fully Pipelined)");
        $display("==========================================================");

        // --------------------------------------------------
        // Pipelined Stream: NTT Mode
        // --------------------------------------------------
        $display("--- Testing Streaming NTT Mode ---");
        drive_pipeline(0, 0, 0, PE_MODE_NTT, "NTT: All Zeros");
        drive_pipeline(1, 1, 1, PE_MODE_NTT, "NTT: All Ones");
        drive_pipeline(10, 2, 5, PE_MODE_NTT, "NTT: Simple Standard");
        drive_pipeline(100, 0, 50, PE_MODE_NTT, "NTT: B is Zero");
        drive_pipeline(100, 50, 0, PE_MODE_NTT, "NTT: W is Zero");
        drive_pipeline(0, 1, 3328, PE_MODE_NTT, "NTT: BW = -1 Wrap");
        drive_pipeline(3328, 3328, 3328, PE_MODE_NTT, "NTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: INTT Mode
        // --------------------------------------------------
        $display("--- Testing Streaming INTT Mode ---");
        drive_pipeline(0, 0, 0, PE_MODE_INTT, "INTT: All Zeros");
        drive_pipeline(20, 10, 2, PE_MODE_INTT, "INTT: Simple Standard");
        drive_pipeline(10, 10, 5, PE_MODE_INTT, "INTT: A=B");
        drive_pipeline(2, 0, 1, PE_MODE_INTT, "INTT: Even Sum");
        drive_pipeline(1, 0, 1, PE_MODE_INTT, "INTT: Odd Sum (Modular Inverse)");
        drive_pipeline(0, 1, 1, PE_MODE_INTT, "INTT: A<B (Neg Wrap)");
        drive_pipeline(3328, 3328, 3328, PE_MODE_INTT, "INTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: CWM Mode
        // --------------------------------------------------
        $display("--- Testing Streaming CWM Mode ---");
        drive_pipeline(0, 0, 0, PE_MODE_CWM, "CWM: All Zeros");
        drive_pipeline(100, 50, 4, PE_MODE_CWM, "CWM: Simple Standard");
        drive_pipeline(3328, 3328, 3328, PE_MODE_CWM, "CWM: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: ADD/SUB Mode
        // --------------------------------------------------
        $display("--- Testing Streaming ADD/SUB Mode ---");
        drive_pipeline(0, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: All Zeros");
        drive_pipeline(200, 50, 0, PE_MODE_ADDSUB, "ADD/SUB: Simple Standard");
        drive_pipeline(1000, 2500, 0, PE_MODE_ADDSUB, "ADD/SUB: Add Overflow");
        drive_pipeline(1000, 2000, 0, PE_MODE_ADDSUB, "ADD/SUB: Sub Underflow");
        drive_pipeline(100, 100, 0, PE_MODE_ADDSUB, "ADD/SUB: A=B");
        drive_pipeline(3328, 3328, 0, PE_MODE_ADDSUB, "ADD/SUB: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: Co/Deco Mode
        // --------------------------------------------------
        $display("--- Testing Streaming Compression (Co/Deco) Mode ---");
        drive_pipeline(0, 0, 0, PE_MODE_COMP, "COMP: All Zeros");
        drive_pipeline(1234, 500, 10, PE_MODE_COMP, "COMP: Simple Standard");
        drive_pipeline(3328, 0, 0, PE_MODE_COMP, "COMP: A-Passthrough Max Check");
        drive_pipeline(1234, 1, 1, PE_MODE_DECOMP, "DECOMP: Multiplier Identity");
        drive_pipeline(3328, 3328, 3328, PE_MODE_DECOMP, "DECOMP: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Random Stress Loop (With Dynamic Flushing)
        // --------------------------------------------------
        $display("--- Starting Random Stress Loop (100 vectors) ---");
        current_mode = PE_MODE_NTT; // Initial assumption

        for (int i = 0; i < 100; i++) begin
            coeff_t ra, rb, rw;
            pe_mode_e rmode;
            int mode_sel;

            ra = $urandom_range(0, 3328);
            rb = $urandom_range(0, 3328);
            rw = $urandom_range(0, 3328);

            // Generate a random mode selection (0 through 5)
            mode_sel = $urandom_range(0, 5);
            case(mode_sel)
                0: rmode = PE_MODE_NTT;
                1: rmode = PE_MODE_INTT;
                2: rmode = PE_MODE_CWM;
                3: rmode = PE_MODE_ADDSUB;
                4: rmode = PE_MODE_COMP;
                5: rmode = PE_MODE_DECOMP;
            endcase

            // Controller Emulation: Flush pipeline if mode switches!
            if (rmode != current_mode) begin
                flush_pipeline();
                current_mode = rmode;
            end

            drive_pipeline(ra, rb, rw, rmode, "Random Streaming Vector");
        end
        flush_pipeline(); // Final flush to clear out the last random vectors

        // Final Summary
        repeat(5) @(posedge clk);
        $display("");
        $display("==========================================================");
        $display("SUMMARY");
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", error_count);
        $display("----------------------------------------------------------");

        if (error_count == 0) $display("RESULT: SUCCESS");
        else begin
            $display("RESULT: FAILURE");
            $fatal(1, "pe0_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end
endmodule
