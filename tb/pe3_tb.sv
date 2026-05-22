// ==========================================================
// Testbench for PE3 (Butterfly Unit)
// Author(s): Kiet Le
// Target: FIPS 203 (ML-KEM) - Mixed Radix Butterfly
// Mode: Fully Pipelined Verification (Streaming Inputs)
// ==========================================================
`timescale 1ns/1ps

import qrem_global_pkg::*;
import poly_arith_pkg::*;

module pe3_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         a3_i;
    coeff_t         b3_i;
    coeff_t         w3_i;
    coeff_t         tf_omega_4_i; // Unique to PE3
    pe_mode_e       ctrl_i;
    logic           valid_i;

    // Outputs
    coeff_t         u3_o;
    coeff_t         v3_o;
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
    } expected_result_t;

    expected_result_t expected_queue [$]; // FIFO Queue
    string            expected_name_queue [$];

    expected_result_t exp_mon;
    string            exp_name_mon;

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    pe3 dut (
        .clk(clk),
        .rst(rst),
        .a3_i(a3_i),
        .b3_i(b3_i),
        .w3_i(w3_i),
        .tf_omega_4_i(tf_omega_4_i), // Bound Twiddle Factor
        .ctrl_i(ctrl_i),
        .valid_i(valid_i),
        .u3_o(u3_o),
        .v3_o(v3_o),
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
        input coeff_t tf, // Accept Twiddle Factor
        input pe_mode_e mode,
        input string name
    );
        expected_result_t exp;
        coeff_t t;
        coeff_t eff_w; // The Effective W value the hardware *should* select

        // Hardware Emulation: PE3 uses ctrl[1] to select Twiddle Factor
        // NTT(1010) and INTT(1111) will select TF. CWM(1000) will select W3.
        if (mode[1] == 1'b1) eff_w = tf;
        else                 eff_w = w;

        // 1. Calculate Expected Values using the dynamically selected W
        if (mode == PE_MODE_NTT || mode == PE_MODE_CWM) begin
            t = mod_mul(b, eff_w);
            exp.u = mod_add(a, t);
            exp.v = mod_sub(a, t);
        end
        else if (mode == PE_MODE_INTT) begin
            exp.u = mod_div2(mod_add(a, b));
            t     = mod_sub(a, b);
            exp.v = mod_mul(t, eff_w);
        end
        else if (mode == PE_MODE_ADDSUB) begin
            exp.u = mod_add(a, b);
            exp.v = mod_sub(a, b);
        end
        else if (mode == PE_MODE_COMP || mode == PE_MODE_DECOMP) begin
            exp.u = a;
            exp.v = mod_mul(b, eff_w);
        end

        exp.mode = mode;

        // 2. Push expected result to queue
        expected_queue.push_back(exp);
        expected_name_queue.push_back(name);

        // 3. Drive Inputs (1 clock cycle)
        @(posedge clk);
        valid_i      <= 1'b1;
        a3_i         <= a;
        b3_i         <= b;
        w3_i         <= w;
        tf_omega_4_i <= tf; // Drive Twiddle Factor
        ctrl_i       <= mode;
    endtask

    // ------------------------------------------------------
    // Task: Flush Pipeline (Waits for data to drain)
    // ------------------------------------------------------
    task automatic flush_pipeline();
        @(posedge clk);
        valid_i      <= 1'b0;
        a3_i         <= '0;
        b3_i         <= '0;
        w3_i         <= '0;
        tf_omega_4_i <= '0;
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
            if (expected_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_o is high, but expected_queue is empty! (Ghost Pulse)");
                $display("==================================================");
                error_count++;
            end else begin
                // Pop the oldest expected result
                exp_mon = expected_queue.pop_front();
                exp_name_mon = expected_name_queue.pop_front();

                if (u3_o !== exp_mon.u || v3_o !== exp_mon.v) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp_name_mon);
                    $display("Mode: %s", exp_mon.mode.name());
                    if (u3_o !== exp_mon.u) $display("   U-Mismatch! Exp: %0d, Got: %0d", exp_mon.u, u3_o);
                    if (v3_o !== exp_mon.v) $display("   V-Mismatch! Exp: %0d, Got: %0d", exp_mon.v, v3_o);
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
        a3_i = 0; b3_i = 0; w3_i = 0; tf_omega_4_i = 0;
        ctrl_i = PE_MODE_NTT;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE3 Verification (Fully Pipelined)");
        $display("==========================================================");

        // --------------------------------------------------
        // Pipelined Stream: NTT Mode (TF should be selected)
        // --------------------------------------------------
        $display("--- Testing Streaming NTT Mode ---");
        // Syntax: a, b, w, tf, mode, name
        drive_pipeline(0, 0, 999, 0, PE_MODE_NTT, "NTT: All Zeros");
        drive_pipeline(10, 2, 999, 5, PE_MODE_NTT, "NTT: Simple Standard (W ignored)");
        drive_pipeline(100, 0, 999, 50, PE_MODE_NTT, "NTT: B is Zero");
        drive_pipeline(100, 50, 999, 0, PE_MODE_NTT, "NTT: TF is Zero");
        drive_pipeline(0, 1, 999, 3328, PE_MODE_NTT, "NTT: B*TF = -1 Wrap");
        drive_pipeline(3328, 3328, 999, 3328, PE_MODE_NTT, "NTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: INTT Mode (TF should be selected)
        // --------------------------------------------------
        $display("--- Testing Streaming INTT Mode ---");
        drive_pipeline(0, 0, 999, 0, PE_MODE_INTT, "INTT: All Zeros");
        drive_pipeline(20, 10, 999, 2, PE_MODE_INTT, "INTT: Simple Standard");
        drive_pipeline(10, 10, 999, 5, PE_MODE_INTT, "INTT: A=B");
        drive_pipeline(2, 0, 999, 1, PE_MODE_INTT, "INTT: Even Sum");
        drive_pipeline(1, 0, 999, 1, PE_MODE_INTT, "INTT: Odd Sum (Modular Inverse)");
        drive_pipeline(0, 1, 999, 1, PE_MODE_INTT, "INTT: A<B (Neg Wrap)");
        drive_pipeline(3328, 3328, 999, 3328, PE_MODE_INTT, "INTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: CWM Mode (W should be selected, TF ignored)
        // --------------------------------------------------
        $display("--- Testing Streaming CWM Mode ---");
        drive_pipeline(0, 0, 0, 999, PE_MODE_CWM, "CWM: All Zeros");
        drive_pipeline(100, 50, 4, 999, PE_MODE_CWM, "CWM: Simple Standard (TF ignored)");
        drive_pipeline(3328, 3328, 3328, 999, PE_MODE_CWM, "CWM: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: ADD/SUB Mode
        // --------------------------------------------------
        $display("--- Testing Streaming ADD/SUB Mode ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: All Zeros");
        drive_pipeline(200, 50, 999, 999, PE_MODE_ADDSUB, "ADD/SUB: Simple Standard");
        drive_pipeline(1000, 2500, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Add Overflow");
        drive_pipeline(1000, 2000, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Sub Underflow");
        drive_pipeline(100, 100, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: A=B");
        drive_pipeline(3328, 3328, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: Compression & Decompression Mode
        // --------------------------------------------------
        $display("--- Testing Streaming Compression / Decompression Mode ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_COMP, "COMP: All Zeros");
        drive_pipeline(1234, 500, 10, 999, PE_MODE_COMP, "COMP: Simple Standard");
        drive_pipeline(3328, 0, 0, 0, PE_MODE_COMP, "COMP: A-Passthrough Max Check");
        drive_pipeline(1234, 1, 1, 999, PE_MODE_DECOMP, "DECOMP: Multiplier Identity");
        drive_pipeline(3328, 3328, 3328, 999, PE_MODE_DECOMP, "DECOMP: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Random Stress Loop (With Dynamic Flushing)
        // --------------------------------------------------
        $display("--- Starting Random Stress Loop (500 vectors) ---");
        current_mode = PE_MODE_NTT; // Initial assumption

        for (int i = 0; i < 100; i++) begin
            coeff_t ra, rb, rw, rtf;
            pe_mode_e rmode;
            int mode_sel;

            ra  = $urandom_range(0, 3328);
            rb  = $urandom_range(0, 3328);
            rw  = $urandom_range(0, 3328);
            rtf = $urandom_range(0, 3328);

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

            drive_pipeline(ra, rb, rw, rtf, rmode, "Random Streaming Vector");
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
            $fatal(1, "pe3_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end
endmodule
