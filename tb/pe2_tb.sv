// ==========================================================
// Testbench for PE2 (Processing Element 2)
// Author(s): Kiet Le
// Target: FIPS 203 (ML-KEM) - Mixed Radix Butterfly
// Mode: Fully Pipelined Verification (Datapath-Centric)
// ==========================================================
`timescale 1ns/1ps

import qrem_global_pkg::*;
import poly_arith_pkg::*;

module pe2_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         a2_i;
    coeff_t         b2_i;
    coeff_t         w1_i;
    coeff_t         w2_i;
    pe_mode_e       ctrl_i;
    logic           valid_i;

    // Outputs
    coeff_t         u2_o;
    coeff_t         v2_o;
    coeff_t         m_o;
    logic           valid_o;
    logic           valid_m_o;

    // Verification Stats
    int error_count = 0;
    int pass_count  = 0;

    // ------------------------------------------------------
    // Constants & Structures
    // ------------------------------------------------------
    localparam int  MODULUS = 3329;

    typedef struct {
        coeff_t     u;
        coeff_t     v;
        pe_mode_e   mode;
        string      name;
    } expected_uv_t;

    typedef struct {
        coeff_t     m;
        string      name;
    } expected_m_t;

    expected_uv_t expected_uv_queue [$];
    expected_m_t  expected_m_queue  [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    pe2 dut (
        .clk(clk),
        .rst(rst),
        .a2_i(a2_i),
        .b2_i(b2_i),
        .w1_i(w1_i),
        .w2_i(w2_i),
        .ctrl_i(ctrl_i),
        .valid_i(valid_i),
        .u2_o(u2_o),
        .v2_o(v2_o),
        .m_o(m_o),
        .valid_o(valid_o),
        .valid_m_o(valid_m_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
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

    // ------------------------------------------------------
    // Task: Drive Pipeline
    // ------------------------------------------------------
    task automatic drive_pipeline(
        input coeff_t a,
        input coeff_t b,
        input coeff_t w1,
        input coeff_t w2,
        input pe_mode_e mode,
        input string name
    );
        expected_uv_t exp_uv;
        expected_m_t  exp_m;
        coeff_t t1, t2;

        // 1. Calculate Expected Values based on actual PE2 datapath routing
        if (mode == PE_MODE_NTT) begin
            // MUXes select: U2 = (A2*W1) + (B2*W2), V2 = (A2*W1) - (B2*W2)
            t1 = mod_mul(a, w1);
            t2 = mod_mul(b, w2);
            exp_uv.u = mod_add(t1, t2);
            exp_uv.v = mod_sub(t1, t2);
        end
        else if (mode == PE_MODE_INTT) begin
            // MUXes select: U2 = (A2+B2)*W1, V2 = (A2-B2)*W2
            t1 = mod_add(a, b);
            t2 = mod_sub(a, b);
            exp_uv.u = mod_mul(t1, w1);
            exp_uv.v = mod_mul(t2, w2);
        end
        else if (mode == PE_MODE_CWM) begin
            // Karatsuba Path: U2 = A*W1, V2 = B*W2, M = U2 + V2
            t1 = mod_mul(a, w1);
            t2 = mod_mul(b, w2);
            exp_uv.u = t1;
            exp_uv.v = t2;

            exp_m.m    = mod_add(t1, t2);
            exp_m.name = name;
            expected_m_queue.push_back(exp_m); // Push M to its separate queue
        end
        else if (mode == PE_MODE_ADDSUB) begin
            exp_uv.u = mod_add(a, b);
            exp_uv.v = mod_sub(a, b);
        end
        else if (mode == PE_MODE_COMP || mode == PE_MODE_DECOMP) begin
            exp_uv.u = mod_mul(a, w1);
            exp_uv.v = mod_mul(b, w2);
        end

        exp_uv.mode = mode;
        exp_uv.name = name;
        expected_uv_queue.push_back(exp_uv);

        // 2. Drive Inputs
        @(posedge clk);
        valid_i <= 1'b1;
        a2_i    <= a;
        b2_i    <= b;
        w1_i    <= w1;
        w2_i    <= w2;
        ctrl_i  <= mode;
    endtask

    // ------------------------------------------------------
    // Task: Flush Pipeline
    // ------------------------------------------------------
    task automatic flush_pipeline();
        @(posedge clk);
        valid_i <= 1'b0;
        a2_i    <= '0;
        b2_i    <= '0;
        w1_i    <= '0;
        w2_i    <= '0;

        wait(expected_uv_queue.size() == 0 && expected_m_queue.size() == 0);
        repeat(2) @(posedge clk);
    endtask

    // ------------------------------------------------------
    // Monitor Process 1: Primary Data (U2, V2)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            expected_uv_t exp;

            if (expected_uv_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_o is high, but expected_uv_queue is empty!");
                error_count++;
            end else begin
                exp = expected_uv_queue.pop_front();

                if (u2_o !== exp.u || v2_o !== exp.v) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp.name);
                    $display("Mode: %s", exp.mode.name());
                    if (u2_o !== exp.u) $display("   U2-Mismatch! Exp: %0d, Got: %0d", exp.u, u2_o);
                    if (v2_o !== exp.v) $display("   V2-Mismatch! Exp: %0d, Got: %0d", exp.v, v2_o);
                    $display("==================================================");
                    error_count++;
                end else begin
                    pass_count++;
                end
            end
        end
    end

    // ------------------------------------------------------
    // Monitor Process 2: Delayed Karatsuba Term (M)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_m_o) begin
            expected_m_t exp;

            if (expected_m_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_m_o is high, but expected_m_queue is empty!");
                error_count++;
            end else begin
                exp = expected_m_queue.pop_front();

                if (m_o !== exp.m) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp.name);
                    $display("Mode: CWM (M_O Cross Term)");
                    $display("   M-Mismatch! Exp: %0d, Got: %0d", exp.m, m_o);
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

        rst = 1;
        valid_i = 0;
        a2_i = 0; b2_i = 0; w1_i = 0; w2_i = 0;
        ctrl_i = PE_MODE_NTT;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE2 Verification (Datapath-Centric)");
        $display("==========================================================");

        // --------------------------------------------------
        // Pipelined Stream: NTT Mode (Structural Verification)
        // --------------------------------------------------
        $display("--- Testing Streaming NTT Mode ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_NTT, "NTT: All Zeros");
        drive_pipeline(10, 2, 999, 5, PE_MODE_NTT, "NTT: Simple Standard");
        drive_pipeline(100, 0, 0, 50, PE_MODE_NTT, "NTT: B is Zero");
        drive_pipeline(3328, 3328, 0, 3328, PE_MODE_NTT, "NTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: INTT Mode (Structural Verification)
        // --------------------------------------------------
        $display("--- Testing Streaming INTT Mode ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_INTT, "INTT: All Zeros");
        drive_pipeline(20, 10, 999, 2, PE_MODE_INTT, "INTT: Simple Standard");
        drive_pipeline(10, 10, 999, 5, PE_MODE_INTT, "INTT: A=B");
        drive_pipeline(1, 0, 999, 1, PE_MODE_INTT, "INTT: Odd Sum");
        drive_pipeline(3328, 3328, 999, 3328, PE_MODE_INTT, "INTT: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: CWM Mode (Triggers Dual-Monitors)
        // --------------------------------------------------
        $display("--- Testing Streaming CWM Mode (Karatsuba Path) ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_CWM, "CWM: All Zeros");
        drive_pipeline(100, 50, 4, 2, PE_MODE_CWM, "CWM: Simple Standard");
        drive_pipeline(3328, 3328, 3328, 3328, PE_MODE_CWM, "CWM: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: ADD/SUB Mode
        // --------------------------------------------------
        $display("--- Testing Streaming ADD/SUB Mode ---");
        drive_pipeline(200, 50, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Simple Standard");
        drive_pipeline(1000, 2500, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Add Overflow");
        drive_pipeline(1000, 2000, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: Sub Underflow");
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: Co/Deco Mode
        // --------------------------------------------------
        $display("--- Testing Streaming Compression (Co/Deco) Mode ---");
        drive_pipeline(0, 0, 0, 0, PE_MODE_COMP, "CODECO: All Zeros");
        drive_pipeline(1234, 500, 10, 5, PE_MODE_COMP, "CODECO: Simple Standard");
        drive_pipeline(3328, 3328, 3328, 3328, PE_MODE_COMP, "CODECO: Max Values");
        flush_pipeline();

        // --------------------------------------------------
        // Random Stress Loop (All Modes)
        // --------------------------------------------------
        $display("--- Starting Random Stress Loop (100 vectors) ---");
        current_mode = PE_MODE_NTT;

        for (int i = 0; i < 100; i++) begin
            coeff_t ra, rb, rw1, rw2;
            pe_mode_e rmode;
            int mode_sel;

            ra  = $urandom_range(0, 3328);
            rb  = $urandom_range(0, 3328);
            rw1 = $urandom_range(0, 3328);
            rw2 = $urandom_range(0, 3328);

            mode_sel = $urandom_range(0, 5);
            case(mode_sel)
                0: rmode = PE_MODE_NTT;
                1: rmode = PE_MODE_INTT;
                2: rmode = PE_MODE_CWM;
                3: rmode = PE_MODE_ADDSUB;
                4: rmode = PE_MODE_COMP;
                5: rmode = PE_MODE_DECOMP;
            endcase

            if (rmode != current_mode) begin
                flush_pipeline();
                current_mode = rmode;
            end

            drive_pipeline(ra, rb, rw1, rw2, rmode, "Random Streaming Vector");
        end
        flush_pipeline();

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
            $fatal(1, "pe2_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
