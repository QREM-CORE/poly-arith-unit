// ==========================================================
// Testbench for PE_UNIT (Top-Level Arithmetic Unit Wrapper)
// Author(s): Kiet Le
// Target: FIPS 203 (ML-KEM) - Unified Polynomial Arithmetic
// Mode: Fully Pipelined Verification (Streaming Inputs)
// ==========================================================
`timescale 1ns/1ps

import qrem_global_pkg::*;
import poly_arith_pkg::*;

module pe_unit_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    logic           valid_i = 0;
    pe_mode_e       ctrl_i = PE_MODE_CWM;
    logic           mode_i = 0; // 0 = Add/Radix-4, 1 = Sub/Radix-2

    // Primary Operand Bus (A)
    coeff_t         op_a0_i, op_a1_i, op_a2_i, op_a3_i;
    // Secondary Operand Bus (B)
    coeff_t         op_b0_i, op_b1_i, op_b2_i, op_b3_i;

    // Outputs
    coeff_t         z0_o, z1_o, z2_o, z3_o;
    logic           valid_o;

    // Verification Stats
    int error_count = 0;
    int pass_count  = 0;

    // ------------------------------------------------------
    // Constants & Structures
    // ------------------------------------------------------
    localparam int  MODULUS = 3329;

    // Struct to hold expected results for ALL outputs
    typedef struct {
        coeff_t     z0;
        coeff_t     z1;
        coeff_t     z2;
        coeff_t     z3;
        pe_mode_e   mode;
        logic       mode_sel;
    } expected_result_t;

    expected_result_t expected_queue [$]; // FIFO Queue
    string            name_queue [$];     // Parallel FIFO Queue for names

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    pe_unit dut (
        .clk(clk),
        .rst(rst),
        .valid_i(valid_i),
        .ctrl_i(ctrl_i),
        .is_radix2_i(mode_i),
        .is_sub_i(mode_i),
        .op_a0_i(op_a0_i), .op_a1_i(op_a1_i), .op_a2_i(op_a2_i), .op_a3_i(op_a3_i),
        .op_b0_i(op_b0_i), .op_b1_i(op_b1_i), .op_b2_i(op_b2_i), .op_b3_i(op_b3_i),
        .z0_o(z0_o), .z1_o(z1_o), .z2_o(z2_o), .z3_o(z3_o),
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
    // Golden Model Helpers (Modulo Math)
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
    // Computes expected results and drives pins for 1 CC
    // ------------------------------------------------------
    task automatic drive_pipeline(
        input coeff_t a0, input coeff_t a1, input coeff_t a2, input coeff_t a3,
        input coeff_t b0, input coeff_t b1, input coeff_t b2, input coeff_t b3,
        input pe_mode_e mode, input logic mode_sel, input string name
    );
        expected_result_t exp;

        // Initialize with default 0s
        exp.z0 = 0; exp.z1 = 0; exp.z2 = 0; exp.z3 = 0;
        exp.mode = mode;
        exp.mode_sel = mode_sel;

        // =======================================================
        // GOLDEN MODEL ROUTING
        // =======================================================
        if (mode == PE_MODE_CWM) begin
            // ---------------------------------------------------
            // CWM Mathematical Definition
            // f0 = a0, f1 = a1, g0 = a2, g1 = a3, w = b0
            // U3 = (f0*g0) + w*(f1*g1)
            // V0 = (g0+g1)*(f0+f1) - (f0*g0) - (f1*g1)
            // ---------------------------------------------------
            coeff_t f0_g0      = mod_mul(a0, a2);
            coeff_t f1_g1      = mod_mul(a1, a3);
            coeff_t f_sum      = mod_add(a0, a1);
            coeff_t g_sum      = mod_add(a2, a3);
            coeff_t sum_prod   = mod_mul(f_sum, g_sum);

            // Expected Z1 (Maps to PE3 U3 output)
            exp.z1 = mod_add(f0_g0, mod_mul(b0, f1_g1));

            // Expected Z2 (Maps to PE0 V0 output)
            exp.z2 = mod_sub(mod_sub(sum_prod, f0_g0), f1_g1);

            // Z0 and Z3 are unused in CWM, we will ignore them in the monitor
        end
        else if (mode == PE_MODE_NTT && mode_sel == 1'b0) begin
            // ---------------------------------------------------
            // NTT Radix-4 Mathematical Definition
            // X0=a0, X1=a1, X2=a2, X3=a3
            // w_2=b0, w_1=b1, w_3=b2, w_4^1=b3
            // U1 = (X0 + w_2*X2) + (X1*w_1 + X3*w_3)
            // V1 = (X0 + w_2*X2) - (X1*w_1 + X3*w_3)
            // U3 = (X0 - w_2*X2) + w_4^1*(X1*w_1 - X3*w_3)
            // V3 = (X0 - w_2*X2) - w_4^1*(X1*w_1 - X3*w_3)
            // ---------------------------------------------------

            // Stage 1 (PE0 & PE2)
            coeff_t pe0_u = mod_add(a0, mod_mul(a2, b0));
            coeff_t pe0_v = mod_sub(a0, mod_mul(a2, b0));
            coeff_t pe2_u = mod_add(mod_mul(a1, b1), mod_mul(a3, b2));
            coeff_t pe2_v = mod_sub(mod_mul(a1, b1), mod_mul(a3, b2));

            // Stage 2 (PE1 & PE3 mapped to Z outputs)
            exp.z0 = mod_add(pe0_u, pe2_u);                         // U1
            exp.z1 = mod_sub(pe0_u, pe2_u);                         // V1
            exp.z2 = mod_add(pe0_v, mod_mul(pe2_v, b3));            // U3
            exp.z3 = mod_sub(pe0_v, mod_mul(pe2_v, b3));            // V3
        end
        else if (mode == PE_MODE_NTT && mode_sel == 1'b1) begin
            // ---------------------------------------------------
            // NTT Radix-2 Mathematical Definition
            // X0=a0, X1=a1, X2=a2, X3=a3
            // w_A=b0, 1=b1, w_B=b2, unused=b3
            // U0 = X0 + w_A*X1
            // V0 = X0 - w_A*X1
            // U2 = X2*1 + X3*w_B
            // V2 = X2*1 - X3*w_B
            // ---------------------------------------------------

            coeff_t pe0_w_x2 = mod_mul(a1, b0);
            coeff_t pe2_x1_1 = mod_mul(a2, b1);                     // a2 * 1
            coeff_t pe2_w_x3 = mod_mul(a3, b2);

            exp.z0 = mod_add(a0, pe0_w_x2);                         // U0
            exp.z1 = mod_sub(a0, pe0_w_x2);                         // V0
            exp.z2 = mod_add(pe2_x1_1, pe2_w_x3);                   // U2
            exp.z3 = mod_sub(pe2_x1_1, pe2_w_x3);                   // V2
        end
        else if (mode == PE_MODE_INTT && mode_sel == 1'b0) begin
            // ---------------------------------------------------
            // INTT Radix-4 Mathematical Definition
            // X0=a0, X1=a1, X2=a2, X3=a3
            // w_2^-1=b0, w_1^-1=b1, w_3^-1=b2, w_4^-1=b3
            // U2 = (w4(X0 - X1) + (X2 - X3))w_1^-1
            // V2 = (w4(X0 - X1) - (X2 - X3))w_3^-1
            // U0 = ((X0 + X1)/2 + (X2 + X3)/2)/2
            // V0 = w_2^-1((X0 + X1)/2 - (X2 + X3)/2)
            // ---------------------------------------------------

            // Note: 1665 is the modulo 3329 inverse of 2 (i.e., division by 2)

            // Stage 1 (PE1 & PE3)
            coeff_t pe1_u = mod_mul(mod_add(a2, a3), 1665); // (X2 + X3) / 2
            coeff_t pe1_v = mod_sub(a2, a3);                // (X2 - X3)

            coeff_t pe3_u = mod_mul(mod_add(a0, a1), 1665); // (X0 + X1) / 2
            coeff_t pe3_v = mod_mul(mod_sub(a0, a1), b3);   // (X0 - X1) * w4

            // Stage 2 (PE0 & PE2 mapped to Z outputs)
            exp.z0 = mod_mul(mod_add(pe3_u, pe1_u), 1665);  // U0
            exp.z1 = mod_mul(mod_add(pe3_v, pe1_v), b1);    // U2
            exp.z2 = mod_mul(mod_sub(pe3_u, pe1_u), b0);    // V0
            exp.z3 = mod_mul(mod_sub(pe3_v, pe1_v), b2);    // V2
        end
        else if (mode == PE_MODE_INTT && mode_sel == 1'b1) begin
            // ---------------------------------------------------
            // INTT Radix-2 Mathematical Definition
            // X0=a0, X1=a1, X2=a2, X3=a3
            // w_A^-1=b0, 1665=b1, w_B^-1=b2, unused=b3
            // U0 = (X0 + X1)/2
            // V0 = (X0 - X1)w_A^-1
            // U2 = (X2 + X3)2^-1
            // V2 = (X2 - X3)w_B^-1
            // ---------------------------------------------------

            exp.z0 = mod_mul(mod_add(a0, a1), 1665);                // U0 (Hardcoded /2 in PE0)
            exp.z1 = mod_mul(mod_sub(a0, a1), b0);                  // V0
            exp.z2 = mod_mul(mod_add(a2, a3), b1);                  // U2 (Using b1 for /2)
            exp.z3 = mod_mul(mod_sub(a2, a3), b2);                  // V2
        end
        else if (mode == PE_MODE_ADDSUB && mode_sel == 1'b0) begin
            // ---------------------------------------------------
            // ADDSUB Mathematical Definition (Addition)
            // Z = X + Y (1-to-1 straight mapping)
            // ---------------------------------------------------
            exp.z0 = mod_add(a0, b0);
            exp.z1 = mod_add(a1, b1);
            exp.z2 = mod_add(a2, b2);
            exp.z3 = mod_add(a3, b3);
        end
        else if (mode == PE_MODE_ADDSUB && mode_sel == 1'b1) begin
            // ---------------------------------------------------
            // ADDSUB Mathematical Definition (Subtraction)
            // Z = X - Y (1-to-1 straight mapping)
            // ---------------------------------------------------
            exp.z0 = mod_sub(a0, b0);
            exp.z1 = mod_sub(a1, b1);
            exp.z2 = mod_sub(a2, b2);
            exp.z3 = mod_sub(a3, b3);
        end

        // 2. Push expected result to queue
        expected_queue.push_back(exp);
        name_queue.push_back(name);

        // 3. Drive Inputs
        @(posedge clk);
        valid_i      <= 1'b1;
        ctrl_i       <= mode;
        mode_i       <= mode_sel;
        op_a0_i <= a0; op_a1_i <= a1; op_a2_i <= a2; op_a3_i <= a3;
        op_b0_i <= b0; op_b1_i <= b1; op_b2_i <= b2; op_b3_i <= b3;
    endtask

    // ------------------------------------------------------
    // Task: Flush Pipeline (Waits for data to drain)
    // ------------------------------------------------------
    task automatic flush_pipeline();
        @(posedge clk);
        valid_i <= 1'b0;
        // Zero out data lines to prevent floating Xs in sim
        op_a0_i <= 0; op_a1_i <= 0; op_a2_i <= 0; op_a3_i <= 0;
        op_b0_i <= 0; op_b1_i <= 0; op_b2_i <= 0; op_b3_i <= 0;

        // Wait until all pushed data has been processed by the monitor
        wait(expected_queue.size() == 0);
        repeat(2) @(posedge clk); // Safety buffer
    endtask

    // ------------------------------------------------------
    // Monitor Process (Continuous Output Verification)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            expected_result_t exp;
            logic match;

            string exp_name;

            if (expected_queue.size() == 0) begin
                $display("==================================================");
                $display("[FATAL ERROR] valid_o is high, but expected_queue is empty!");
                $display("==================================================");
                error_count++;
            end else begin
                exp = expected_queue.pop_front();
                exp_name = name_queue.pop_front();
                match = 1'b1; // Assume pass until proven otherwise

                // Dynamic mode checking - Only check the ports active for the mode
                if (exp.mode == PE_MODE_CWM) begin
                    if (z1_o !== exp.z1 || z2_o !== exp.z2) match = 1'b0;
                end
                else if (exp.mode == PE_MODE_NTT || exp.mode == PE_MODE_INTT || exp.mode == PE_MODE_ADDSUB) begin
                    // NTT, INTT, and ADDSUB check all 4 outputs
                    if (z0_o !== exp.z0 || z1_o !== exp.z1 || z2_o !== exp.z2 || z3_o !== exp.z3) match = 1'b0;
                end

                if (!match) begin
                    $display("==================================================");
                    $display("[FAIL] %s", exp_name);
                    $display("Mode: %s", exp.mode.name());

                    // CWM only evaluates Z1 and Z2. All other modes evaluate all 4 ports.
                    if (exp.mode != PE_MODE_CWM && z0_o !== exp.z0) $display("   Z0 Mismatch! Exp: %0d, Got: %0d", exp.z0, z0_o);
                    if (z1_o !== exp.z1)                            $display("   Z1 Mismatch! Exp: %0d, Got: %0d", exp.z1, z1_o);
                    if (z2_o !== exp.z2)                            $display("   Z2 Mismatch! Exp: %0d, Got: %0d", exp.z2, z2_o);
                    if (exp.mode != PE_MODE_CWM && z3_o !== exp.z3) $display("   Z3 Mismatch! Exp: %0d, Got: %0d", exp.z3, z3_o);

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
        rst = 1;
        valid_i = 0;
        ctrl_i = PE_MODE_CWM;
        mode_i = 0;
        op_a0_i = 0; op_a1_i = 0; op_a2_i = 0; op_a3_i = 0;
        op_b0_i = 0; op_b1_i = 0; op_b2_i = 0; op_b3_i = 0;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE_UNIT Wrapper Verification");
        $display("==========================================================");

        // --------------------------------------------------
        // Pipelined Stream: CWM Mode
        // op_a mapping: [f0, f1, g0, g1]
        // op_b mapping: [w, unused, unused, unused]
        // --------------------------------------------------
        $display("--- Testing Streaming CWM Mode (8-Cycle Latency) ---");

        //              f0  f1  g0  g1    w   b1 b2 b3   Mode          ModeSel Name
        drive_pipeline(  0,  0,  0,  0,   0,  0, 0, 0,   PE_MODE_CWM,  1'b0,   "CWM: All Zeros");
        drive_pipeline(  1,  1,  1,  1,   1,  0, 0, 0,   PE_MODE_CWM,  1'b0,   "CWM: All Ones");
        drive_pipeline( 10,  5,  2,  4,  20,  0, 0, 0,   PE_MODE_CWM,  1'b0,   "CWM: Simple Math");
        drive_pipeline(100, 50,  0,  0,  10,  0, 0, 0,   PE_MODE_CWM,  1'b0,   "CWM: g is Zero");
        drive_pipeline(100, 20, 50, 40,   0,  0, 0, 0,   PE_MODE_CWM,  1'b0,   "CWM: w is Zero");
        drive_pipeline(3328, 3328, 3328, 3328, 3328, 0, 0, 0, PE_MODE_CWM, 1'b0, "CWM: Max Stress");

        // CWM Random Pipeline Saturation Test
        begin
            // 1. Declare the variables first
            coeff_t rf0, rf1, rg0, rg1, rw;

            for (int i = 0; i < 100; i++) begin
                // 2. Assign them inside the loop
                rf0 = $urandom_range(0, 3328);
                rf1 = $urandom_range(0, 3328);
                rg0 = $urandom_range(0, 3328);
                rg1 = $urandom_range(0, 3328);
                rw  = $urandom_range(0, 3328);

                drive_pipeline(rf0, rf1, rg0, rg1, rw, 0, 0, 0, PE_MODE_CWM, 1'b0, "CWM: Random Flow");
            end
        end
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: NTT Mode (Radix-4)
        // op_a mapping: [X_0, X_1, X_2, X_3]
        // op_b mapping: [w_2, w_1, w_3, w_4^1]
        // --------------------------------------------------
        $display("--- Testing Streaming NTT Mode (Radix-4, 8-Cycle Latency) ---");

        //              X0  X1  X2  X3   w2 w1 w3 w4^1 Mode          ModeSel Name
        drive_pipeline(  0,  0,  0,  0,   0, 0, 0, 0,  PE_MODE_NTT,  1'b0,   "NTT R4: All Zeros");
        drive_pipeline(  1,  1,  1,  1,   1, 1, 1, 1,  PE_MODE_NTT,  1'b0,   "NTT R4: All Ones");
        drive_pipeline( 10, 20, 30, 40,   2, 3, 4, 5,  PE_MODE_NTT,  1'b0,   "NTT R4: Simple Math");
        drive_pipeline(100,  0,  0,  0,   0, 0, 0, 0,  PE_MODE_NTT,  1'b0,   "NTT R4: X0 Only");
        drive_pipeline(3328, 3328, 3328, 3328, 3328, 3328, 3328, 3328, PE_MODE_NTT, 1'b0, "NTT R4: Max Stress");

        // NTT R4 Random Pipeline Saturation Test
        begin
            coeff_t rx0, rx1, rx2, rx3, rw2, rw1, rw3, rw4;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                rw2 = $urandom_range(0, 3328); rw1 = $urandom_range(0, 3328);
                rw3 = $urandom_range(0, 3328); rw4 = $urandom_range(0, 3328);
                drive_pipeline(rx0, rx1, rx2, rx3, rw2, rw1, rw3, rw4, PE_MODE_NTT, 1'b0, "NTT R4: Random Flow");
            end
        end
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: NTT Mode (Radix-2)
        // op_a mapping: [X_0, X_1, X_2, X_3]
        // op_b mapping: [w_A, 1, w_B, unused]
        // --------------------------------------------------
        $display("--- Testing Streaming NTT Mode (Radix-2, 4-Cycle Latency) ---");

        //              X0  X1  X2  X3   wA  1 wB d/c  Mode         ModeSel  Name
        drive_pipeline(  0,  0,  0,  0,   0, 1, 0, 0,  PE_MODE_NTT,  1'b1,   "NTT R2: All Zeros");
        drive_pipeline(  1,  1,  1,  1,   1, 1, 1, 0,  PE_MODE_NTT,  1'b1,   "NTT R2: All Ones");
        drive_pipeline( 10, 20, 30, 40,   2, 1, 3, 0,  PE_MODE_NTT,  1'b1,   "NTT R2: Simple Math");
        drive_pipeline(100,  0,  0,  0,   0, 1, 0, 0,  PE_MODE_NTT,  1'b1,   "NTT R2: X0 Only");
        drive_pipeline(3328, 3328, 3328, 3328, 3328, 1, 3328, 0, PE_MODE_NTT, 1'b1, "NTT R2: Max Stress");
        // NOTE: b1 is explicitly driven to 1 below to ensure PE2 correctly bypasses multiplication!

        // NTT R2 Random Pipeline Saturation Test
        begin
            coeff_t rx0, rx1, rx2, rx3, rwA, rwB;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                rwA = $urandom_range(0, 3328); rwB = $urandom_range(0, 3328);
                // Explicitly pinning b1 to 1 for the random test vectors
                drive_pipeline(rx0, rx1, rx2, rx3, rwA, 1, rwB, 0, PE_MODE_NTT, 1'b1, "NTT R2: Random Flow");
            end
        end
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: INTT Mode (Radix-4)
        // op_a mapping: [X_0, X_1, X_2, X_3]
        // op_b mapping: [w_2^-1, w_1^-1, w_3^-1, w_4^-1]
        // --------------------------------------------------
        $display("--- Testing Streaming INTT Mode (Radix-4, 8-Cycle Latency) ---");

        //              X0  X1  X2  X3  w2'w1'w3'w4'   Mode          ModeSel Name
        drive_pipeline(  0,  0,  0,  0,   0, 0, 0, 0,  PE_MODE_INTT, 1'b0,   "INTT R4: All Zeros");
        drive_pipeline(  1,  1,  1,  1,   1, 1, 1, 1,  PE_MODE_INTT, 1'b0,   "INTT R4: All Ones");
        drive_pipeline( 10, 20, 30, 40,   2, 3, 4, 5,  PE_MODE_INTT, 1'b0,   "INTT R4: Simple Math");
        drive_pipeline(100,  0,  0,  0,   0, 0, 0, 0,  PE_MODE_INTT, 1'b0,   "INTT R4: X0 Only");
        drive_pipeline(3328, 3328, 3328, 3328, 3328, 3328, 3328, 3328, PE_MODE_INTT, 1'b0, "INTT R4: Max Stress");

        // INTT R4 Random Pipeline Saturation Test
        begin
            coeff_t rx0, rx1, rx2, rx3, rw2, rw1, rw3, rw4;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                rw2 = $urandom_range(0, 3328); rw1 = $urandom_range(0, 3328);
                rw3 = $urandom_range(0, 3328); rw4 = $urandom_range(0, 3328);
                drive_pipeline(rx0, rx1, rx2, rx3, rw2, rw1, rw3, rw4, PE_MODE_INTT, 1'b0, "INTT R4: Random Flow");
            end
        end
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: INTT Mode (Radix-2)
        // op_a mapping: [X_0, X_1, X_2, X_3]
        // op_b mapping: [w_A^-1, 1665, w_B^-1, unused]
        // --------------------------------------------------
        $display("--- Testing Streaming INTT Mode (Radix-2, 4-Cycle Latency) ---");

        //                  X0  X1  X2  X3   wA' 1665 wB' d/c Mode          ModeSel Name
        drive_pipeline(  0,  0,  0,  0,   0, 1665, 0, 0,  PE_MODE_INTT, 1'b1,   "INTT R2: All Zeros");
        drive_pipeline(  1,  1,  1,  1,   1, 1665, 1, 0,  PE_MODE_INTT, 1'b1,   "INTT R2: All Ones");
        drive_pipeline( 10, 20, 30, 40,   2, 1665, 3, 0,  PE_MODE_INTT, 1'b1,   "INTT R2: Simple Math");
        drive_pipeline(100,  0,  0,  0,   0, 1665, 0, 0,  PE_MODE_INTT, 1'b1,   "INTT R2: X0 Only");
        drive_pipeline(3328, 3328, 3328, 3328, 3328, 1665, 3328, 0, PE_MODE_INTT, 1'b1, "INTT R2: Max Stress");

        begin
            coeff_t rx0, rx1, rx2, rx3, rwA, rwB;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                rwA = $urandom_range(0, 3328); rwB = $urandom_range(0, 3328);
                // Explicitly pinning b1 to 1665 (mod inverse of 2) for random vectors
                drive_pipeline(rx0, rx1, rx2, rx3, rwA, 1665, rwB, 0, PE_MODE_INTT, 1'b1, "INTT R2: Random Flow");
            end
        end
        flush_pipeline();

        // --------------------------------------------------
        // Pipelined Stream: ADDSUB Mode
        // op_a mapping: [X_0, X_1, X_2, X_3]
        // op_b mapping: [Y_0, Y_1, Y_2, Y_3]
        // --------------------------------------------------
        $display("--- Testing Streaming ADDSUB Mode (ADD, 1-Cycle Latency) ---");

        // -----------------------------------------
        // 1. ADDITION BLOCK (mode_i = 0)
        // -----------------------------------------
        //              X0  X1  X2  X3   Y0 Y1 Y2 Y3       Mode            ModeSel Name
        drive_pipeline(  0,  0,  0,  0,   0, 0, 0, 0,      PE_MODE_ADDSUB, 1'b0,   "ADD: All Zeros");
        drive_pipeline( 10, 20, 30, 40,   5, 5, 5, 5,      PE_MODE_ADDSUB, 1'b0,   "ADD: Simple Math");
        drive_pipeline(3328, 3328, 3328, 3328, 1, 1, 1, 1, PE_MODE_ADDSUB, 1'b0,   "ADD: Modulo Wrap");

        begin
            coeff_t rx0, rx1, rx2, rx3, ry0, ry1, ry2, ry3;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                ry0 = $urandom_range(0, 3328); ry1 = $urandom_range(0, 3328);
                ry2 = $urandom_range(0, 3328); ry3 = $urandom_range(0, 3328);
                drive_pipeline(rx0, rx1, rx2, rx3, ry0, ry1, ry2, ry3, PE_MODE_ADDSUB, 1'b0, "ADD: Random Flow");
            end
        end
        flush_pipeline(); // MUST FLUSH BEFORE SWITCHING MODE_I TO SUBTRACTION!

        $display("--- Testing Streaming ADDSUB Mode (SUB, 1-Cycle Latency) ---");
        // -----------------------------------------
        // 2. SUBTRACTION BLOCK (mode_i = 1)
        // -----------------------------------------
        //              X0  X1  X2  X3   Y0 Y1 Y2 Y3       Mode            ModeSel Name
        drive_pipeline( 10, 20, 30, 40,   5, 5, 5, 5,      PE_MODE_ADDSUB, 1'b1,   "SUB: Simple Math");
        drive_pipeline(  0,  0,  0,  0,   1, 1, 1, 1,      PE_MODE_ADDSUB, 1'b1,   "SUB: Modulo Wrap (0-1)");

        begin
            coeff_t rx0, rx1, rx2, rx3, ry0, ry1, ry2, ry3;
            for (int i = 0; i < 20; i++) begin
                rx0 = $urandom_range(0, 3328); rx1 = $urandom_range(0, 3328);
                rx2 = $urandom_range(0, 3328); rx3 = $urandom_range(0, 3328);
                ry0 = $urandom_range(0, 3328); ry1 = $urandom_range(0, 3328);
                ry2 = $urandom_range(0, 3328); ry3 = $urandom_range(0, 3328);
                drive_pipeline(rx0, rx1, rx2, rx3, ry0, ry1, ry2, ry3, PE_MODE_ADDSUB, 1'b1, "SUB: Random Flow");
            end
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
            $fatal(1, "pe_unit_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end
endmodule
