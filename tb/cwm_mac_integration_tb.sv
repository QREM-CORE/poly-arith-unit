// ==========================================================
// Integration Testbench: PE_Unit (CWM) -> MAC Adder
// Author(s): Salwan Aldhahab
// Target: FIPS 203 (ML-KEM) - CWM + MAC Datapath
//
// Description:
// End-to-end integration test that wires pe_unit in CWM mode
// to mac_adder, verifying the full multiply-accumulate datapath
// used for matrix-vector multiplication in ML-KEM.
//
// Architecture Under Test:
//   [zeta_stim]--[delay 3]--+
//                            v
//   pe_unit (CWM) --- z1_o --[delay 1]--> mac_adder.a0_i  (c0 lane)
//                 --- z2_o -------------> mac_adder.a1_i  (c1 lane)
//                 --- valid_o [delay 4]-> mac_adder.valid_i
//   TB (sim PM)   --- acc0 ------------> mac_adder.b0_i
//                 --- acc1 ------------> mac_adder.b1_i
//   TB control    --- first_term ------> mac_adder.first_term_i
//
// Pipeline Alignment (Critical Design Notes):
//
// 1. OUTPUT ALIGNMENT (z1 vs z2 vs valid):
//    In CWM mode, pe_unit has cross-PE feedback creating
//    different path latencies:
//      z1_o (PE2->PE3 path): 3CC(PE2) + 4CC(PE3) = 7CC
//      z2_o (PE1/PE2->PE0 path): 4CC(PE1)+4CC(PE0) = 8CC
//      valid_o (PE0 delay_4): 4CC (under-reports true latency)
//    Fix: delay z1 by 1CC, delay valid by 4CC -> all at 8CC.
//
// 2. ZETA INPUT ALIGNMENT (w0 -> PE3):
//    PE3 reads w3_i (=w0_i=zeta) combinationally via
//    mod_mul_op1_i = delay_1_w_data_i (ctrl[0]=0 in CWM).
//    But PE3's multiplier doesn't use zeta until cycle 3
//    when PE2's 3CC-pipelined outputs arrive as b3.
//    If zeta is driven for only 1 cycle with the data, it's
//    gone by cycle 3 -> PE3 computes 0*f1g1 = 0.
//    Fix: delay zeta_stim by 3CC before feeding to pe_w0.
//    For streaming, this also aligns pair K's zeta with
//    pair K's PE2 result (both at cycle K+3).
//
// Total CWM+MAC pipeline latency: 8CC(PE) + 1CC(MAC) = 9CC
//
// Golden Model:
// BaseCaseMultiply from FIPS 203 Algorithm 12:
//   c0 = f0*g0 + f1*g1*zeta  (mod q)  ->  z1_o (PE3 u3_o)
//   c1 = -(f0*g1 + f1*g0) mod q       ->  z2_o (PE0 v0_o)
//     [Karatsuba: PE0 computes (f0g0+f1g1) - (f0+f1)(g0+g1)]
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module cwm_mac_integration_tb();

    // ------------------------------------------------------
    // Constants
    // ------------------------------------------------------
    localparam int MODULUS       = 3329;
    localparam int CWM_LATENCY   = 8;   // Cycles from pe_unit input to aligned z1/z2
    localparam int MAC_LATENCY   = 1;   // mac_adder registered output
    localparam int TOTAL_LATENCY = CWM_LATENCY + MAC_LATENCY;

    // Number of streaming coefficient pairs per pass
    localparam int NUM_PAIRS = 8;

    // ------------------------------------------------------
    // Clock & Reset
    // ------------------------------------------------------
    logic clk, rst;

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ------------------------------------------------------
    // PE_Unit Signals
    // ------------------------------------------------------
    logic       pe_valid_i;
    pe_mode_e   pe_ctrl_i = PE_MODE_CWM;  // Default avoids time-0 X
    coeff_t     pe_x0, pe_x1, pe_x2, pe_x3;
    coeff_t     pe_y0, pe_y1, pe_y2, pe_y3;
    coeff_t     pe_w0, pe_w1, pe_w2, pe_w3;
    coeff_t     pe_z0, pe_z1, pe_z2, pe_z3;
    logic       pe_valid_o;

    // Zeta alignment staging signal
    // TB drives this; it passes through a 3CC delay before reaching pe_w0.
    coeff_t     zeta_stim = '0;
    coeff_t     zeta_delayed;

    // MAC Adder Signals
    logic       mac_first_term_i;
    logic       mac_valid_i;
    coeff_t     mac_a0_i, mac_a1_i;
    coeff_t     mac_b0_i, mac_b1_i;
    coeff_t     mac_z0_o, mac_z1_o;
    logic       mac_valid_o;

    // Alignment Wires
    coeff_t     z1_aligned;         // z1 delayed by 1CC to align with z2
    logic       valid_aligned;      // valid delayed by 4CC to align with data

    // Verification Stats
    int pass_count  = 0;
    int error_count = 0;

    // Simulated Polynomial Memory (for MAC feedback)
    coeff_t sim_pm_c0 [0:NUM_PAIRS-1];
    coeff_t sim_pm_c1 [0:NUM_PAIRS-1];

    // Golden Model Storage
    typedef struct {
        coeff_t c0;     // f0*g0 + zeta*f1*g1 (mod q)
        coeff_t c1;     // negated Karatsuba cross-term from PE0
    } cwm_result_t;

    // ==========================================================
    // DUT Instantiations
    // ==========================================================

    // -------- PE Unit (Arithmetic Unit) --------
    pe_unit u_pe_unit (
        .clk        (clk),
        .rst        (rst),
        .valid_i    (pe_valid_i),
        .ctrl_i     (pe_ctrl_i),
        .x0_i       (pe_x0),
        .x1_i       (pe_x1),
        .x2_i       (pe_x2),
        .x3_i       (pe_x3),
        .y0_i       (pe_y0),
        .y1_i       (pe_y1),
        .y2_i       (pe_y2),
        .y3_i       (pe_y3),
        .w0_i       (pe_w0),
        .w1_i       (pe_w1),
        .w2_i       (pe_w2),
        .w3_i       (pe_w3),
        .z0_o       (pe_z0),
        .z1_o       (pe_z1),
        .z2_o       (pe_z2),
        .z3_o       (pe_z3),
        .valid_o    (pe_valid_o)
    );

    // ==========================================================
    // Pipeline Alignment Delays
    // ==========================================================

    // -------- Alignment Delay: z1 by 1CC --------
    // PE3 path (z1) is 7CC, PE0 path (z2) is 8CC.
    // Delay z1 by 1CC to align both at 8CC.
    delay_n #(
        .DWIDTH (COEFF_WIDTH),
        .DEPTH  (1)
    ) u_align_z1 (
        .clk    (clk),
        .rst    (rst),
        .data_i (pe_z1),
        .data_o (z1_aligned)
    );

    // -------- Alignment Delay: valid by 4CC --------
    // pe_unit.valid_o fires at 4CC but data arrives at 8CC.
    // Delay valid by 4 extra CCs to match the data.
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (4)
    ) u_align_valid (
        .clk    (clk),
        .rst    (rst),
        .data_i (pe_valid_o),
        .data_o (valid_aligned)
    );

    // -------- Alignment Delay: zeta (w0) by 3CC --------
    // PE3 samples w3_i combinationally but doesn't use it until
    // PE2's outputs arrive 3CC later. Delay zeta to align with
    // PE2 -> PE3 data flow. For streaming, pair K's zeta exits
    // the delay at cycle K+3, matching PE2's result for pair K.
    delay_n #(
        .DWIDTH (COEFF_WIDTH),
        .DEPTH  (3)
    ) u_align_zeta (
        .clk    (clk),
        .rst    (rst),
        .data_i (zeta_stim),
        .data_o (zeta_delayed)
    );
    assign pe_w0 = zeta_delayed;

    // -------- MAC Adder --------
    mac_adder u_mac_adder (
        .clk        (clk),
        .rst        (rst),
        .first_term_i(mac_first_term_i),
        .valid_i    (mac_valid_i),
        .a0_i       (mac_a0_i),
        .b0_i       (mac_b0_i),
        .a1_i       (mac_a1_i),
        .b1_i       (mac_b1_i),
        .z0_o       (mac_z0_o),
        .z1_o       (mac_z1_o),
        .valid_o    (mac_valid_o)
    );

    // Wire PE CWM outputs (aligned) to MAC adder inputs
    assign mac_a0_i    = z1_aligned;   // c0 lane (BaseCaseMultiply even result)
    assign mac_a1_i    = pe_z2;        // c1 lane (Karatsuba cross-term from PE0)
    assign mac_valid_i = valid_aligned;

    // ==========================================================
    // Golden Model Functions
    // ==========================================================

    function automatic coeff_t gm_mod_add(input coeff_t a, input coeff_t b);
        logic [12:0] s;
        s = a + b;
        return (s >= MODULUS) ? coeff_t'(s - MODULUS) : coeff_t'(s);
    endfunction

    function automatic coeff_t gm_mod_sub(input coeff_t a, input coeff_t b);
        logic [12:0] d;
        d = a - b;
        return d[12] ? coeff_t'(d + MODULUS) : coeff_t'(d);
    endfunction

    function automatic coeff_t gm_mod_mul(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) * longint'(b)) % MODULUS);
    endfunction

    // BaseCaseMultiply: c0 = f0*g0 + zeta*f1*g1, c1 = -(f0*g1 + f1*g0)
    function automatic cwm_result_t gm_basecase_multiply(
        input coeff_t f0, input coeff_t f1,
        input coeff_t g0, input coeff_t g1,
        input coeff_t zeta
    );
        cwm_result_t result;
        coeff_t p1, p2, cross_term;

        p1         = gm_mod_mul(f0, g0);
        p2         = gm_mod_mul(f1, g1);
        cross_term = gm_mod_add(gm_mod_mul(f0, g1), gm_mod_mul(f1, g0));

        result.c0 = gm_mod_add(p1, gm_mod_mul(zeta, p2));
        // PE0 Karatsuba: (f0g0+f1g1) - (f0+f1)(g0+g1) = -(f0g1+f1g0) mod q
        result.c1 = gm_mod_sub(12'd0, cross_term);

        return result;
    endfunction

    // ==========================================================
    // Test Data Storage
    // ==========================================================

    typedef struct {
        coeff_t f0, f1;
        coeff_t g0, g1;
        coeff_t zeta;
    } cwm_stimulus_t;

    cwm_stimulus_t stim_pass0 [0:NUM_PAIRS-1];
    cwm_stimulus_t stim_pass1 [0:NUM_PAIRS-1];

    cwm_result_t   exp_pass0 [0:NUM_PAIRS-1];
    cwm_result_t   exp_pass1 [0:NUM_PAIRS-1];

    coeff_t        exp_acc_c0 [0:NUM_PAIRS-1];
    coeff_t        exp_acc_c1 [0:NUM_PAIRS-1];

    // ==========================================================
    // Task: Generate random stimulus for one pass
    // ==========================================================
    task automatic generate_stimulus(
        output cwm_stimulus_t stim [0:NUM_PAIRS-1],
        output cwm_result_t   expected [0:NUM_PAIRS-1]
    );
        for (int i = 0; i < NUM_PAIRS; i++) begin
            stim[i].f0   = $urandom_range(0, MODULUS - 1);
            stim[i].f1   = $urandom_range(0, MODULUS - 1);
            stim[i].g0   = $urandom_range(0, MODULUS - 1);
            stim[i].g1   = $urandom_range(0, MODULUS - 1);
            stim[i].zeta = ZETA_MUL_TABLE[$urandom_range(0, 127)];

            expected[i] = gm_basecase_multiply(
                stim[i].f0, stim[i].f1,
                stim[i].g0, stim[i].g1,
                stim[i].zeta
            );
        end
    endtask

    // ==========================================================
    // Task: Drive one streaming CWM pass through pe_unit
    // Drives NUM_PAIRS coefficient pairs back-to-back.
    // Zeta goes through zeta_stim -> 3CC delay -> pe_w0.
    // ==========================================================
    task automatic drive_cwm_pass(
        input cwm_stimulus_t stim [0:NUM_PAIRS-1]
    );
        for (int i = 0; i < NUM_PAIRS; i++) begin
            @(posedge clk);
            pe_valid_i <= 1'b1;
            pe_ctrl_i  <= PE_MODE_CWM;

            // CWM mapping: x0=f0, x1=f1, y0=g0, y1=g1
            pe_x0 <= stim[i].f0;
            pe_x1 <= stim[i].f1;
            pe_y0 <= stim[i].g0;
            pe_y1 <= stim[i].g1;

            // Zeta through staging -> delay_n(3) -> pe_w0
            zeta_stim <= stim[i].zeta;

            // Unused inputs for CWM
            pe_x2 <= '0; pe_x3 <= '0;
            pe_y2 <= '0; pe_y3 <= '0;
            pe_w1 <= '0; pe_w2 <= '0; pe_w3 <= '0;
        end

        // Deassert valid after last pair
        @(posedge clk);
        pe_valid_i <= 1'b0;
        pe_x0 <= '0; pe_x1 <= '0;
        pe_y0 <= '0; pe_y1 <= '0;
        zeta_stim <= '0;
    endtask

    // ==========================================================
    // Task: Drive one pass and collect MAC outputs
    // Manages the first-term/accumulate control and PM feedback.
    // ==========================================================
    task automatic drive_cwm_mac_pass(
        input cwm_stimulus_t stim [0:NUM_PAIRS-1],
        input logic          is_first_pass
    );
        // Set MAC first-term control
        mac_first_term_i <= is_first_pass ? 1'b1 : 1'b0;

        // Start PM feedback driver BEFORE CWM drive so feedback
        // values (sim_pm) are pre-loaded on mac_b0/b1 well before
        // the first valid_aligned fires.
        fork
            begin : feedback_driver
                int fb_idx;
                fb_idx = 0;
                forever begin
                    @(posedge clk);
                    if (!is_first_pass) begin
                        // On valid_aligned, advance fb_idx FIRST so the
                        // subsequent NBA schedules the NEXT pair's feedback.
                        // The MAC FF samples the CURRENT mac_b0_i (set by
                        // the prior cycle's NBA = current pair's value).
                        if (valid_aligned && fb_idx < NUM_PAIRS) begin
                            fb_idx++;
                        end
                        if (fb_idx < NUM_PAIRS) begin
                            mac_b0_i <= sim_pm_c0[fb_idx];
                            mac_b1_i <= sim_pm_c1[fb_idx];
                        end else begin
                            mac_b0_i <= '0;
                            mac_b1_i <= '0;
                        end
                    end else begin
                        mac_b0_i <= '0;
                        mac_b1_i <= '0;
                    end
                end
            end
        join_none

        // Drive CWM stimulus (feedback driver already running)
        drive_cwm_pass(stim);

        // Wait for pipeline to fill and all results to emerge
        repeat(TOTAL_LATENCY + NUM_PAIRS + 5) @(posedge clk);

        disable feedback_driver;
    endtask

    // ==========================================================
    // Monitor: Capture and verify MAC outputs
    // ==========================================================
    int output_idx;
    int current_test_phase;   // 0=idle, 1=init_pass, 2=acc_pass

    always @(posedge clk) begin
        if (mac_valid_o && current_test_phase > 0) begin
            coeff_t exp_c0, exp_c1;
            string  phase_name;

            phase_name = (current_test_phase == 1) ? "INIT" :
                         (current_test_phase == 2) ? "ACC-k2" : "ACC-k3";

            exp_c0 = exp_acc_c0[output_idx];
            exp_c1 = exp_acc_c1[output_idx];

            if (mac_z0_o !== exp_c0 || mac_z1_o !== exp_c1) begin
                $display("==================================================");
                $display("[FAIL] %s Pair[%0d]", phase_name, output_idx);
                if (mac_z0_o !== exp_c0)
                    $display("   Lane 0 (c0) Mismatch! Exp: %0d, Got: %0d", exp_c0, mac_z0_o);
                if (mac_z1_o !== exp_c1)
                    $display("   Lane 1 (c1) Mismatch! Exp: %0d, Got: %0d", exp_c1, mac_z1_o);
                $display("==================================================");
                error_count++;
            end else begin
                pass_count++;
            end

            // Store in simulated PM for next pass
            sim_pm_c0[output_idx] = mac_z0_o;
            sim_pm_c1[output_idx] = mac_z1_o;

            output_idx++;
        end
    end

    // ==========================================================
    // Phase 0: Pipeline Latency Calibration
    // ==========================================================
    task automatic calibrate_pipeline();
        int cycle_count;
        cwm_result_t cal_exp;
        logic z1_found, z2_found, mac_found;

        $display("--- Phase 0: Pipeline Latency Calibration ---");

        rst = 1;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        // Drive a single non-zero CWM pair
        // f0=10, f1=20, g0=30, g1=40, zeta=17
        @(posedge clk);
        pe_valid_i <= 1'b1;
        pe_ctrl_i  <= PE_MODE_CWM;
        pe_x0 <= 12'd10;  pe_x1 <= 12'd20;
        pe_y0 <= 12'd30;  pe_y1 <= 12'd40;
        zeta_stim <= 12'd17;
        pe_x2 <= '0; pe_x3 <= '0;
        pe_y2 <= '0; pe_y3 <= '0;
        pe_w1 <= '0; pe_w2 <= '0; pe_w3 <= '0;

        @(posedge clk);
        pe_valid_i <= 1'b0;
        pe_x0 <= '0; pe_x1 <= '0;
        pe_y0 <= '0; pe_y1 <= '0;
        zeta_stim <= '0;

        // Compute expected
        cal_exp = gm_basecase_multiply(12'd10, 12'd20, 12'd30, 12'd40, 12'd17);
        $display("   Golden Model: c0=%0d, c1_neg=%0d", cal_exp.c0, cal_exp.c1);

        // Monitor all signals for 20 cycles
        cycle_count = 2;
        z1_found = 0; z2_found = 0; mac_found = 0;

        for (int i = 0; i < 20; i++) begin
            @(posedge clk);
            cycle_count++;

            if (z1_aligned == cal_exp.c0 && z1_aligned != 12'd0 && !z1_found) begin
                $display("   z1_aligned matched c0=%0d at cycle %0d", cal_exp.c0, cycle_count);
                z1_found = 1;
            end

            if (pe_z2 == cal_exp.c1 && pe_z2 != 12'd0 && !z2_found) begin
                $display("   z2 matched c1_neg=%0d at cycle %0d", cal_exp.c1, cycle_count);
                z2_found = 1;
            end

            if (valid_aligned) begin
                $display("   valid_aligned asserted at cycle %0d", cycle_count);
            end

            if (mac_valid_o) begin
                $display("   mac_valid_o asserted at cycle %0d (mac z0=%0d, z1=%0d)",
                         cycle_count, mac_z0_o, mac_z1_o);

                if (mac_z0_o == cal_exp.c0 && mac_z1_o == cal_exp.c1 && !mac_found) begin
                    $display("   [PASS] MAC output matches golden model!");
                    mac_found = 1;
                    pass_count++;
                end
            end
        end

        if (!mac_found) begin
            $display("   [FAIL] MAC output never matched golden model.");
            $display("          z1_found=%0b, z2_found=%0b", z1_found, z2_found);
            error_count++;
        end

        $display("");
    endtask

    // ==========================================================
    // Phase 1: Single-Shot CWM -> MAC Init Verification
    // ==========================================================
    task automatic test_single_shot_init();
        cwm_result_t exp;
        logic matched;

        $display("--- Phase 1: Single-Shot CWM -> MAC Init ---");

        rst = 1;
        mac_first_term_i = 1'b1;
        mac_b0_i = '0; mac_b1_i = '0;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        // Compute expected for f0=100, f1=200, g0=300, g1=400, zeta=17
        exp = gm_basecase_multiply(12'd100, 12'd200, 12'd300, 12'd400, 12'd17);
        $display("   Expected: c0=%0d, c1_neg=%0d", exp.c0, exp.c1);

        // Drive single CWM
        @(posedge clk);
        pe_valid_i <= 1'b1;
        pe_ctrl_i  <= PE_MODE_CWM;
        mac_first_term_i <= 1'b1;
        pe_x0 <= 12'd100; pe_x1 <= 12'd200;
        pe_y0 <= 12'd300; pe_y1 <= 12'd400;
        zeta_stim <= 12'd17;
        pe_x2 <= '0; pe_x3 <= '0;
        pe_y2 <= '0; pe_y3 <= '0;
        pe_w1 <= '0; pe_w2 <= '0; pe_w3 <= '0;

        @(posedge clk);
        pe_valid_i <= 1'b0;
        pe_x0 <= '0; pe_x1 <= '0;
        pe_y0 <= '0; pe_y1 <= '0;
        zeta_stim <= '0;

        // Wait for MAC output
        matched = 0;
        for (int i = 0; i < TOTAL_LATENCY + 5; i++) begin
            @(posedge clk);
            if (mac_valid_o && !matched) begin
                $display("   MAC output: z0=%0d (exp %0d), z1=%0d (exp %0d)",
                         mac_z0_o, exp.c0, mac_z1_o, exp.c1);
                if (mac_z0_o == exp.c0 && mac_z1_o == exp.c1) begin
                    $display("   [PASS] Single-shot first-term passthrough correct!");
                    pass_count++;
                end else begin
                    $display("   [FAIL] Single-shot first-term mismatch!");
                    error_count++;
                end
                matched = 1;
            end
        end

        if (!matched) begin
            $display("   [FAIL] No MAC output received!");
            error_count++;
        end

        $display("");
    endtask

    // ==========================================================
    // Phase 3: Streaming CWM -> MAC with k=2 Accumulation
    // Simulates: result = A[i][0]*s[0] + A[i][1]*s[1]
    // ==========================================================
    task automatic test_streaming_mac_k2();
        $display("--- Phase 3: Streaming CWM -> MAC (k=2) ---");

        // Generate stimulus
        generate_stimulus(stim_pass0, exp_pass0);
        generate_stimulus(stim_pass1, exp_pass1);

        // --- Pass 0: Init (j=0) ---
        $display("   Pass 0 (Init)...");
        rst = 1;
        mac_first_term_i = 1'b1;
        mac_b0_i = '0; mac_b1_i = '0;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        // Expected for first-term pass: just pass0 results
        for (int i = 0; i < NUM_PAIRS; i++) begin
            exp_acc_c0[i] = exp_pass0[i].c0;
            exp_acc_c1[i] = exp_pass0[i].c1;
        end
        output_idx = 0;
        current_test_phase = 1;

        drive_cwm_mac_pass(stim_pass0, 1'b1);

        repeat(5) @(posedge clk);
        current_test_phase = 0;

        $display("   Pass 0 collected %0d outputs.", output_idx);

        // --- Pass 1: Accumulate (j=1) ---
        $display("   Pass 1 (Accumulate)...");

        // Drain pipeline between passes
        repeat(10) @(posedge clk);

        // Expected for accumulate pass: pass0 + pass1
        for (int i = 0; i < NUM_PAIRS; i++) begin
            exp_acc_c0[i] = gm_mod_add(exp_pass0[i].c0, exp_pass1[i].c0);
            exp_acc_c1[i] = gm_mod_add(exp_pass0[i].c1, exp_pass1[i].c1);
        end
        output_idx = 0;
        current_test_phase = 2;

        drive_cwm_mac_pass(stim_pass1, 1'b0);

        repeat(5) @(posedge clk);
        current_test_phase = 0;

        $display("   Pass 1 collected %0d outputs.", output_idx);
        $display("");
    endtask

    // ==========================================================
    // Phase 2: Known-Value Directed Test
    // Uses hand-calculated values for deterministic verification.
    // ==========================================================
    task automatic test_known_values();
        cwm_result_t exp_j0, exp_j1;
        coeff_t expected_acc_c0, expected_acc_c1;
        logic found_j0, found_j1;

        $display("--- Phase 2: Known-Value Directed Test ---");

        // j=0: f=(1,0), g=(1,0), zeta=17
        // c0 = 1*1 + 17*0*0 = 1, c1 = -(1*0 + 0*1) = 0
        exp_j0 = gm_basecase_multiply(12'd1, 12'd0, 12'd1, 12'd0, 12'd17);
        $display("   j=0 Golden: c0=%0d, c1_neg=%0d", exp_j0.c0, exp_j0.c1);

        // j=1: f=(2,3), g=(4,5), zeta=17
        // c0 = 2*4 + 17*3*5 = 8+255 = 263, c1 = -(2*5+3*4) = -(22) = 3307
        exp_j1 = gm_basecase_multiply(12'd2, 12'd3, 12'd4, 12'd5, 12'd17);
        $display("   j=1 Golden: c0=%0d, c1_neg=%0d", exp_j1.c0, exp_j1.c1);

        // Accumulated: c0 = 1+263=264, c1_neg = (0+3307)%3329=3307
        expected_acc_c0 = gm_mod_add(exp_j0.c0, exp_j1.c0);
        expected_acc_c1 = gm_mod_add(exp_j0.c1, exp_j1.c1);
        $display("   Acc Golden: c0=%0d, c1=%0d", expected_acc_c0, expected_acc_c1);

        // --- Drive j=0 (first term) ---
        rst = 1;
        mac_first_term_i = 1'b1;
        mac_b0_i = '0; mac_b1_i = '0;
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        @(posedge clk);
        pe_valid_i <= 1'b1;
        pe_ctrl_i  <= PE_MODE_CWM;
        pe_x0 <= 12'd1; pe_x1 <= 12'd0;
        pe_y0 <= 12'd1; pe_y1 <= 12'd0;
        zeta_stim <= 12'd17;
        pe_x2 <= '0; pe_x3 <= '0; pe_y2 <= '0; pe_y3 <= '0;
        pe_w1 <= '0; pe_w2 <= '0; pe_w3 <= '0;

        @(posedge clk);
        pe_valid_i <= 1'b0;
        pe_x0 <= '0; pe_x1 <= '0;
        pe_y0 <= '0; pe_y1 <= '0;
        zeta_stim <= '0;

        // Wait for j=0 MAC output and capture result immediately.
        // The MAC output register updates every cycle (not gated by
        // valid_i), so z0_o is overwritten to 0 after mac_valid_o
        // deasserts. We MUST capture during the mac_valid_o pulse.
        found_j0 = 0;
        begin
            coeff_t saved_c0, saved_c1;
            saved_c0 = '0;
            saved_c1 = '0;

            for (int i = 0; i < TOTAL_LATENCY + 5; i++) begin
                @(posedge clk);
                if (mac_valid_o && !found_j0) begin
                    saved_c0 = mac_z0_o;
                    saved_c1 = mac_z1_o;
                    $display("   After j=0: mac_z0=%0d (exp %0d), mac_z1=%0d (exp %0d)",
                             mac_z0_o, exp_j0.c0, mac_z1_o, exp_j0.c1);
                    found_j0 = 1;
                end
            end

            repeat(10) @(posedge clk);
            mac_first_term_i <= 1'b0;
            mac_b0_i <= saved_c0;
            mac_b1_i <= saved_c1;

            @(posedge clk);
            pe_valid_i <= 1'b1;
            pe_ctrl_i  <= PE_MODE_CWM;
            pe_x0 <= 12'd2; pe_x1 <= 12'd3;
            pe_y0 <= 12'd4; pe_y1 <= 12'd5;
            zeta_stim <= 12'd17;
            pe_x2 <= '0; pe_x3 <= '0; pe_y2 <= '0; pe_y3 <= '0;
            pe_w1 <= '0; pe_w2 <= '0; pe_w3 <= '0;

            @(posedge clk);
            pe_valid_i <= 1'b0;
            pe_x0 <= '0; pe_x1 <= '0;
            pe_y0 <= '0; pe_y1 <= '0;
            zeta_stim <= '0;

            found_j1 = 0;
            for (int i = 0; i < TOTAL_LATENCY + 5; i++) begin
                @(posedge clk);
                if (mac_valid_o && !found_j1) begin
                    $display("   After j=1: mac_z0=%0d (exp %0d), mac_z1=%0d (exp %0d)",
                             mac_z0_o, expected_acc_c0, mac_z1_o, expected_acc_c1);

                    if (mac_z0_o == expected_acc_c0 && mac_z1_o == expected_acc_c1) begin
                        $display("   [PASS] Known-value accumulation correct!");
                        pass_count++;
                    end else begin
                        $display("   [FAIL] Known-value accumulation mismatch!");
                        error_count++;
                    end
                    found_j1 = 1;
                end
            end

            if (!found_j1) begin
                $display("   [FAIL] No MAC output for j=1!");
                error_count++;
            end
        end

        $display("");
    endtask

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize all signals
        rst = 1;
        pe_valid_i = 0;
        pe_ctrl_i = PE_MODE_CWM;
        pe_x0 = 0; pe_x1 = 0; pe_x2 = 0; pe_x3 = 0;
        pe_y0 = 0; pe_y1 = 0; pe_y2 = 0; pe_y3 = 0;
        zeta_stim = 0; pe_w1 = 0; pe_w2 = 0; pe_w3 = 0;
        mac_first_term_i = 1;
        mac_b0_i = 0; mac_b1_i = 0;
        output_idx = 0;
        current_test_phase = 0;

        // Clear simulated PM
        for (int i = 0; i < NUM_PAIRS; i++) begin
            sim_pm_c0[i] = '0;
            sim_pm_c1[i] = '0;
        end

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting CWM + MAC Integration Verification");
        $display("==========================================================");
        $display("Configuration:");
        $display("  CWM Pipeline Latency: %0d CC", CWM_LATENCY);
        $display("  MAC Adder Latency:    %0d CC", MAC_LATENCY);
        $display("  Total Latency:        %0d CC", TOTAL_LATENCY);
        $display("  z1 alignment delay:   1 CC (PE3 7CC -> 8CC)");
        $display("  valid alignment delay: 4 CC (valid 4CC -> 8CC)");
        $display("  zeta alignment delay:  3 CC (w0 to PE3 multiplier)");
        $display("  Pairs per pass:       %0d", NUM_PAIRS);
        $display("==========================================================");
        $display("");

        // ---- Run Test Phases ----
        calibrate_pipeline();          // Phase 0
        test_single_shot_init();       // Phase 1
        test_known_values();           // Phase 2
        test_streaming_mac_k2();       // Phase 3

        // ---- Final Summary ----
        repeat(10) @(posedge clk);
        $display("==========================================================");
        $display("CWM + MAC INTEGRATION TESTBENCH SUMMARY");
        $display("----------------------------------------------------------");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", error_count);
        $display("----------------------------------------------------------");

        if (error_count == 0 && pass_count > 0)
            $display("RESULT: SUCCESS");
        else if (error_count > 0) begin
            $display("RESULT: FAILURE");
            $display("NOTE: If lane alignment mismatches occur, adjust");
            $display("      CWM_LATENCY and alignment delay parameters.");
        end else begin
            $display("RESULT: INCONCLUSIVE (no outputs captured)");
            $display("NOTE: The pe_unit CWM cross-PE pipeline may need");
            $display("      different alignment delays. Run calibration.");
        end

        $display("==========================================================");
        $finish;
    end

endmodule
