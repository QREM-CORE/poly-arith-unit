// ==========================================================
// Testbench for Twiddle Factor ROM + Address Generator
// Author: Salwan Aldhahab
// Description: Verifies that the tf_rom (4-ROM architecture)
//              and tf_addr_gen (sequential counter) modules
//              produce the correct twiddle factor values for
//              all NTT and INTT passes.
//
//              The testbench checks:
//              1. r4_addr / r2_addr sequences from tf_addr_gen
//              2. ROM output values (w0, w1, w2, w3) from tf_rom
//              3. Pass transitions and total cycle counts
//              4. Pre-negated INTT values are correct
// ==========================================================
`timescale 1ns/1ps

module tf_rom_addr_gen_tb;

    import poly_arith_pkg::*;
    import qrem_global_pkg::*;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic           clk;
    logic           rst;

    // Address Generator
    logic           start;
    pe_mode_e       mode;
    logic [1:0]     pass_idx;

    logic [5:0]     tf_addr;
    logic           is_intt;
    logic           is_radix2;
    logic           ag_valid;
    logic [1:0]     pass_num;
    logic           advance;

    // ROM
    coeff_t         w0, w1, w2, w3;

    // =========================================================================
    // Clock Generation (100 MHz)
    // =========================================================================
    localparam int CLK_PERIOD = 10;

    initial clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    tf_addr_gen u_tf_addr_gen (
        .clk            (clk),
        .rst            (rst),
        .start_i        (start),
        .advance_i      (advance),
        .ctrl_i         (mode),
        .pass_idx_i     (pass_idx),
        .tf_addr_o      (tf_addr),
        .is_radix2_o    (is_radix2),
        .valid_o        (ag_valid),
        .pass_o         (pass_num)
    );

    // is_intt is derived from mode (controller responsibility)
    assign is_intt = (mode == PE_MODE_INTT);

    tf_rom u_tf_rom (
        .clk            (clk),
        .rst            (rst),
        .is_intt_i      (is_intt),
        .is_radix2_i    (is_radix2),
        .is_cwm_i       (1'b0),
        .tf_addr_i      (tf_addr),
        .w0_o           (w0),
        .w1_o           (w1),
        .w2_o           (w2),
        .w3_o           (w3)
    );

    // =========================================================================
    // Expected ROM Contents (first few values for spot-check)
    // =========================================================================

    // R4NTT_ROM[0] = {17, 289, 1584} -> w0=289, w1=17, w2=1584
    // R4NTT_ROM[1] = {296, 1062, 1409} -> w0=1062, w1=296, w2=1409
    // OMEGA_ROM[0] = 1729
    // R4INTT_ROM[0] = {2761, 2649, 331} -> w0=2649, w1=2761, w2=331
    // OMEGA_INV_ROM[0] = 17

    // =========================================================================
    // Test Variables
    // =========================================================================
    int errors = 0;
    int total_tests = 0;
    int cycle_count;

    // =========================================================================
    // Task: Wait for N clock cycles
    // =========================================================================
    task automatic wait_cycles(int n);
        repeat(n) @(negedge clk);
    endtask

    // =========================================================================
    // Task: Verify NTT Address Sequence & ROM Values
    // =========================================================================
    task automatic verify_ntt();
        int expected_r4, expected_r2;
        int num_blocks, bfs_per_block;
        int ntt_errors_start;

        ntt_errors_start = errors;

        $display("===================================================");
        $display("  VERIFYING NTT TWIDDLE FACTOR GENERATION");
        $display("===================================================");

        // Start NTT Pass 1
        @(negedge clk);
        mode = PE_MODE_NTT;
        pass_idx = 2'd0;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        expected_r4 = 0;

        // === NTT Pass 1 (R4, stages 1&2): 1 block x 64 BFs ===
        $display("\n--- NTT Pass 1 (Radix-4, Stages 1&2) ---");
        num_blocks = 1;
        bfs_per_block = 64;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                // 1. Check outputs first on the settled negedge
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("NTT P1 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end

                if (is_radix2 !== 1'b0) begin
                    $error("NTT P1: is_radix2 should be 0");
                    errors++;
                end

                if (pass_num !== 2'd0) begin
                    $error("NTT P1: pass_num=%0d, expected=0", pass_num);
                    errors++;
                end

                // 2. Advance the clock for the next iteration
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 1: Verified %0d cycles, r4 counter ended at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd1;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === NTT Pass 2 (R4, stages 3&4): 4 blocks x 16 BFs ===
        $display("\n--- NTT Pass 2 (Radix-4, Stages 3&4) ---");
        num_blocks = 4;
        bfs_per_block = 16;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("NTT P2 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 2: Verified %0d cycles, r4 counter ended at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd2;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === NTT Pass 3 (R4, stages 5&6): 16 blocks x 4 BFs ===
        $display("\n--- NTT Pass 3 (Radix-4, Stages 5&6) ---");
        num_blocks = 16;
        bfs_per_block = 4;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("NTT P3 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 3: Verified %0d cycles, r4 counter ended at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Verify r4 counter reached 21 (0..20 all used)
        if (expected_r4 !== 21) begin
            $error("NTT: r4 counter should reach 21, got %0d", expected_r4);
            errors++;
        end

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd3;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === NTT Pass 4 (R2, stage 7): 64 blocks x 1 cycle (2 parallel BFs) ===
        $display("\n--- NTT Pass 4 (Radix-2, Stage 7) ---");
        num_blocks = 64;
        bfs_per_block = 1;
        expected_r2 = 0;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r2)) begin
                    $error("NTT P4 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r2);
                    errors++;
                end

                if (is_radix2 !== 1'b1) begin
                    $error("NTT P4: is_radix2 should be 1");
                    errors++;
                end
                @(negedge clk);
            end
            expected_r2++;
        end
        $display("  Pass 4: Verified %0d cycles", cycle_count);
        advance = 1'b0;

        // Wait for FSM to return to IDLE
        wait (ag_valid == 1'b0);
        @(negedge clk);
        if (ag_valid) begin
            $error("NTT: Address generator still busy after all passes!");
            errors++;
        end

        $display("\n  NTT VERIFICATION COMPLETE: %0d errors in %0d tests",
                 errors - ntt_errors_start, total_tests);
    endtask

    // =========================================================================
    // Task: Verify INTT Address Sequence & ROM Values
    // =========================================================================
    task automatic verify_intt();
        int expected_r4, expected_r2;
        int num_blocks, bfs_per_block;
        int intt_errors_start;

        intt_errors_start = errors;

        $display("\n===================================================");
        $display("  VERIFYING INTT TWIDDLE FACTOR GENERATION");
        $display("===================================================");

        // Start INTT Pass 1
        @(negedge clk);
        mode = PE_MODE_INTT;
        pass_idx = 2'd0;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === INTT Pass 1 (R2, stage 1): 64 blocks x 1 cycle (2 parallel BFs) ===
        $display("\n--- INTT Pass 1 (Radix-2, Stage 1) ---");
        num_blocks = 64;
        bfs_per_block = 1;
        expected_r2 = 0;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r2)) begin
                    $error("INTT P1 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r2);
                    errors++;
                end

                if (is_radix2 !== 1'b1) begin
                    $error("INTT P1: is_radix2 should be 1");
                    errors++;
                end

                if (is_intt !== 1'b1) begin
                    $error("INTT P1: is_intt should be 1");
                    errors++;
                end
                @(negedge clk);
            end
            expected_r2++;
        end
        $display("  Pass 1: Verified %0d cycles", cycle_count);
        advance = 1'b0;

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd1;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === INTT Pass 2 (R4, stages 2&3): 16 blocks x 4 BFs ===
        $display("\n--- INTT Pass 2 (Radix-4, Stages 2&3) ---");
        num_blocks = 16;
        bfs_per_block = 4;
        expected_r4 = 0;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("INTT P2 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end

                if (is_radix2 !== 1'b0) begin
                    $error("INTT P2: is_radix2 should be 0");
                    errors++;
                end
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 2: Verified %0d cycles, r4 counter at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd2;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === INTT Pass 3 (R4, stages 4&5): 4 blocks x 16 BFs ===
        $display("\n--- INTT Pass 3 (Radix-4, Stages 4&5) ---");
        num_blocks = 4;
        bfs_per_block = 16;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("INTT P3 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 3: Verified %0d cycles, r4 counter at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Wait for FSM to return to IDLE, then start next pass
        wait (ag_valid == 1'b0);
        @(negedge clk);
        pass_idx = 2'd3;
        start = 1'b1;
        advance = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // === INTT Pass 4 (R4, stages 6&7): 1 block x 64 BFs ===
        $display("\n--- INTT Pass 4 (Radix-4, Stages 6&7) ---");
        num_blocks = 1;
        bfs_per_block = 64;
        cycle_count = 0;

        for (int b = 0; b < num_blocks; b++) begin
            for (int j = 0; j < bfs_per_block; j++) begin
                total_tests++;
                cycle_count++;

                if (tf_addr !== 6'(expected_r4)) begin
                    $error("INTT P4 B%0d BF%0d: tf_addr=%0d, expected=%0d",
                           b, j, tf_addr, expected_r4);
                    errors++;
                end
                @(negedge clk);
            end
            expected_r4++;
        end
        $display("  Pass 4: Verified %0d cycles, r4 counter at %0d", cycle_count, expected_r4);
        advance = 1'b0;

        // Verify r4 counter reached 21
        if (expected_r4 !== 21) begin
            $error("INTT: r4 counter should reach 21, got %0d", expected_r4);
            errors++;
        end

        // Wait for FSM to return to IDLE
        wait (ag_valid == 1'b0);
        @(negedge clk);
        if (ag_valid) begin
            $error("INTT: Address generator still busy after all passes!");
            errors++;
        end

        $display("\n  INTT VERIFICATION COMPLETE: %0d errors in %0d tests",
                 errors - intt_errors_start, total_tests);
    endtask

    // =========================================================================
    // Task: Verify ROM Output Values (spot-check after ROM registration delay)
    // =========================================================================
    task automatic verify_rom_values();
        int rom_errors_start;

        rom_errors_start = errors;

        $display("\n===================================================");
        $display("  VERIFYING ROM OUTPUT VALUES (Spot-Check)");
        $display("===================================================");

        // --- NTT: Check first Radix-4 entry ---
        $display("\n--- NTT Radix-4 Pass 1 ROM Values ---");
        @(negedge clk);
        mode = PE_MODE_NTT;
        pass_idx = 2'd0;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // Address gen outputs r4_addr=0 immediately on S_PASS_1 entry.
        // ROM is registered, so output appears 1 cycle later.
        @(negedge clk); // Cycle 1: addr_gen outputs addr=0
        @(negedge clk); // Cycle 2: ROM outputs registered value for addr=0

        // R4NTT_ROM[0] = {2580, 1729, 3289}
        // w0 = r4_w2 = 1729 (Stage B), w1 = r4_w1 = 2580 (Stage A top), w2 = r4_w3 = 3289 (Stage A bot)
        total_tests += 4;
        if (w0 !== 12'd1729) begin
            $error("NTT ROM: w0=%0d, expected=1729 (r4_w2 from R4NTT_ROM[0])", w0);
            errors++;
        end
        if (w1 !== 12'd2580) begin
            $error("NTT ROM: w1=%0d, expected=2580 (r4_w1 from R4NTT_ROM[0])", w1);
            errors++;
        end
        if (w2 !== 12'd3289) begin
            $error("NTT ROM: w2=%0d, expected=3289 (r4_w3 from R4NTT_ROM[0])", w2);
            errors++;
        end
        if (w3 !== 12'd1729) begin
            $error("NTT ROM: w3=%0d, expected=1729 (OMEGA_4_NTT)", w3);
            errors++;
        end
        $display("  NTT R4[0]: w0=%0d w1=%0d w2=%0d w3=%0d %s",
                 w0, w1, w2, w3,
                 (w0 == 1729 && w1 == 2580 && w2 == 3289 && w3 == 1729) ? "PASS" : "FAIL");

        // Let NTT run to completion
        advance = 1'b1;
        wait (ag_valid == 1'b0);
        advance = 1'b0;
        @(negedge clk);

        // --- INTT: Check first Radix-2 entry ---
        $display("\n--- INTT Radix-2 Pass 1 ROM Values ---");
        @(negedge clk);
        mode = PE_MODE_INTT;
        pass_idx = 2'd0;
        start = 1'b1;
        advance = 1'b0; // No advance during spot check
        @(negedge clk);
        start = 1'b0;

        @(negedge clk); // Cycle 1: addr_gen outputs addr
        @(negedge clk); // Cycle 2: ROM outputs registered value

        // OMEGA_INV_ROM[0] = 2252
        // w0 = r2_data = 2252, w1 = 1665 (INV_2_MOD_Q), w2 = r2_data = 2252
        total_tests += 4;
        if (w0 !== 12'd2252) begin
            $error("INTT ROM R2: w0=%0d, expected=2252", w0);
            errors++;
        end
        if (w1 !== 12'd1665) begin
            $error("INTT ROM R2: w1=%0d, expected=1665 (INV_2_MOD_Q for INTT R2)", w1);
            errors++;
        end
        if (w2 !== 12'd2252) begin
            $error("INTT ROM R2: w2=%0d, expected=2252 (same twiddle as w0)", w2);
            errors++;
        end
        if (w3 !== 12'd1729) begin
            $error("INTT ROM: w3=%0d, expected=1729 (OMEGA_4)", w3);
            errors++;
        end
        $display("  INTT R2[0]: w0=%0d w1=%0d w2=%0d w3=%0d %s",
                 w0, w1, w2, w3,
                 (w0 == 2252 && w1 == 1665 && w2 == 2252 && w3 == 1729) ? "PASS" : "FAIL");

        // Let INTT run to completion
        advance = 1'b1;
        wait (ag_valid == 1'b0);
        advance = 1'b0;
        @(negedge clk);

        $display("\n  ROM VALUE CHECK COMPLETE: %0d errors",
                 errors - rom_errors_start);
    endtask

    // =========================================================================
    // Task: Verify Total Cycle Counts
    // =========================================================================
    task automatic verify_cycle_counts();
        int ntt_cycles, intt_cycles;

        $display("\n===================================================");
        $display("  VERIFYING TOTAL CYCLE COUNTS");
        $display("===================================================");

        // NTT: 64 + 64 + 64 + 64 = 256 cycles (4 passes)
        ntt_cycles = 0;
        for (int p = 0; p < 4; p++) begin
            @(negedge clk);
            mode = PE_MODE_NTT;
            pass_idx = p[1:0];
            start = 1'b1;
            advance = 1'b1;
            @(negedge clk);
            start = 1'b0;

            // Corrected check-then-wait logic!
            while (ag_valid) begin
                ntt_cycles++;
                @(negedge clk);
            end
            advance = 1'b0;
            wait (ag_valid == 1'b0);
            @(negedge clk);
        end

        total_tests++;
        if (ntt_cycles !== 256) begin
            $error("NTT cycle count: %0d, expected 256", ntt_cycles);
            errors++;
        end
        $display("  NTT: %0d cycles %s", ntt_cycles, (ntt_cycles == 256) ? "PASS" : "FAIL");

        @(negedge clk);

        // INTT: 64 + 64 + 64 + 64 = 256 cycles (4 passes)
        intt_cycles = 0;
        for (int p = 0; p < 4; p++) begin
            @(negedge clk);
            mode = PE_MODE_INTT;
            pass_idx = p[1:0];
            start = 1'b1;
            advance = 1'b1;
            @(negedge clk);
            start = 1'b0;

            // Corrected check-then-wait logic!
            while (ag_valid) begin
                intt_cycles++;
                @(negedge clk);
            end

            advance = 1'b0;

            wait (ag_valid == 1'b0);
            @(negedge clk);
        end

        total_tests++;
        if (intt_cycles !== 256) begin
            $error("INTT cycle count: %0d, expected 256", intt_cycles);
            errors++;
        end
        $display("  INTT: %0d cycles %s", intt_cycles, (intt_cycles == 256) ? "PASS" : "FAIL");
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $display("\n==========================================================");
        $display(" Twiddle Factor ROM + Address Generator Testbench");
        $display(" Architecture: 4-ROM (R4NTT, OMEGA, R4INTT, OMEGA_INV)");
        $display(" Addressing: Sequential t++ counter");
        $display("==========================================================\n");

        // Initialize
        rst      = 1'b1;
        start    = 1'b0;
        advance  = 1'b0;
        mode     = PE_MODE_NTT;
        pass_idx = 2'd0;

        // Hold reset for 5 cycles
        repeat(5) @(negedge clk);
        rst = 1'b0;
        repeat(2) @(negedge clk);

        // Run all verification tasks
        verify_ntt();
        wait_cycles(5);

        verify_intt();
        wait_cycles(5);

        verify_rom_values();
        wait_cycles(5);

        verify_cycle_counts();
        wait_cycles(5);

        // Final Summary
        $display("\n==========================================================");
        if (errors == 0) begin
            $display(" ALL %0d TESTS PASSED!", total_tests);
        end else begin
            $display(" FAILED: %0d errors in %0d tests", errors, total_tests);
        end
        $display("==========================================================\n");

        $finish;
    end

    // Simulation timeout
    initial begin
        #1_000_000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
