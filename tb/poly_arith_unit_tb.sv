/*
 * Module Name: poly_arith_unit_tb
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Description:
 * Integration testbench for poly_arith_unit. Instantiates the DUT directly
 * without poly_mem_subsystem. A lightweight memory model in the TB responds
 * to PAU read requests (1 cycle latency, lane-echo) and captures writebacks.
 *
 * Memory Model Protocol:
 *   - Primary port: 1cc read latency. Request beat latched; response issued
 *     next cycle with pau_rd_idx_i echoed back so CMI lane-match reorder works.
 *   - Auxiliary port: tied off (NTT does not use it).
 *   - Write: captured into result_mem[] on every pau_wr_en_o lane.
 *
 * Tests (NTT focus):
 *   Test 1 - NTT Random:   ntt_in.mem  → ntt_out.mem
 *   Test 2 - NTT Boundary: ntt_max_in.mem → ntt_max_out.mem
 */

`default_nettype none
`timescale 1ns/1ps

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module poly_arith_unit_tb;

    // =========================================================================
    // 1. Clock & Reset
    // =========================================================================
    logic clk = 0;
    logic rst = 1;
    always #5 clk = ~clk;

    // =========================================================================
    // 2. DUT Signals
    // =========================================================================
    logic         start_i          = 0;
    pe_mode_e     op_type_i        = PE_MODE_NTT;
    logic [POLY_ID_WIDTH-1:0] primary_poly_id_i = '0;
    logic [POLY_ID_WIDTH-1:0] aux_poly_id_i     = '0;
    logic [POLY_ID_WIDTH-1:0] cwm_num_terms_i   = '0;
    logic         done_o;

    // Primary memory port (DUT → TB)
    logic         pau_req_o;
    logic         pau_rd_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_rd_poly_id_o;
    logic [3:0][7:0]          pau_rd_idx_o;
    logic [3:0]               pau_rd_lane_valid_o;
    logic [3:0]               pau_wr_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_wr_poly_id_o;
    logic [3:0][7:0]          pau_wr_idx_o;
    logic [3:0][15:0]         pau_wr_data_o;

    // Primary memory port (TB → DUT)
    logic         pau_rd_valid_i   = 0;
    logic [POLY_ID_WIDTH-1:0] pau_rd_poly_id_i = '0;
    logic [3:0][7:0]          pau_rd_idx_i     = '0;
    logic [3:0]               pau_rd_lane_valid_i = '0;
    logic [3:0][15:0]         pau_rd_data_i    = '0;
    logic         pau_stall_i      = 0;

    // Auxiliary port (TB → DUT, tied off for NTT)
    logic         pau_aux_req_o;
    logic         pau_aux_rd_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_aux_rd_poly_id_o;
    logic [3:0][7:0]          pau_aux_rd_idx_o;
    logic [3:0]               pau_aux_rd_lane_valid_o;
    logic [3:0]               pau_aux_wr_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_aux_wr_poly_id_o;
    logic [3:0][7:0]          pau_aux_wr_idx_o;
    logic [3:0][15:0]         pau_aux_wr_data_o;

    logic         pau_aux_rd_valid_i   = 0;
    logic [POLY_ID_WIDTH-1:0] pau_aux_rd_poly_id_i = '0;
    logic [3:0][7:0]          pau_aux_rd_idx_i     = '0;
    logic [3:0]               pau_aux_rd_lane_valid_i = '0;
    logic [3:0][15:0]         pau_aux_rd_data_i    = '0;

    // =========================================================================
    // 3. DUT Instantiation
    // =========================================================================
    poly_arith_unit #(
        .NUM_POLYS     (NUM_POLYS),
        .CWM_NUM_TERMS (3)
    ) dut (
        .clk                    (clk),
        .rst                    (rst),
        .start_i                (start_i),
        .op_type_i              (op_type_i),
        .primary_poly_id_i      (primary_poly_id_i),
        .aux_poly_id_i          (aux_poly_id_i),
        .cwm_num_terms_i        (cwm_num_terms_i),
        .done_o                 (done_o),
        .pau_req_o              (pau_req_o),
        .pau_rd_en_o            (pau_rd_en_o),
        .pau_rd_poly_id_o       (pau_rd_poly_id_o),
        .pau_rd_idx_o           (pau_rd_idx_o),
        .pau_rd_lane_valid_o    (pau_rd_lane_valid_o),
        .pau_wr_en_o            (pau_wr_en_o),
        .pau_wr_poly_id_o       (pau_wr_poly_id_o),
        .pau_wr_idx_o           (pau_wr_idx_o),
        .pau_wr_data_o          (pau_wr_data_o),
        .pau_rd_valid_i         (pau_rd_valid_i),
        .pau_rd_poly_id_i       (pau_rd_poly_id_i),
        .pau_rd_idx_i           (pau_rd_idx_i),
        .pau_rd_lane_valid_i    (pau_rd_lane_valid_i),
        .pau_rd_data_i          (pau_rd_data_i),
        .pau_stall_i            (pau_stall_i),
        .pau_aux_req_o          (pau_aux_req_o),
        .pau_aux_rd_en_o        (pau_aux_rd_en_o),
        .pau_aux_rd_poly_id_o   (pau_aux_rd_poly_id_o),
        .pau_aux_rd_idx_o       (pau_aux_rd_idx_o),
        .pau_aux_rd_lane_valid_o(pau_aux_rd_lane_valid_o),
        .pau_aux_wr_en_o        (pau_aux_wr_en_o),
        .pau_aux_wr_poly_id_o   (pau_aux_wr_poly_id_o),
        .pau_aux_wr_idx_o       (pau_aux_wr_idx_o),
        .pau_aux_wr_data_o      (pau_aux_wr_data_o),
        .pau_aux_rd_valid_i     (pau_aux_rd_valid_i),
        .pau_aux_rd_poly_id_i   (pau_aux_rd_poly_id_i),
        .pau_aux_rd_idx_i       (pau_aux_rd_idx_i),
        .pau_aux_rd_lane_valid_i(pau_aux_rd_lane_valid_i),
        .pau_aux_rd_data_i      (pau_aux_rd_data_i)
    );

    // =========================================================================
    // 4. TB Memory Storage
    //    coeff_mem is the working memory — both reads and writes use it so
    //    in-place NTT passes see updated intermediate results.
    // =========================================================================
    logic [15:0] coeff_mem  [0:255];  // Working polynomial memory (in-place r/w)
    logic [15:0] expected   [0:255];  // Golden output

    // =========================================================================
    // 5. Pending Read State (1-cycle latency model)
    // =========================================================================
    logic           rd_pending_r     = 0;
    // Separate latched copies used for response (not overwritten same cycle)
    logic [POLY_ID_WIDTH-1:0] rd_resp_poly_id = '0;
    logic [3:0][7:0]          rd_resp_idx     = '0;
    logic [3:0]               rd_resp_valid   = '0;

    // Stage 1: Register the accepted request
    always_ff @(posedge clk) begin
        if (rst) begin
            rd_pending_r    <= 0;
            rd_resp_poly_id <= '0;
            rd_resp_idx     <= '0;
            rd_resp_valid   <= '0;
        end else begin
            // Accept request exactly when CMI does: req && rd_en && |lane_valid && !stall
            if (pau_req_o && pau_rd_en_o && (|pau_rd_lane_valid_o) && !pau_stall_i) begin
                rd_pending_r    <= 1;
                rd_resp_poly_id <= pau_rd_poly_id_o;
                rd_resp_idx     <= pau_rd_idx_o;
                rd_resp_valid   <= pau_rd_lane_valid_o;
            end else begin
                rd_pending_r    <= 0;
            end
        end
    end

    // Stage 2: Combinational response — drives pau_rd_valid_i same cycle CMI reads it
    always_comb begin
        pau_rd_valid_i      = rd_pending_r;
        pau_rd_poly_id_i    = rd_resp_poly_id;
        pau_rd_idx_i        = rd_resp_idx;
        pau_rd_lane_valid_i = rd_resp_valid;
        pau_rd_data_i       = '0;
        if (rd_pending_r) begin
            for (int lane = 0; lane < 4; lane++) begin
                if (rd_resp_valid[lane])
                    pau_rd_data_i[lane] = {4'b0, coeff_mem[rd_resp_idx[lane]][11:0]};
            end
        end
    end

    // =========================================================================
    // 6. Writeback — write back into coeff_mem (in-place, supports multi-pass)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst) begin
            for (int lane = 0; lane < 4; lane++) begin
                if (pau_wr_en_o[lane]) begin
                    coeff_mem[pau_wr_idx_o[lane]] <= {4'b0, pau_wr_data_o[lane][11:0]};
                end
            end
        end
    end

    // =========================================================================
    // 7. Debug Monitor
    // =========================================================================
    int cycle_cnt = 0;
    always_ff @(posedge clk) begin
        if (rst) begin
            cycle_cnt <= 0;
        end else begin
            cycle_cnt <= cycle_cnt + 1;

            // Read issue
            if (pau_req_o && pau_rd_en_o && !pau_stall_i) begin
                $display("[%0d] RD_ISSUE: poly=%0d idx={%0d,%0d,%0d,%0d} valid=%b",
                    cycle_cnt,
                    pau_rd_poly_id_o,
                    pau_rd_idx_o[0], pau_rd_idx_o[1],
                    pau_rd_idx_o[2], pau_rd_idx_o[3],
                    pau_rd_lane_valid_o);
            end

            // Read response
            if (pau_rd_valid_i) begin
                $display("[%0d] RD_RESP:  poly=%0d idx={%0d,%0d,%0d,%0d} data={%03x,%03x,%03x,%03x}",
                    cycle_cnt,
                    pau_rd_poly_id_i,
                    pau_rd_idx_i[0], pau_rd_idx_i[1],
                    pau_rd_idx_i[2], pau_rd_idx_i[3],
                    pau_rd_data_i[0][11:0], pau_rd_data_i[1][11:0],
                    pau_rd_data_i[2][11:0], pau_rd_data_i[3][11:0]);
            end

            // Write beat
            if (|pau_wr_en_o) begin
                $display("[%0d] WR_BEAT:  idx={%0d,%0d,%0d,%0d} en=%b data={%03x,%03x,%03x,%03x}",
                    cycle_cnt,
                    pau_wr_idx_o[0], pau_wr_idx_o[1],
                    pau_wr_idx_o[2], pau_wr_idx_o[3],
                    pau_wr_en_o,
                    pau_wr_data_o[0][11:0], pau_wr_data_o[1][11:0],
                    pau_wr_data_o[2][11:0], pau_wr_data_o[3][11:0]);
            end
        end
    end

    // =========================================================================
    // 8. Watchdog
    // =========================================================================
    localparam int WATCHDOG_CYCLES = 5000;
    int watchdog_cnt = 0;
    logic test_running = 0;

    always_ff @(posedge clk) begin
        if (rst || !test_running) begin
            watchdog_cnt <= 0;
        end else begin
            watchdog_cnt <= watchdog_cnt + 1;
            if (watchdog_cnt >= WATCHDOG_CYCLES) begin
                $fatal(1, "[FATAL] Watchdog timeout after %0d cycles. DUT done_o never asserted.",
                       WATCHDOG_CYCLES);
            end
        end
    end

    // =========================================================================
    // 9. Tasks
    // =========================================================================

    // Clear coeff_mem before each test
    task automatic clear_result_mem;
        for (int i = 0; i < 256; i++)
            coeff_mem[i] = '0;
    endtask

    // Run one PAU operation and wait for done_o
    task automatic run_pau(input pe_mode_e mode);
        @(negedge clk);
        op_type_i = mode;
        start_i   = 1;
        test_running = 1;
        @(negedge clk);
        start_i = 0;
        // Wait for done_o
        @(posedge clk);
        while (!done_o) @(posedge clk);
        test_running = 0;
        // One extra cycle to allow final writeback to settle
        @(posedge clk);
    endtask

    // Compare result_mem vs expected, return mismatch count
    // Compare coeff_mem vs expected (coeff_mem has final in-place result)
    function automatic int compare_results(input string test_name);
        int mismatches = 0;
        for (int i = 0; i < 256; i++) begin
            if (coeff_mem[i][11:0] !== expected[i][11:0]) begin
                $display("[MISMATCH] %s coeff[%0d]: got %03x, expected %03x",
                         test_name, i, coeff_mem[i][11:0], expected[i][11:0]);
                mismatches++;
            end
        end
        return mismatches;
    endfunction

    // =========================================================================
    // 10. Main Execution
    // =========================================================================
    int total_pass = 0;
    int total_fail = 0;
    int mismatches;

    initial begin
        // Reset
        rst = 1;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 0;
        @(posedge clk);

        $display("==================================================");
        $display("  poly_arith_unit Integration Testbench");
        $display("==================================================");

        // ----------------------------------------------------------------
        // Test 1: NTT Random
        // ----------------------------------------------------------------
        $display("\n=== TEST 1: NTT (Random Input) ===");
        clear_result_mem();
        $readmemh("verif/vectors/k2/ntt_in.mem",  coeff_mem);
        $readmemh("verif/vectors/k2/ntt_out.mem", expected);

        run_pau(PE_MODE_NTT);

        mismatches = compare_results("NTT_RANDOM");
        if (mismatches == 0) begin
            $display("[PASS] NTT Random: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] NTT Random: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        // Cool-down between tests
        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 2: NTT Boundary (all q-1)
        // ----------------------------------------------------------------
        $display("\n=== TEST 2: NTT (Boundary: all q-1) ===");
        clear_result_mem();
        $readmemh("verif/vectors/k2/ntt_max_in.mem",  coeff_mem);
        $readmemh("verif/vectors/k2/ntt_max_out.mem", expected);

        run_pau(PE_MODE_NTT);

        mismatches = compare_results("NTT_MAX");
        if (mismatches == 0) begin
            $display("[PASS] NTT Boundary: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] NTT Boundary: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Summary
        // ----------------------------------------------------------------
        $display("\n==================================================");
        $display("  TEST SUMMARY: PASS=%0d  FAIL=%0d", total_pass, total_fail);
        $display("==================================================");
        if (total_fail == 0)
            $display("  RESULT: SUCCESS");
        else
            $display("  RESULT: FAILED");
        $display("==================================================\n");

        if (total_fail != 0)
            $fatal(1, "poly_arith_unit_tb failed");

        #50 $finish;
    end

endmodule

`default_nettype wire
