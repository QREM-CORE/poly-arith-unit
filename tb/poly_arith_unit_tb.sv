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
 *   - Auxiliary port: 1cc read latency, same protocol as primary.
 *   - Both ports index coeff_mem[poly_id][coeff_idx] — 2D poly memory.
 *   - Write: captured into coeff_mem[poly_id][coeff_idx] on every pau_wr_en_o lane.
 *
 * Tests:
 *   Test 1 - NTT Random:   ntt_in.mem  → ntt_out.mem
 *   Test 2 - NTT Boundary: ntt_max_in.mem → ntt_max_out.mem
 *   Test 3 - INTT Random:  intt_in.mem → intt_out.mem
 *   Test 4 - INTT Boundary: intt_max_in.mem → intt_max_out.mem
 *   Test 5 - CWM k=1:      cwm_a0 * cwm_s0 + cwm_e → cwm_k1_out.mem
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

    // Auxiliary port (DUT → TB)
    logic         pau_aux_req_o;
    logic         pau_aux_rd_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_aux_rd_poly_id_o;
    logic [3:0][7:0]          pau_aux_rd_idx_o;
    logic [3:0]               pau_aux_rd_lane_valid_o;
    logic [3:0]               pau_aux_wr_en_o;
    logic [POLY_ID_WIDTH-1:0] pau_aux_wr_poly_id_o;
    logic [3:0][7:0]          pau_aux_wr_idx_o;
    logic [3:0][15:0]         pau_aux_wr_data_o;

    // Auxiliary port (TB → DUT)
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
    //    coeff_mem[poly_id][coeff_idx] — indexed by polynomial ID.
    //    NTT/INTT use poly_id 0 only (in-place). CWM uses poly_ids 0..2.
    // =========================================================================
    localparam int MEM_DEPTH = 256;
    logic [15:0] coeff_mem [0:NUM_POLYS-1][0:MEM_DEPTH-1];
    logic [15:0] expected  [0:MEM_DEPTH-1];

    // =========================================================================
    // 5a. Primary Port — Pending Read State (1-cycle latency model)
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
                    pau_rd_data_i[lane] = {4'b0, coeff_mem[rd_resp_poly_id][rd_resp_idx[lane]][11:0]};
            end
        end
    end

    // =========================================================================
    // 5b. Auxiliary Port — Pending Read State (1-cycle latency model)
    // =========================================================================
    logic           aux_rd_pending_r     = 0;
    logic [POLY_ID_WIDTH-1:0] aux_rd_resp_poly_id = '0;
    logic [3:0][7:0]          aux_rd_resp_idx     = '0;
    logic [3:0]               aux_rd_resp_valid   = '0;

    always_ff @(posedge clk) begin
        if (rst) begin
            aux_rd_pending_r    <= 0;
            aux_rd_resp_poly_id <= '0;
            aux_rd_resp_idx     <= '0;
            aux_rd_resp_valid   <= '0;
        end else begin
            if (pau_aux_req_o && pau_aux_rd_en_o && (|pau_aux_rd_lane_valid_o)) begin
                aux_rd_pending_r    <= 1;
                aux_rd_resp_poly_id <= pau_aux_rd_poly_id_o;
                aux_rd_resp_idx     <= pau_aux_rd_idx_o;
                aux_rd_resp_valid   <= pau_aux_rd_lane_valid_o;
            end else begin
                aux_rd_pending_r    <= 0;
            end
        end
    end

    always_comb begin
        pau_aux_rd_valid_i      = aux_rd_pending_r;
        pau_aux_rd_poly_id_i    = aux_rd_resp_poly_id;
        pau_aux_rd_idx_i        = aux_rd_resp_idx;
        pau_aux_rd_lane_valid_i = aux_rd_resp_valid;
        pau_aux_rd_data_i       = '0;
        if (aux_rd_pending_r) begin
            for (int lane = 0; lane < 4; lane++) begin
                if (aux_rd_resp_valid[lane])
                    pau_aux_rd_data_i[lane] = {4'b0, coeff_mem[aux_rd_resp_poly_id][aux_rd_resp_idx[lane]][11:0]};
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
                    coeff_mem[pau_wr_poly_id_o][pau_wr_idx_o[lane]] <= {4'b0, pau_wr_data_o[lane][11:0]};
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

            // Aux read issue
            if (pau_aux_req_o && pau_aux_rd_en_o) begin
                $display("[%0d] AUX_ISSUE: poly=%0d idx={%0d,%0d,%0d,%0d} valid=%b",
                    cycle_cnt,
                    pau_aux_rd_poly_id_o,
                    pau_aux_rd_idx_o[0], pau_aux_rd_idx_o[1],
                    pau_aux_rd_idx_o[2], pau_aux_rd_idx_o[3],
                    pau_aux_rd_lane_valid_o);
            end

            // Write beat
            if (|pau_wr_en_o) begin
                $display("[%0d] WR_BEAT:  poly=%0d idx={%0d,%0d,%0d,%0d} en=%b data={%03x,%03x,%03x,%03x}",
                    cycle_cnt,
                    pau_wr_poly_id_o,
                    pau_wr_idx_o[0], pau_wr_idx_o[1],
                    pau_wr_idx_o[2], pau_wr_idx_o[3],
                    pau_wr_en_o,
                    pau_wr_data_o[0][11:0], pau_wr_data_o[1][11:0],
                    pau_wr_data_o[2][11:0], pau_wr_data_o[3][11:0]);
            end
        end
    end

    // =========================================================================
    // 7b. CWM Pipeline Trace (fires only during CWM test, first few pairs)
    //     Uses hierarchical references to probe internal DUT signals.
    //     Compare against scripts/cwm_trace_ref.py for root-cause isolation.
    // =========================================================================
    logic cwm_trace_en = 0;   // set in main block before run_pau(PE_MODE_CWM)
    int   cwm_issue_cnt = 0;  // counts accepted CWM issue beats
    int   cwm_pe_out_cnt = 0; // counts valid PE output beats

    localparam int CWM_TRACE_PAIRS = 4; // trace first N pairs only

    always_ff @(posedge clk) begin
        if (!rst && cwm_trace_en) begin

            // ------------------------------------------------------------------
            // Stage 1: Controller → CMI issue beat
            // Fires when controller issues a CWM read to CMI (issue_fire)
            // ------------------------------------------------------------------
            if (dut.u_controller.state_r == 3'd2 &&  // S_RUN
                dut.u_controller.op_r    == PE_MODE_CWM &&
                dut.cmi_ready &&
                cwm_issue_cnt < CWM_TRACE_PAIRS) begin

                $display("[CWM-CTRL][cc=%0d] pair=%0d term=%0d | pri_poly=%0d aux_poly=%0d | coeff_idx={%0d,%0d}",
                    cycle_cnt,
                    dut.u_controller.issue_addr_r[6:0],
                    dut.u_controller.cwm_term_idx_r,
                    dut.cmi_poly_id,
                    dut.cmi_aux_poly_id,
                    dut.u_controller.cmi_coeff_idx_o[0],
                    dut.u_controller.cmi_coeff_idx_o[1]);
                cwm_issue_cnt <= cwm_issue_cnt + 1;
            end

            // ------------------------------------------------------------------
            // Stage 2: CMI response — what coefficients arrived at the PE
            // Fires one cycle after a valid read response (pau_rd_valid_i)
            // ------------------------------------------------------------------
            if (pau_rd_valid_i && cwm_issue_cnt > 0 && cwm_issue_cnt <= CWM_TRACE_PAIRS + 1) begin
                $display("[CWM-CMI ][cc=%0d] coeff_from_cmi (A): a0=%03x a1=%03x | aux (s): b0=%03x b1=%03x",
                    cycle_cnt,
                    dut.coeff_from_cmi[0][11:0],
                    dut.coeff_from_cmi[1][11:0],
                    dut.aux_coeff_from_cmi[0][11:0],
                    dut.aux_coeff_from_cmi[1][11:0]);
            end

            // ------------------------------------------------------------------
            // Stage 3+4: PE output → row accumulator input
            // Fires when pe_wb_valid asserted for CWM mode
            // ------------------------------------------------------------------
            if (dut.pe_wb_valid && dut.pe_ctrl == PE_MODE_CWM &&
                cwm_pe_out_cnt < CWM_TRACE_PAIRS) begin
                $display("[CWM-PE  ][cc=%0d] valid | omega=%03x | z1(c0)=%03x z2(c1)=%03x",
                    cycle_cnt,
                    dut.w0_cwm_aligned,
                    dut.z1_o,
                    dut.z2_o);
                $display("[CWM-ACCM][cc=%0d] acc_fire=%b first=%b pair=%0d | in: cwm0=%03x cwm1=%03x",
                    cycle_cnt,
                    dut.u_row_accum.acc_fire_i,
                    dut.u_row_accum.first_term_i,
                    dut.u_row_accum.pair_idx_i,
                    dut.u_row_accum.cwm0_i,
                    dut.u_row_accum.cwm1_i);
                cwm_pe_out_cnt <= cwm_pe_out_cnt + 1;
            end

            // ------------------------------------------------------------------
            // Stage 5: Drain output
            // Fires when drain_valid_o rises during the drain phase
            // ------------------------------------------------------------------
            if (dut.u_row_accum.drain_valid_o && dut.u_row_accum.fuse_e_i &&
                dut.u_controller.cwm_drain_idx_r < CWM_TRACE_PAIRS) begin
                $display("[CWM-DRNN][cc=%0d] drain_idx=%0d | e0=%03x e1=%03x | out0=%03x out1=%03x",
                    cycle_cnt,
                    dut.u_controller.cwm_drain_idx_r,
                    dut.u_row_accum.e0_i,
                    dut.u_row_accum.e1_i,
                    dut.u_row_accum.drain0_o,
                    dut.u_row_accum.drain1_o);
            end

            // Tail trace: last few accumulator writes (pair >= 119)
            if (dut.u_row_accum.acc_fire_i && dut.u_row_accum.pair_idx_i >= 7'd119) begin
                $display("[CWM-TAIL][cc=%0d] ACC pair=%0d cwm0=%03x cwm1=%03x",
                    cycle_cnt,
                    dut.u_row_accum.pair_idx_i,
                    dut.u_row_accum.cwm0_i,
                    dut.u_row_accum.cwm1_i);
            end

            // Tail trace: last few drain writes (drain_idx >= 124)
            if (dut.u_row_accum.drain_valid_o && dut.u_controller.cwm_drain_idx_r >= 7'd124) begin
                $display("[CWM-TAIL][cc=%0d] DRAIN idx=%0d out0=%03x out1=%03x fuse=%b e0=%03x e1=%03x",
                    cycle_cnt,
                    dut.u_controller.cwm_drain_idx_r,
                    dut.u_row_accum.drain0_o,
                    dut.u_row_accum.drain1_o,
                    dut.u_row_accum.fuse_e_i,
                    dut.u_row_accum.e0_i,
                    dut.u_row_accum.e1_i);
            end
        end else begin
            cwm_issue_cnt  <= 0;
            cwm_pe_out_cnt <= 0;
        end
    end


    // =========================================================================
    // 8. Watchdog
    // =========================================================================
    localparam int WATCHDOG_CYCLES = 10000;
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

    // Clear poly_id=0 slice (NTT/INTT in-place)
    task automatic clear_poly0;
        for (int i = 0; i < 256; i++)
            coeff_mem[0][i] = '0;
    endtask

    // Clear all poly slices
    task automatic clear_all_polys;
        for (int p = 0; p < NUM_POLYS; p++)
            for (int i = 0; i < 256; i++)
                coeff_mem[p][i] = '0;
    endtask

    // Run one PAU operation and wait for done_o
    task automatic run_pau(input pe_mode_e mode);
        @(negedge clk);
        op_type_i = mode;
        start_i   = 1;
        test_running = 1;
        @(negedge clk);
        start_i = 0;
        @(posedge clk);
        while (!done_o) @(posedge clk);
        test_running = 0;
        // Wait 3 extra cycles to allow final writeback pipeline to drain
        // (CWM drain has 2 pending beats after done_o; NTT/INTT need 1)
        repeat (3) @(posedge clk);
    endtask

    // Compare coeff_mem[poly_id] vs expected
    function automatic int compare_results(input string test_name, input int poly_id);
        int mismatches = 0;
        for (int i = 0; i < 256; i++) begin
            if (coeff_mem[poly_id][i][11:0] !== expected[i][11:0]) begin
                $display("[MISMATCH] %s coeff[%0d]: got %03x, expected %03x",
                         test_name, i, coeff_mem[poly_id][i][11:0], expected[i][11:0]);
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
        clear_poly0();
        $readmemh("verif/vectors/k2/ntt_in.mem",  coeff_mem[0]);
        $readmemh("verif/vectors/k2/ntt_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 0;
        run_pau(PE_MODE_NTT);

        mismatches = compare_results("NTT_RANDOM", 0);
        if (mismatches == 0) begin
            $display("[PASS] NTT Random: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] NTT Random: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 2: NTT Boundary (all q-1)
        // ----------------------------------------------------------------
        $display("\n=== TEST 2: NTT (Boundary: all q-1) ===");
        clear_poly0();
        $readmemh("verif/vectors/k2/ntt_max_in.mem",  coeff_mem[0]);
        $readmemh("verif/vectors/k2/ntt_max_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 0;
        run_pau(PE_MODE_NTT);

        mismatches = compare_results("NTT_MAX", 0);
        if (mismatches == 0) begin
            $display("[PASS] NTT Boundary: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] NTT Boundary: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 3: INTT Random
        // ----------------------------------------------------------------
        $display("\n=== TEST 3: INTT (Random) ===");
        clear_poly0();
        $readmemh("verif/vectors/k2/intt_in.mem",  coeff_mem[0]);
        $readmemh("verif/vectors/k2/intt_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 0;
        run_pau(PE_MODE_INTT);

        mismatches = compare_results("INTT_RANDOM", 0);
        if (mismatches == 0) begin
            $display("[PASS] INTT Random: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] INTT Random: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 4: INTT Boundary (all q-1)
        // ----------------------------------------------------------------
        $display("\n=== TEST 4: INTT (Boundary: all q-1) ===");
        clear_poly0();
        $readmemh("verif/vectors/k2/intt_max_in.mem",  coeff_mem[0]);
        $readmemh("verif/vectors/k2/intt_max_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 0;
        run_pau(PE_MODE_INTT);

        mismatches = compare_results("INTT_MAX", 0);
        if (mismatches == 0) begin
            $display("[PASS] INTT Boundary: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] INTT Boundary: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 5: CWM k=1
        //   poly_id 0 = A_0 (primary walks A terms starting at 0)
        //   poly_id 1 = s_0 (aux = secret key term)
        //   poly_id 2 = e   (primary dest / accumulator)
        // ----------------------------------------------------------------
        // CMI in CWM mode ignores poly_id_i/aux_poly_id_i and uses hardcoded
        // constants from poly_arith_pkg: POLY_ID_S0=0, POLY_ID_EI=4,
        // POLY_ID_A0=5, POLY_ID_T0=9. Load vectors into the correct slots.
        $display("\n=== TEST 5: CWM (k=1, single term) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k2/cwm_a0.mem", coeff_mem[5]);  // A_0 -> POLY_ID_A0=5
        $readmemh("verif/vectors/k2/cwm_s0.mem", coeff_mem[0]);  // s_0 -> POLY_ID_S0=0
        $readmemh("verif/vectors/k2/cwm_e.mem",  coeff_mem[4]);  // e   -> POLY_ID_EI=4
        $readmemh("verif/vectors/cwm_k1_out.mem", expected);

        primary_poly_id_i = 0;  // unused by CMI in CWM mode
        aux_poly_id_i     = 0;  // unused by CMI in CWM mode
        cwm_num_terms_i   = 1;  // k=1 term

        $display("\n--- CWM Pipeline Trace (first %0d pairs) ---", CWM_TRACE_PAIRS);
        cwm_trace_en = 1;
        run_pau(PE_MODE_CWM);
        cwm_trace_en = 0;
        $display("--- End CWM Pipeline Trace ---\n");


        mismatches = compare_results("CWM_K1", 9);  // result written to POLY_ID_T0=9
        if (mismatches == 0) begin
            $display("[PASS] CWM k=1: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM k=1: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 6: CWM k=2
        //   poly_id 0,1 = s_0, s_1
        //   poly_id 5,6 = A_0, A_1
        //   poly_id 4   = e
        // ----------------------------------------------------------------
        $display("\n=== TEST 6: CWM (k=2, two terms) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k2/cwm_a0.mem", coeff_mem[5]);
        $readmemh("verif/vectors/k2/cwm_a1.mem", coeff_mem[6]);
        $readmemh("verif/vectors/k2/cwm_s0.mem", coeff_mem[0]);
        $readmemh("verif/vectors/k2/cwm_s1.mem", coeff_mem[1]);
        $readmemh("verif/vectors/k2/cwm_e.mem",  coeff_mem[4]);
        $readmemh("verif/vectors/k2/cwm_out.mem", expected);

        primary_poly_id_i = 0;  // unused by CMI in CWM mode for A/S fetch, but output written to T0 + 0 = 9
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 2;  // k=2 terms

        $display("\n--- CWM Pipeline Trace (first %0d pairs) ---", CWM_TRACE_PAIRS);
        cwm_trace_en = 1;
        run_pau(PE_MODE_CWM);
        cwm_trace_en = 0;
        $display("--- End CWM Pipeline Trace ---\n");

        mismatches = compare_results("CWM_K2", 9);  // result written to POLY_ID_T0=9
        if (mismatches == 0) begin
            $display("[PASS] CWM k=2: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM k=2: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 7: CWM k=3
        //   poly_id 0,1,2 = s_0, s_1, s_2
        //   poly_id 5,6,7 = A_0, A_1, A_2
        //   poly_id 4     = e
        // ----------------------------------------------------------------
        $display("\n=== TEST 7: CWM (k=3, three terms) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k3/cwm_a0.mem", coeff_mem[5]);
        $readmemh("verif/vectors/k3/cwm_a1.mem", coeff_mem[6]);
        $readmemh("verif/vectors/k3/cwm_a2.mem", coeff_mem[7]);
        $readmemh("verif/vectors/k3/cwm_s0.mem", coeff_mem[0]);
        $readmemh("verif/vectors/k3/cwm_s1.mem", coeff_mem[1]);
        $readmemh("verif/vectors/k3/cwm_s2.mem", coeff_mem[2]);
        $readmemh("verif/vectors/k3/cwm_e.mem",  coeff_mem[4]);
        $readmemh("verif/vectors/k3/cwm_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 3;  // k=3 terms

        run_pau(PE_MODE_CWM);

        mismatches = compare_results("CWM_K3", 9);
        if (mismatches == 0) begin
            $display("[PASS] CWM k=3: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM k=3: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 8: CWM k=4
        //   poly_id 0,1,2,3 = s_0, s_1, s_2, s_3
        //   poly_id 5,6,7,8 = A_0, A_1, A_2, A_3
        //   poly_id 4       = e
        // ----------------------------------------------------------------
        $display("\n=== TEST 8: CWM (k=4, four terms) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k4/cwm_a0.mem", coeff_mem[5]);
        $readmemh("verif/vectors/k4/cwm_a1.mem", coeff_mem[6]);
        $readmemh("verif/vectors/k4/cwm_a2.mem", coeff_mem[7]);
        $readmemh("verif/vectors/k4/cwm_a3.mem", coeff_mem[8]);
        $readmemh("verif/vectors/k4/cwm_s0.mem", coeff_mem[0]);
        $readmemh("verif/vectors/k4/cwm_s1.mem", coeff_mem[1]);
        $readmemh("verif/vectors/k4/cwm_s2.mem", coeff_mem[2]);
        $readmemh("verif/vectors/k4/cwm_s3.mem", coeff_mem[3]);
        $readmemh("verif/vectors/k4/cwm_e.mem",  coeff_mem[4]);
        $readmemh("verif/vectors/k4/cwm_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 4;  // k=4 terms

        run_pau(PE_MODE_CWM);

        mismatches = compare_results("CWM_K4", 9);
        if (mismatches == 0) begin
            $display("[PASS] CWM k=4: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM k=4: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 9: CWM Boundary (Max) k=2
        //   poly_id 0,1 = s_0, s_1 (boundary)
        //   poly_id 5,6 = A_0, A_1 (boundary)
        //   poly_id 4   = e (boundary)
        // ----------------------------------------------------------------
        $display("\n=== TEST 9: CWM Boundary (Max/Min mix) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k2/cwm_max_a0.mem", coeff_mem[5]);
        $readmemh("verif/vectors/k2/cwm_max_a1.mem", coeff_mem[6]);
        $readmemh("verif/vectors/k2/cwm_max_s0.mem", coeff_mem[0]);
        $readmemh("verif/vectors/k2/cwm_max_s1.mem", coeff_mem[1]);
        $readmemh("verif/vectors/k2/cwm_max_e.mem",  coeff_mem[4]);
        $readmemh("verif/vectors/k2/cwm_max_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 2;  // k=2 terms for boundary testing

        run_pau(PE_MODE_CWM);

        mismatches = compare_results("CWM_MAX_K2", 9);
        if (mismatches == 0) begin
            $display("[PASS] CWM Boundary: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM Boundary: %0d coefficient mismatches.", mismatches);
            total_fail++;
        end

        repeat (10) @(posedge clk);

        // ----------------------------------------------------------------
        // Test 10: CWM Zero Noise (e = 0) k=2
        //   poly_id 0,1 = s_0, s_1 (random)
        //   poly_id 5,6 = A_0, A_1 (random)
        //   poly_id 4   = e (all 0)
        // ----------------------------------------------------------------
        $display("\n=== TEST 10: CWM Zero Noise (e = 0) ===");
        clear_all_polys();
        $readmemh("verif/vectors/k2/cwm_zero_a0.mem", coeff_mem[5]);
        $readmemh("verif/vectors/k2/cwm_zero_a1.mem", coeff_mem[6]);
        $readmemh("verif/vectors/k2/cwm_zero_s0.mem", coeff_mem[0]);
        $readmemh("verif/vectors/k2/cwm_zero_s1.mem", coeff_mem[1]);
        $readmemh("verif/vectors/k2/cwm_zero_e.mem",  coeff_mem[4]);
        $readmemh("verif/vectors/k2/cwm_zero_out.mem", expected);

        primary_poly_id_i = 0;
        aux_poly_id_i     = 0;
        cwm_num_terms_i   = 2;  // k=2 terms

        run_pau(PE_MODE_CWM);

        mismatches = compare_results("CWM_ZERO_NOISE_K2", 9);
        if (mismatches == 0) begin
            $display("[PASS] CWM Zero Noise: all 256 coefficients match.");
            total_pass++;
        end else begin
            $display("[FAIL] CWM Zero Noise: %0d coefficient mismatches.", mismatches);
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
