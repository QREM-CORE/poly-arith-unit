/*
 * Module Name: mac_adder (Multiply-Accumulate Adder)
 * Author(s): Salwan Aldhahab
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * This module implements the Multiply-Accumulate (MAC) adder for the PAU
 * architecture. It enables the seamless transition from Coordinate-Wise
 * Multiplication (CWM) mode to MAC mode, which is essential for matrix-vector
 * multiplications in ML-KEM (e.g., t_hat = A_hat ∘ s_hat + e in NTT domain).
 *
 * The MAC adder sits at the output of the Arithmetic Unit (AU) and performs
 * modular accumulation of successive CWM results. For each element of the
 * output vector in ML-KEM matrix-vector multiplication:
 *
 *   result_hat[i] = sum_{j=0}^{k-1} ( A_hat[i][j] ∘ s_hat[j] )
 *
 * The first CWM result (j=0) is passed through directly, and subsequent
 * results (j=1..k-1) are accumulated via modular addition (mod q = 3329)
 * with the partial sum read back from Polynomial Memory (PM).
 *
 * Architecture:
 * Two parallel modular adders (mod_add instances) process the two coefficient
 * lanes produced by CWM mode (2 coefficients per clock cycle, 128 CCs for a
 * full 256-coefficient polynomial). A bypass multiplexer controlled by `init_i`
 * selects between:
 *   - init_i = 1: Passthrough (stores first CWM result as initial accumulation)
 *   - init_i = 0: Accumulate (adds new CWM result to existing partial sum)
 *
 * Data Flow:
 *   PM[read] --> CMI --> AU (CWM) --+--> MAC Adder --> CMI --> PM[write]
 *                                   |       ^
 *                                   |       |
 *                  PM[acc_read] ----+-------+
 *                  (previous partial sum)
 *
 * Latency: 1 Clock Cycle (Registered output)
 * Throughput: 2 coefficients per clock cycle (matching CWM output rate)
 * Total MAC operation: 128 CCs for a full 256-coefficient polynomial
 */

import poly_arith_pkg::*;

module mac_adder (
    input   logic           clk,
    input   logic           rst,

    // Control
    input   logic           init_i,     // 1: Passthrough (first CWM iteration)
                                        // 0: Accumulate  (subsequent iterations)
    input   logic           valid_i,    // Input data valid

    // Lane 0: Even-index coefficient (c_2i)
    input   coeff_t         a0_i,       // New CWM result from AU
    input   coeff_t         b0_i,       // Previous partial sum from PM

    // Lane 1: Odd-index coefficient (c_2i+1)
    input   coeff_t         a1_i,       // New CWM result from AU
    input   coeff_t         b1_i,       // Previous partial sum from PM

    // Accumulated Output (to PM write port)
    output  coeff_t         z0_o,       // Accumulated result lane 0
    output  coeff_t         z1_o,       // Accumulated result lane 1

    output  logic           valid_o     // Output valid
);

    // =========================================================================
    // Internal Wires
    // =========================================================================
    coeff_t mod_add_0_result;
    coeff_t mod_add_1_result;
    coeff_t mac_result_0;
    coeff_t mac_result_1;

    // =========================================================================
    // Modular Adder Instantiations
    // =========================================================================

    // -------- Lane 0 Modular Adder --------
    // Computes: (new_coeff_0 + old_acc_0) mod 3329
    mod_add u_mod_add_0 (
        .op1_i      (a0_i),
        .op2_i      (b0_i),

        .result_o   (mod_add_0_result)
    );

    // -------- Lane 1 Modular Adder --------
    // Computes: (new_coeff_1 + old_acc_1) mod 3329
    mod_add u_mod_add_1 (
        .op1_i      (a1_i),
        .op2_i      (b1_i),

        .result_o   (mod_add_1_result)
    );

    // =========================================================================
    // Init / Accumulate Bypass MUX
    // =========================================================================
    // When init_i = 1 (first CWM in accumulation sequence):
    //   Output = new CWM result (passthrough, no addition needed)
    // When init_i = 0 (subsequent CWMs):
    //   Output = (new CWM result + old partial sum) mod q
    assign mac_result_0 = init_i ? a0_i : mod_add_0_result;
    assign mac_result_1 = init_i ? a1_i : mod_add_1_result;

    // =========================================================================
    // Output Registration (1 CC Latency)
    // =========================================================================
    // Registered output breaks the combinational path from AU to PM write,
    // ensuring clean timing closure across the datapath.
    always_ff @(posedge clk) begin
        if (rst) begin
            z0_o    <= '0;
            z1_o    <= '0;
            valid_o <= 1'b0;
        end else begin
            z0_o    <= mac_result_0;
            z1_o    <= mac_result_1;
            valid_o <= valid_i;
        end
    end

endmodule

/* ------- Connection Digram For Future Reference -------
*                    ┌──────────────────────────────────┐
*                    │         Arithmetic Unit          │
*                    │                                  │
*  operand_a0 ──►    │ PE in CWM mode                   │──► cwm_result_0 ──► a0_i ┐
*  operand_a1 ──►    │ (2 coefficients/CC)              │──► cwm_result_1 ──► a1_i ┤
*  operand_b0 ──►    │                                  │                          │
*  operand_b1 ──►    │                                  │                   ┌──────▼─────┐
*                    └──────────────────────────────────┘                   │ mac_adder  │
*                                                                           │            │
*  PM[acc_addr] ──► partial_sum_0 ──────────────────────────────────► b0_i  │            │──► z0_o ──► PM[write]
*  PM[acc_addr] ──► partial_sum_1 ──────────────────────────────────► b1_i  │            │──► z1_o ──► PM[write]
*                                                                           │            │
*  controller ──► init_i (j==0 ? 1 : 0)                                     │            │
*  controller ──► valid_i                                                   │            │
*                                                                           └────────────┘
*/
