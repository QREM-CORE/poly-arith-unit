/*
 * Module Name: pe_unit
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Top-level wrapper for the Processing Elements.
 * Includes pipeline synchronization delays for cascaded Twiddle Factors
 * and Output Latency alignment.
 *
 * ==============================================================================
 * PE_UNIT USER GUIDE (I/O MAPPING)
 * ==============================================================================
 * This module dynamically reconfigures its 8-port input interface based on the
 * active ctrl_i state. The top-level Memory Controller/AGU MUST drive the op_a
 * and op_b buses according to the tables below.
 * 1. CWM (Coordinate-Wise Multiplication) -> mode_i = Don't Care
 * op_a = [f_2i, f_2i+1, g_2i, g_2i+1]
 * op_b = [omega, d/c, d/c, d/c]
 * Outputs: z1_o = U3, z2_o = V0 (z0/z3 are unused)
 * 2. NTT (Number Theoretic Transform) -> mode_i = 0 (Radix-4)
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [w_2, w_1, w_3, w_4^1]
 * Outputs: z0_o = U1, z1_o = V1, z2_o = U3, z3_o = V3
 * 3. NTT (Number Theoretic Transform) -> mode_i = 1 (Radix-2)
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [w_A, 12'd1, w_B, d/c] -> **NOTE: op_b1_i MUST be exactly 1**
 * Outputs: z0_o = U0, z1_o = V0, z2_o = U2, z3_o = V2
 * 4. INTT (Inverse NTT) -> mode_i = 0 (Radix-4)
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [w_2^-1, w_1^-1, w_3^-1, w_4^-1]
 * Outputs: z0_o = U0, z1_o = U2, z2_o = V0, z3_o = V2
 * 5. INTT (Inverse NTT) -> mode_i = 1 (Radix-2)
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [w_A^-1, 12'd1665, w_B^-1, d/c] -> **NOTE: op_b1_i MUST be 1665 (2^-1)**
 * Outputs: z0_o = U0, z1_o = V0, z2_o = U2, z3_o = V2
 * 6. ADDSUB (Point-wise Addition/Subtraction) -> mode_i = 0 (Add), 1 (Sub)
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [Y_0, Y_1, Y_2, Y_3]
 * Outputs: Z maps directly to the requested U (Add) or V (Sub) results.
 * 7. COMP / DECOMP -> mode_i = Don't Care
 * op_a = [X_0, X_1, X_2, X_3]
 * op_b = [m/q, m/q, m/q, m/q]
 * Outputs: z0_o = V0, z1_o = U2, z2_o = V2, z3_o = V3
 * ==============================================================================
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module pe_unit (
    input   logic           clk,
    input   logic           rst,

    input   logic           valid_i,
    input   pe_mode_e       ctrl_i,

    // 0 = Add (U), 1 = Sub (V) for ADD/SUB ctrl
    // 0 = Radix 4, 1 = Radix 2 for NTT/INTT
    input   logic           mode_i,

    // ==========================================
    // Primary Operand Bus (Fed by SRAM Bank A)
    // ==========================================
    // Mappings:
    // - NTT/INTT/COMP/DECOMP : X_0, X_1, X_2, X_3
    // - ADDSUB               : X_0, X_1, X_2, X_3
    // - CWM                  : f_2i, f_2i+1, g_2i, g_2i+1
    input   coeff_t         op_a0_i,
    input   coeff_t         op_a1_i,
    input   coeff_t         op_a2_i,
    input   coeff_t         op_a3_i,

    // ==========================================
    // Secondary Operand Bus (Fed by SRAM Bank B OR Twiddle ROM)
    // ==========================================
    // Mappings:
    // - NTT/INTT/COMP/DECOMP : w_2, w_1, w_3, w_4 (Twiddles / Constants)
    // - ADDSUB               : Y_0, Y_1, Y_2, Y_3 (Secondary Polynomial)
    // - CWM                  : omega, Unused, Unused, Unused
    input   coeff_t         op_b0_i,
    input   coeff_t         op_b1_i,
    input   coeff_t         op_b2_i,
    input   coeff_t         op_b3_i,

    // ==========================================
    // Outputs
    // ==========================================
    output  coeff_t         z0_o,
    output  coeff_t         z1_o,
    output  coeff_t         z2_o,
    output  coeff_t         z3_o,

    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // -------- PE0 Wires --------
    coeff_t     pe0_a0_i, pe0_b0_i, pe0_w0_i;
    coeff_t     pe0_u0_o, pe0_v0_o;
    pe_mode_e   pe0_ctrl_i;
    logic       pe0_valid_i, pe0_valid_o;

    // -------- PE1 Wires --------
    coeff_t     pe1_a1_i, pe1_b1_i, pe1_c0_i, pe1_c1_i;
    coeff_t     pe1_u1_o, pe1_v1_o;
    pe_mode_e   pe1_ctrl_i;
    logic       pe1_valid_i, pe1_valid_o;

    // -------- PE2 Wires --------
    coeff_t     pe2_a2_i, pe2_b2_i, pe2_w1_i, pe2_w2_i;
    coeff_t     pe2_u2_o, pe2_v2_o, pe2_m_o;
    pe_mode_e   pe2_ctrl_i;
    logic       pe2_valid_i, pe2_valid_o, pe2_valid_m_o;

    // -------- PE3 Wires --------
    coeff_t     pe3_a3_i, pe3_b3_i, pe3_w3_i, pe3_tf_omega_4_i;
    coeff_t     pe3_u3_o, pe3_v3_o;
    pe_mode_e   pe3_ctrl_i;
    logic       pe3_valid_i, pe3_valid_o;

    // =========================================================================
    // Pipeline Synchronization Delays
    // =========================================================================

    // -------- Twiddle Factor Input Delays --------
    coeff_t op_b0_d3, op_b0_d4;
    coeff_t op_b1_d4;
    coeff_t op_b2_d4;
    coeff_t op_b3_d4;

    // op_b0 delays (Needs 3-cycle tap for CWM, 4-cycle tap for INTT)
    delay_n #(.DWIDTH(12), .DEPTH(3)) u_delay_b0_3 (
        .clk(clk), .rst(rst), .data_i(op_b0_i), .data_o(op_b0_d3)
    );
    delay_n #(.DWIDTH(12), .DEPTH(1)) u_delay_b0_4 (
        .clk(clk), .rst(rst), .data_i(op_b0_d3), .data_o(op_b0_d4) // Chained
    );

    // op_b1 delay (4-cycle for INTT)
    delay_n #(.DWIDTH(12), .DEPTH(4)) u_delay_b1_4 (
        .clk(clk), .rst(rst), .data_i(op_b1_i), .data_o(op_b1_d4)
    );

    // op_b2 delay (4-cycle for INTT)
    delay_n #(.DWIDTH(12), .DEPTH(4)) u_delay_b2_4 (
        .clk(clk), .rst(rst), .data_i(op_b2_i), .data_o(op_b2_d4)
    );

    // op_b3 delay (4-cycle for NTT)
    delay_n #(.DWIDTH(12), .DEPTH(4)) u_delay_b3_4 (
        .clk(clk), .rst(rst), .data_i(op_b3_i), .data_o(op_b3_d4)
    );

    // -------- CWM Output Alignment Delays --------
    // In CWM, the PE3 path finishes 1 cycle earlier than the PE0 path.
    // These delays align the PE3 output to match the 8-cycle latency.
    coeff_t pe3_u3_o_d1;
    logic   pe3_valid_o_d1;

    delay_n #(.DWIDTH(12), .DEPTH(1)) u_delay_pe3_u3_out (
        .clk(clk), .rst(rst), .data_i(pe3_u3_o), .data_o(pe3_u3_o_d1)
    );

    delay_n #(.DWIDTH(1), .DEPTH(1)) u_delay_pe3_valid_out (
        .clk(clk), .rst(rst), .data_i(pe3_valid_o), .data_o(pe3_valid_o_d1)
    );

    // =========================================================================
    // Processing Element (PE) Instantiations
    // =========================================================================

    // -------- PE0 --------
    pe0 u_pe0 (
        .clk        (clk),
        .rst        (rst),
        .a0_i       (pe0_a0_i),
        .b0_i       (pe0_b0_i),
        .w0_i       (pe0_w0_i),
        .ctrl_i     (pe0_ctrl_i),
        .valid_i    (pe0_valid_i),
        .u0_o       (pe0_u0_o),
        .v0_o       (pe0_v0_o),
        .valid_o    (pe0_valid_o)
    );

    // -------- PE1 --------
    pe1 u_pe1 (
        .clk        (clk),
        .rst        (rst),
        .a1_i       (pe1_a1_i),
        .b1_i       (pe1_b1_i),
        .c0_i       (pe1_c0_i),
        .c1_i       (pe1_c1_i),
        .ctrl_i     (pe1_ctrl_i),
        .valid_i    (pe1_valid_i),
        .u1_o       (pe1_u1_o),
        .v1_o       (pe1_v1_o),
        .valid_o    (pe1_valid_o)
    );

    // -------- PE2 --------
    pe2 u_pe2 (
        .clk        (clk),
        .rst        (rst),
        .a2_i       (pe2_a2_i),
        .b2_i       (pe2_b2_i),
        .w1_i       (pe2_w1_i),
        .w2_i       (pe2_w2_i),
        .ctrl_i     (pe2_ctrl_i),
        .valid_i    (pe2_valid_i),
        .u2_o       (pe2_u2_o),
        .v2_o       (pe2_v2_o),
        .valid_o    (pe2_valid_o),
        .m_o        (pe2_m_o),
        .valid_m_o  (pe2_valid_m_o)
    );

    // -------- PE3 --------
    pe3 u_pe3 (
        .clk            (clk),
        .rst            (rst),
        .a3_i           (pe3_a3_i),
        .b3_i           (pe3_b3_i),
        .w3_i           (pe3_w3_i),
        .tf_omega_4_i   (pe3_tf_omega_4_i),
        .ctrl_i         (pe3_ctrl_i),
        .valid_i        (pe3_valid_i),
        .u3_o           (pe3_u3_o),
        .v3_o           (pe3_v3_o),
        .valid_o        (pe3_valid_o)
    );

    // =========================================================================
    // Processing Element (PE) Routing (Optimized 8-Port Interface)
    // Reference: Table 1 (Inha University UniPAM Architecture)
    // =========================================================================

    always_comb begin
        // Default assignments to prevent inferred latches
        pe0_a0_i = '0; pe0_b0_i = '0; pe0_w0_i = '0;
        pe1_a1_i = '0; pe1_b1_i = '0; pe1_c0_i = '0; pe1_c1_i = '0;
        pe2_a2_i = '0; pe2_b2_i = '0; pe2_w1_i = '0; pe2_w2_i = '0;
        pe3_a3_i = '0; pe3_b3_i = '0; pe3_w3_i = '0; pe3_tf_omega_4_i = '0;
        z0_o = '0; z1_o = '0; z2_o = '0; z3_o = '0;

        // Default Control propagation
        pe0_ctrl_i = ctrl_i; pe1_ctrl_i = ctrl_i;
        pe2_ctrl_i = ctrl_i; pe3_ctrl_i = ctrl_i;

        // Default Valid propagation to prevent latches
        pe0_valid_i = 1'b0; pe1_valid_i = 1'b0;
        pe2_valid_i = 1'b0; pe3_valid_i = 1'b0;
        valid_o = 1'b0;

        case(ctrl_i)
            PE_MODE_IDLE: begin
                // Idle state, do nothing
            end
            PE_MODE_CWM : begin
                // ---------------------------------------------------------
                // External Mapping for CWM (5 Inputs Used):
                // op_a0_i = f_2i        op_b0_i = omega (w)
                // op_a1_i = f_2i+1      op_b1_i = Unused
                // op_a2_i = g_2i        op_b2_i = Unused
                // op_a3_i = g_2i+1      op_b3_i = Unused
                // ---------------------------------------------------------

                // STAGE 1: PE1 and PE2 receive fresh inputs
                pe1_valid_i = valid_i;
                pe2_valid_i = valid_i;

                // STAGE 2: PE0 and PE3 receive valid cascades
                pe0_valid_i = pe1_valid_o & pe2_valid_m_o; // M matches U1/V1
                pe3_valid_i = pe2_valid_o;                 // Driven by U2/V2

                // CE0 Routing (Feedback Heavy)
                pe0_a0_i = pe2_m_o;   // M
                pe0_b0_i = pe1_u1_o;  // U1
                pe0_w0_i = pe1_v1_o;  // V1

                // CE1 Routing
                pe1_a1_i = op_a2_i;   // g_2i
                pe1_b1_i = op_a1_i;   // f_2i+1
                // NOTE: Swapped c0 and c1 inputs to fix theoretical CWM routing
                pe1_c0_i = op_a0_i;   // f_2i
                pe1_c1_i = op_a3_i;   // g_2i+1

                // CE2 Routing
                pe2_a2_i = op_a0_i;   // f_2i
                pe2_b2_i = op_a3_i;   // g_2i+1
                pe2_w1_i = op_a2_i;   // g_2i
                pe2_w2_i = op_a1_i;   // f_2i+1

                // CE3 Routing
                pe3_a3_i = pe2_u2_o;  // U2
                pe3_b3_i = pe2_v2_o;  // V2
                pe3_w3_i = op_b0_d3;  // omega (SYNCHRONIZED: 3-Cycle Delay)

                // Outputs
                z1_o = pe3_u3_o_d1;   // U3 (SYNCHRONIZED: 1-Cycle Delay to match PE0)
                z2_o = pe0_v0_o;      // V0

                if (pe0_valid_o) begin
                    $display("PE_UNIT_CWM: cc=%0t pe0_v0_o=%x pe3_u3_o_d1=%x pe0_a0_i=%x pe0_b0_i=%x pe0_w0_i=%x", $time, pe0_v0_o, pe3_u3_o_d1, pe0_a0_i, pe0_b0_i, pe0_w0_i);
                end
                valid_o = pe0_valid_o & pe3_valid_o_d1; // Perfectly aligned to 8 CCs
            end

            PE_MODE_NTT : begin
                if (mode_i == 1'b0) begin
                    // =========================================================
                    // RADIX-4 NTT MODE (Cascaded PEs)
                    // =========================================================

                    // ---------------------------------------------------------
                    // External Mapping for RADIX-4 NTT (8 Inputs Used):
                    // op_a0_i = X_0         op_b0_i = w_2
                    // op_a1_i = X_1         op_b1_i = w_1
                    // op_a2_i = X_2         op_b2_i = w_3
                    // op_a3_i = X_3         op_b3_i = w_4^1 (or single omega)
                    // ---------------------------------------------------------

                    // STAGE 1: PE0 and PE2 receive fresh inputs
                    pe0_valid_i = valid_i;
                    pe2_valid_i = valid_i;

                    // STAGE 2: PE1 and PE3 receive valid cascades
                    pe1_valid_i = pe0_valid_o & pe2_valid_o;
                    pe3_valid_i = pe0_valid_o & pe2_valid_o;

                    // CE0 Routing
                    pe0_a0_i = op_a0_i;   // X_0
                    pe0_b0_i = op_a2_i;   // X_2
                    pe0_w0_i = op_b0_i;   // w_2 (No delay, Stage 1)

                    // CE2 Routing
                    pe2_a2_i = op_a1_i;   // X_1
                    pe2_b2_i = op_a3_i;   // X_3
                    pe2_w1_i = op_b1_i;   // w_1 (No delay, Stage 1)
                    pe2_w2_i = op_b2_i;   // w_3 (No delay, Stage 1)
                end else begin
                    // =========================================================
                    // RADIX-2 NTT MODE (Bypasses PE1/PE3 completely)
                    // Uses PE0 and PE2 in parallel for two separate butterflies.
                    // =========================================================

                    // ---------------------------------------------------------
                    // External Mapping for RADIX-2 NTT (7 Inputs Used):
                    // op_a0_i = X_0         op_b0_i = omega_A (PE0 Twiddle)
                    // op_a1_i = X_1         op_b1_i = 12'd1   (PE2 Bypass Multiplier)
                    // op_a2_i = X_2         op_b2_i = omega_B (PE2 Twiddle)
                    // op_a3_i = X_3         op_b3_i = Unused
                    //
                    // CRITICAL NOTE: op_b1_i MUST be driven to 1. PE2 computes
                    // A*W1 + B*W2. Setting W1 to 1 turns it into a standard butterfly.
                    // ---------------------------------------------------------

                    pe0_valid_i = valid_i;
                    pe2_valid_i = valid_i;

                    // PE0 Routing (Interleaved read lanes: X0, X2 mapped to op_a0, op_a1)
                    pe0_a0_i = op_a0_i;   // X0
                    pe0_b0_i = op_a1_i;   // X2
                    pe0_w0_i = op_b0_i;

                    // PE2 Routing (Interleaved read lanes: X1, X3 mapped to op_a2, op_a3)
                    pe2_a2_i = op_a2_i;   // X1
                    pe2_b2_i = op_a3_i;   // X3
                    pe2_w1_i = op_b1_i;   // Driven to 1
                    pe2_w2_i = op_b2_i;
                end

                // Unconditional assignments for PE1/PE3 to cut mode_i MUXes
                pe1_a1_i = pe0_u0_o;
                pe1_b1_i = pe2_u2_o;
                pe3_a3_i = pe0_v0_o;
                pe3_b3_i = pe2_v2_o;
                pe3_tf_omega_4_i = op_b3_d4;

                if (mode_i == 1'b0) begin
                    // Outputs Radix-4
                    z0_o = pe1_u1_o;      // U1
                    z1_o = pe1_v1_o;      // V1
                    z2_o = pe3_u3_o;      // U3
                    z3_o = pe3_v3_o;      // V3
                    valid_o = pe1_valid_o & pe3_valid_o;
                end else begin
                    // Direct Outputs Radix-2
                    z0_o = pe0_u0_o;
                    z1_o = pe0_v0_o;
                    z2_o = pe2_u2_o;
                    z3_o = pe2_v2_o;
                    valid_o = pe0_valid_o & pe2_valid_o;
                end
            end

            PE_MODE_INTT : begin
                if (mode_i == 1'b0) begin
                    // =========================================================
                    // RADIX-4 INTT MODE (Cascaded PEs)
                    // =========================================================

                    // ---------------------------------------------------------
                    // External Mapping for RADIX-4 INTT (8 Inputs Used):
                    // op_a0_i = X_0         op_b0_i = w_2^-1
                    // op_a1_i = X_1         op_b1_i = w_1^-1
                    // op_a2_i = X_2         op_b2_i = w_3^-1
                    // op_a3_i = X_3         op_b3_i = w_4^-1 (or single inv)
                    // ---------------------------------------------------------

                    // STAGE 1: PE1 and PE3 receive fresh inputs
                    pe1_valid_i = valid_i;
                    pe3_valid_i = valid_i;

                    // STAGE 2: PE0 and PE2 receive valid cascades
                    pe0_valid_i = pe1_valid_o & pe3_valid_o;
                    pe2_valid_i = pe1_valid_o & pe3_valid_o;

                    // CE0 Routing (Cross-PE Feedback)
                    pe0_a0_i = pe3_u3_o;  // U3
                    pe0_b0_i = pe1_u1_o;  // U1
                    pe0_w0_i = op_b0_d4;  // w_2^-1 (SYNCHRONIZED: 4-Cycle Delay)

                    // CE2 Routing (Cross-PE Feedback)
                    pe2_a2_i = pe3_v3_o;  // V3
                    pe2_b2_i = pe1_v1_o;  // V1
                    pe2_w1_i = op_b1_d4;  // w_1^-1 (SYNCHRONIZED: 4-Cycle Delay)
                    pe2_w2_i = op_b2_d4;  // w_3^-1 (SYNCHRONIZED: 4-Cycle Delay)
                end else begin
                    // =========================================================
                    // RADIX-2 INTT MODE (Bypasses PE1/PE3 completely)
                    // Uses PE0 and PE2 in parallel for two separate butterflies.
                    // =========================================================

                    // ---------------------------------------------------------
                    // External Mapping for RADIX-2 INTT (7 Inputs Used):
                    // op_a0_i = X_0         op_b0_i = omega_A^-1 (PE0 Twiddle)
                    // op_a1_i = X_1         op_b1_i = 12'd1665   (PE2 Div-by-2 Constant)
                    // op_a2_i = X_2         op_b2_i = omega_B^-1 (PE2 Twiddle)
                    // op_a3_i = X_3         op_b3_i = Unused
                    //
                    // CRITICAL NOTE: op_b1_i MUST be driven to 1665 (which is the
                    // modulo 3329 inverse of 2). PE2 computes (A+B)*W1. Setting
                    // W1 to 1665 performs the required (A+B)/2 division for INTT.
                    // ---------------------------------------------------------

                    pe0_valid_i = valid_i;
                    pe2_valid_i = valid_i;

                    // PE0 Routing (Adjacent Elements: X0, X1)
                    pe0_a0_i = op_a0_i;   // X0
                    pe0_b0_i = op_a1_i;   // X1
                    pe0_w0_i = op_b0_i;

                    // PE2 Routing (Adjacent Elements: X2, X3)
                    pe2_a2_i = op_a2_i;   // X2
                    pe2_b2_i = op_a3_i;   // X3
                    pe2_w1_i = op_b1_i;   // Driven to 2^-1
                    pe2_w2_i = op_b2_i;
                end

                // Unconditional assignments for PE1/PE3 to cut mode_i MUXes
                pe1_a1_i = op_a2_i;
                pe1_b1_i = op_a3_i;
                pe3_a3_i = op_a0_i;
                pe3_b3_i = op_a1_i;
                pe3_tf_omega_4_i = op_b3_i;

                if (mode_i == 1'b0) begin
                    // Outputs Radix-4
                    z0_o = pe0_u0_o;      // U0
                    z1_o = pe2_u2_o;      // U2
                    z2_o = pe0_v0_o;      // V0
                    z3_o = pe2_v2_o;      // V2
                    valid_o = pe0_valid_o & pe2_valid_o;
                end else begin
                    // Direct Outputs Radix-2
                    z0_o = pe0_u0_o;
                    z1_o = pe0_v0_o;
                    z2_o = pe2_u2_o;
                    z3_o = pe2_v2_o;
                    valid_o = pe0_valid_o & pe2_valid_o;
                end
            end

            PE_MODE_ADDSUB : begin
                // ---------------------------------------------------------
                // External Mapping for Point-wise ADD/SUB (8 Inputs Used):
                // op_a0_i = X_0         op_b0_i = Y_0
                // op_a1_i = X_1         op_b1_i = Y_1
                // op_a2_i = X_2         op_b2_i = Y_2
                // op_a3_i = X_3         op_b3_i = Y_3
                // ---------------------------------------------------------

                // ALL PEs run in parallel (Stage 1)
                pe0_valid_i = valid_i;
                pe1_valid_i = valid_i;
                pe2_valid_i = valid_i;
                pe3_valid_i = valid_i;

                // CE0 Routing
                pe0_a0_i = op_a0_i;
                pe0_b0_i = op_b0_i;

                // Note: Swapped CE1 and CE2 inputs for +/- from Table 1 Inha
                // CE1 Routing (Straight 1-to-1 Mapping)
                pe1_a1_i = op_a1_i;
                pe1_b1_i = op_b1_i;

                // CE2 Routing (Straight 1-to-1 Mapping)
                pe2_a2_i = op_a2_i;
                pe2_b2_i = op_b2_i;

                // CE3 Routing (Straight 1-to-1 Mapping)
                pe3_a3_i = op_a3_i;
                pe3_b3_i = op_b3_i;

                // Outputs - Use the mode_i flag to select Add (U) or Sub (V)
                if (mode_i == 1'b0) begin
                    // Addition Mode
                    z0_o = pe0_u0_o;
                    z1_o = pe1_u1_o;
                    z2_o = pe2_u2_o;
                    z3_o = pe3_u3_o;
                end else begin
                    // Subtraction Mode
                    z0_o = pe0_v0_o;
                    z1_o = pe1_v1_o;
                    z2_o = pe2_v2_o;
                    z3_o = pe3_v3_o;
                end
                valid_o = pe0_valid_o & pe1_valid_o & pe2_valid_o & pe3_valid_o;
            end

            PE_MODE_COMP, PE_MODE_DECOMP : begin
                // ---------------------------------------------------------
                // External Mapping for COMP / DECOMP (8 Inputs Used):
                // op_a0_i = X_0         op_b0_i = m/q
                // op_a1_i = X_1         op_b1_i = m/q
                // op_a2_i = X_2         op_b2_i = m/q
                // op_a3_i = X_3         op_b3_i = m/q
                // ---------------------------------------------------------

                // PE0, PE2, PE3 run in parallel. PE1 is unused.
                pe0_valid_i = valid_i;
                pe2_valid_i = valid_i;
                pe3_valid_i = valid_i;

                // CE0 Routing
                pe0_b0_i = op_a0_i;   // X_0
                pe0_w0_i = op_b0_i;   // m/q

                // CE2 Routing
                pe2_a2_i = op_a1_i;   // X_1
                pe2_b2_i = op_a2_i;   // X_2
                pe2_w1_i = op_b1_i;   // m/q
                pe2_w2_i = op_b2_i;   // m/q

                // CE3 Routing
                pe3_b3_i = op_a3_i;   // X_3
                pe3_w3_i = op_b3_i;   // m/q

                // Outputs
                z0_o = pe0_v0_o;      // V0
                z1_o = pe2_u2_o;      // U2
                z2_o = pe2_v2_o;      // V2
                z3_o = pe3_v3_o;      // V3
                valid_o = pe0_valid_o & pe2_valid_o & pe3_valid_o;
            end

            default : begin
                // synthesis translate_off
                if (!$isunknown(ctrl_i)) begin // Use isunknown condition to avoid startup error
                    $error("[AU Wrapper] ERROR: Invalid pe_mode_e state received: %b", ctrl_i);
                end
                // synthesis translate_on
            end
        endcase
    end
endmodule
