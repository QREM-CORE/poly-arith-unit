/*
 * Module Name: mod_mul
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Modular Multiplication: (A * B) mod 3329.
 * Implements the "Table-Based Modular Reduction" algorithm described by
 * Jung et al. (2025). This architecture replaces expensive division with
 * parallel Look-Up Tables (LUTs).
 *
 * Latency: 3 Clock Cycles (Required for Butterfly Unit sync)
 * - Cycle 0: Input Registration
 * - Cycle 1: Multiplication & Stage 1 Reduction (LUT Lookup)
 * - Cycle 2: Stage 2 Reduction (4-Mux & Final Correction) & Output Registration
 */

import poly_arith_pkg::*;
import qrem_global_pkg::*;

module mod_mul (
    input   logic   clk,
    input   logic   rst,

    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,
    input   logic   valid_i,

    // Output: 12-bit result (0 to 3328)
    output  coeff_t result_o,
    output  logic   valid_o
);

    // =========================================================================
    // CYCLE 0: Input Registration (Pipeline Stage 1)
    // =========================================================================
    coeff_t op1_reg, op2_reg;
    logic   valid_reg1;

    always_ff @(posedge clk) begin
        if (rst) begin
            op1_reg    <= '0;
            op2_reg    <= '0;
            valid_reg1 <= 1'b0;
        end else begin
            op1_reg    <= op1_i;
            op2_reg    <= op2_i;
            valid_reg1 <= valid_i;
        end
    end

    // =========================================================================
    // CYCLE 1 Logic: Multiplication & Bit Slicing
    // =========================================================================

    // 1. Raw Multiplication (Using Registered Inputs)
    // Max value: 3328 * 3328 = 11,075,584 (requires 24 bits)
    logic [23:0] mul_result;
    assign mul_result = op1_reg * op2_reg;

    // 2. Bit Slicing (Radix-16 Decomposition)
    // Logic: Product = (C_23_20 * 2^20) + (C_19_16 * 2^16) + (C_15_12 * 2^12) + C_11_0
    logic [3:0]  chunk_23_20, chunk_19_16, chunk_15_12;
    logic [11:0] chunk_11_00;

    assign chunk_23_20 = mul_result[23:20];
    assign chunk_19_16 = mul_result[19:16];
    assign chunk_15_12 = mul_result[15:12];
    assign chunk_11_00 = mul_result[11:0];

    // 3. First Reduction (Parallel LUTs)
    // LUT Outputs: These hold (Chunk_Value * 2^Shift) mod 3329
    logic [11:0] lut_val_15_12, lut_val_19_16;

    always_comb begin
        // LUT [15:12] (Weight 2^12 mod Q = 767)
        case(chunk_15_12)
            4'd0:    lut_val_15_12 = 12'd0;    4'd1:    lut_val_15_12 = 12'd767;
            4'd2:    lut_val_15_12 = 12'd1534; 4'd3:    lut_val_15_12 = 12'd2301;
            4'd4:    lut_val_15_12 = 12'd3068; 4'd5:    lut_val_15_12 = 12'd506;
            4'd6:    lut_val_15_12 = 12'd1273; 4'd7:    lut_val_15_12 = 12'd2040;
            4'd8:    lut_val_15_12 = 12'd2807; 4'd9:    lut_val_15_12 = 12'd245;
            4'd10:   lut_val_15_12 = 12'd1012; 4'd11:   lut_val_15_12 = 12'd1779;
            4'd12:   lut_val_15_12 = 12'd2546; 4'd13:   lut_val_15_12 = 12'd3313;
            4'd14:   lut_val_15_12 = 12'd751;  4'd15:   lut_val_15_12 = 12'd1518;
            default: lut_val_15_12 = '0;
        endcase

        // LUT [19:16] (Weight 2^16 mod Q = 2285)
        case(chunk_19_16)
            4'd0:    lut_val_19_16 = 12'd0;    4'd1:    lut_val_19_16 = 12'd2285;
            4'd2:    lut_val_19_16 = 12'd1241; 4'd3:    lut_val_19_16 = 12'd197;
            4'd4:    lut_val_19_16 = 12'd2482; 4'd5:    lut_val_19_16 = 12'd1438;
            4'd6:    lut_val_19_16 = 12'd394;  4'd7:    lut_val_19_16 = 12'd2679;
            4'd8:    lut_val_19_16 = 12'd1635; 4'd9:    lut_val_19_16 = 12'd591;
            4'd10:   lut_val_19_16 = 12'd2876; 4'd11:   lut_val_19_16 = 12'd1832;
            4'd12:   lut_val_19_16 = 12'd788;  4'd13:   lut_val_19_16 = 12'd3073;
            4'd14:   lut_val_19_16 = 12'd2029; 4'd15:   lut_val_19_16 = 12'd985;
            default: lut_val_19_16 = '0;
        endcase

        // LUT [23:20] is deferred to Cycle 2 to reduce Cycle 1 critical path
    end

    // Stage 1 Summation (Partial)
    // We sum the partial residues from the lower 20 bits.
    logic [13:0] stage1_sum_partial;
    assign stage1_sum_partial = lut_val_19_16 + lut_val_15_12 + chunk_11_00;

    // =========================================================================
    // CYCLE 2: Pipeline Register (Pipeline Stage 2)
    // =========================================================================
    logic [13:0] stage2_sum_partial;
    logic [3:0]  chunk_23_20_reg;
    logic valid_reg2;

    always_ff @(posedge clk) begin
        if (rst) begin
            stage2_sum_partial <= '0;
            chunk_23_20_reg    <= '0;
            valid_reg2         <= 1'b0;
        end else begin
            stage2_sum_partial <= stage1_sum_partial;
            chunk_23_20_reg    <= chunk_23_20;
            valid_reg2         <= valid_reg1;
        end
    end

    // =========================================================================
    // CYCLE 2 Logic: Second Reduction (4-Mux & Correction)
    // =========================================================================

    // 0. Deferred LUT [23:20] from Cycle 1
    logic [11:0] lut_val_23_20;
    always_comb begin
        case(chunk_23_20_reg)
            4'd0:    lut_val_23_20 = 12'd0;    4'd1:    lut_val_23_20 = 12'd3270;
            4'd2:    lut_val_23_20 = 12'd3211; 4'd3:    lut_val_23_20 = 12'd3152;
            4'd4:    lut_val_23_20 = 12'd3093; 4'd5:    lut_val_23_20 = 12'd3034;
            4'd6:    lut_val_23_20 = 12'd2975; 4'd7:    lut_val_23_20 = 12'd2916;
            4'd8:    lut_val_23_20 = 12'd2857; 4'd9:    lut_val_23_20 = 12'd2798;
            4'd10:   lut_val_23_20 = 12'd2739; 4'd11:   lut_val_23_20 = 12'd2680;
            4'd12:   lut_val_23_20 = 12'd2621; 4'd13:   lut_val_23_20 = 12'd2562;
            4'd14:   lut_val_23_20 = 12'd2503; 4'd15:   lut_val_23_20 = 12'd2444;
            default: lut_val_23_20 = '0;
        endcase
    end

    // Complete the Stage 1 Summation
    logic [13:0] stage2_sum;
    assign stage2_sum = stage2_sum_partial + lut_val_23_20;

    // =========================================================================
    // CYCLE 2 Logic: Second Reduction (4-Mux & Correction)
    // =========================================================================

    // 1. Split Register
    logic [1:0] stage2_sum_13_12;
    assign stage2_sum_13_12 = stage2_sum[13:12];

    logic [11:0] stage2_sum_11_0;
    assign stage2_sum_11_0 = stage2_sum[11:0];

    // 2. The "4-Mux" (Mini-Reduction)
    // Reduces bits 13:12 (Weight 2^12) into their mod 3329 equivalents.
    logic [11:0] lut_val_13_12;
    always_comb begin
        case (stage2_sum_13_12)
            2'd0:    lut_val_13_12 = 12'd0;
            2'd1:    lut_val_13_12 = 12'd767;
            2'd2:    lut_val_13_12 = 12'd1534;
            2'd3:    lut_val_13_12 = 12'd2301;
            default: lut_val_13_12 = 12'd0;
        endcase
    end

    // 3. Final Addition & Subtraction (Flattened to parallel 3-operand math)
    // CRITICAL: Must be 13 bits.
    // Max Possible: 4095 (stage2_lower) + 2301 (lut_val) = 6396.
    // Since 6396 < 2*Q (6658), we only need to subtract Q once.
    logic [12:0] final_sum;
    logic [12:0] final_sub;
    
    assign final_sum = stage2_sum_11_0 + lut_val_13_12;
    assign final_sub = stage2_sum_11_0 + lut_val_13_12 - 13'(Q);

    logic [11:0] final_result_wire;
    // If final_sub[12] is 1 (Negative/Underflow), result < Q, so keep final_sum.
    assign final_result_wire = (final_sub[12]) ? final_sum[11:0] : final_sub[11:0];

    // =========================================================================
    // CYCLE 3: Output Registration (Pipeline Stage 3)
    // =========================================================================

    always_ff @(posedge clk) begin
        if (rst) begin
            result_o <= '0;
            valid_o  <= 1'b0;
        end else begin
            result_o <= final_result_wire;
            valid_o  <= valid_reg2;
        end
    end

endmodule
