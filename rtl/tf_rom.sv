/*
 * Module Name: tf_rom (Twiddle Factor ROM)
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
 * Four-ROM twiddle factor storage for the Mixed-Radix-4/2 NTT/INTT butterfly.
 *
 * ROM Organisation (from Algorithm 5 & 6 of the Inha paper):
 * +---------------+--------+----------+---------------------------------------+
 * | ROM           | Depth  | Width    | Content                               |
 * +---------------+--------+----------+---------------------------------------+
 * | R4NTT_ROM     | 21     | 36-bit   | {w1, w2, w3}  (NTT Radix-4 passes)   |
 * | OMEGA_ROM     | 64     | 12-bit   | w              (NTT Radix-2 pass)     |
 * | R4INTT_ROM    | 21     | 36-bit   | {w1^-1, w2^-1, w3^-1} pre-negated    |
 * | OMEGA_INV_ROM | 64     | 12-bit   | w^-1           pre-negated            |
 * +---------------+--------+----------+---------------------------------------+
 *
 * Total storage: 2 x (21x36) + 2 x (64x12) = 3048 bits (381 bytes)
 *
 * "Pre-negated" INTT trick:
 *   INTT ROMs store  Q - zeta^-1  instead of  zeta^-1.
 *   The PEs in INTT mode compute  (A - B) * W  for the V output.
 *   The algorithm actually needs  (B - A) * zeta^-1.
 *   Since  (A - B) * (Q - zeta^-1) = -(A - B) * zeta^-1 = (B - A) * zeta^-1 (mod Q),
 *   the negation is absorbed into the ROM constant, eliminating runtime
 *   subtractors on the twiddle factor path.
 *
 * Port Mapping to pe_unit (op_b bus):
 * +----------+---------------+--------------------+-----------------------------+
 * | ROM Port | pe_unit Input | Radix-4 Role       | Radix-2 Role                |
 * +----------+---------------+--------------------+-----------------------------+
 * | w0_o     | op_b0 (PE0)   | Stage B twiddle    | PE0 twiddle (omega)         |
 * | w1_o     | op_b1 (PE2 W1)| Stage A top twiddle| 1 (NTT) / 1665 (INTT)      |
 * | w2_o     | op_b2 (PE2 W2)| Stage A bot twiddle| PE2 twiddle (= same omega)  |
 * | w3_o     | op_b3 (PE3 TF)| omega_4 / ^(-1)    | omega_4 / ^(-1) (unused)    |
 * +----------+---------------+--------------------+-----------------------------+
 *
 * Radix-2 Note:
 *   PE0 and PE2 operate in parallel, each performing a separate butterfly.
 *   Both butterflies in the same block use the same twiddle factor, so
 *   w0_o == w2_o.  PE2's W1 input (op_b1) must be 1 for NTT (bypass) or
 *   1665 = 2^-1 mod Q for INTT (division by 2 via modular multiplication).
 *
 * All outputs are registered (1 clock cycle read latency).
 */

import poly_arith_pkg::*;

module tf_rom (
    input   logic           clk,
    input   logic           rst,

    // ---- Control ----
    input   logic           is_intt_i,      // 1 = INTT mode
    input   logic           is_radix2_i,    // 1 = Radix-2 pass
    input   logic           is_cwm_i,       // 1 = CWM mode (single omega stream)

    // ---- Address Inputs ----
    input   logic [5:0]     tf_addr_i,      // Unified ROM address (0..63)

    // ---- Twiddle Factor Outputs (registered) ----
    output  coeff_t         w0_o,           // PE0 twiddle factor
    output  coeff_t         w1_o,           // PE2 multiplier 1 twiddle factor
    output  coeff_t         w2_o,           // PE2 multiplier 2 twiddle factor
    output  coeff_t         w3_o            // PE3 Radix-4 root (omega_4)
);

    typedef logic [35:0] r4_rom_t [0:20];
    typedef logic [11:0] r2_rom_t [0:63];

    // =========================================================================
    // ROM Storage: R4NTT_ROM (21 x 36-bit)
    // =========================================================================
    // {w1[11:0], w2[11:0], w3[11:0]} -- Standard forward twiddle factors.
    // Addressed sequentially (t++) during NTT Radix-4 passes 1-3.
    //   Pass 1 (p=3, stages 1&2):  1 entry   [0]
    //   Pass 2 (p=2, stages 3&4):  4 entries  [1..4]
    //   Pass 3 (p=1, stages 5&6): 16 entries  [5..20]

    localparam r4_rom_t R4NTT_ROM = '{
        {12'd17,   12'd289,  12'd1584},  // [ 0] Pass 1
        {12'd296,  12'd1062, 12'd1409},  // [ 1] Pass 2
        {12'd1703, 12'd650,  12'd1063},  // [ 2]
        {12'd2319, 12'd1426, 12'd939 },  // [ 3]
        {12'd2804, 12'd2647, 12'd1722},  // [ 4]
        {12'd2642, 12'd2580, 12'd583 },  // [ 5] Pass 3
        {12'd1637, 12'd3253, 12'd2037},  // [ 6]
        {12'd1197, 12'd1339, 12'd2789},  // [ 7]
        {12'd375,  12'd807,  12'd403 },  // [ 8]
        {12'd3046, 12'd193,  12'd3281},  // [ 9]
        {12'd1847, 12'd2513, 12'd2773},  // [10]
        {12'd1438, 12'd535,  12'd2437},  // [11]
        {12'd1143, 12'd1481, 12'd1874},  // [12]
        {12'd2786, 12'd1897, 12'd2288},  // [13]
        {12'd756,  12'd2277, 12'd2090},  // [14]
        {12'd2865, 12'd2240, 12'd1461},  // [15]
        {12'd2099, 12'd1534, 12'd2775},  // [16]
        {12'd2393, 12'd569,  12'd3015},  // [17]
        {12'd733,  12'd1320, 12'd2466},  // [18]
        {12'd2474, 12'd1974, 12'd268 },  // [19]
        {12'd2110, 12'd1227, 12'd885 }   // [20]
    };

    // =========================================================================
    // ROM Storage: OMEGA_ROM (64 x 12-bit)
    // =========================================================================
    // Standard forward twiddle factors for NTT Radix-2 pass (stage 7).
    // Addressed by j/4 (Alg 5, line 16).

    localparam r2_rom_t OMEGA_ROM = '{
        12'd1729, 12'd2761, 12'd331,  12'd2298, 12'd2447, 12'd1651, 12'd1435, 12'd1092,
        12'd1919, 12'd2662, 12'd1977, 12'd319,  12'd2094, 12'd2308, 12'd2617, 12'd1212,
        12'd630,  12'd723,  12'd2304, 12'd2549, 12'd56,   12'd952,  12'd2868, 12'd2150,
        12'd3260, 12'd2156, 12'd33,   12'd561,  12'd2879, 12'd2337, 12'd3110, 12'd2935,
        12'd3289, 12'd2649, 12'd1756, 12'd3220, 12'd1476, 12'd1789, 12'd452,  12'd1026,
        12'd797,  12'd233,  12'd632,  12'd757,  12'd2882, 12'd2388, 12'd648,  12'd1029,
        12'd848,  12'd1100, 12'd2055, 12'd1645, 12'd1333, 12'd2687, 12'd2402, 12'd886,
        12'd1746, 12'd3050, 12'd1915, 12'd2594, 12'd821,  12'd641,  12'd910,  12'd2154
    };

    // =========================================================================
    // ROM Storage: R4INTT_ROM (21 x 36-bit)
    // =========================================================================
    // {w1_r4_intt[11:0], w0_r4_intt[11:0], w2_r4_intt[11:0]}.
    //
    // The merged radix-4 INTT datapath is not the simple "plug inverse twiddles
    // into the forward butterfly" mirror of NTT. PE3 keeps a fixed omega_4 path
    // while PE0/PE2 consume transformed inverse constants that realize:
    //   a0 = (s0 + d0) / 2
    //   a2 = (s0 - d0) * (2*w2)^(-1)
    //   a1 = (t0 + t1) * (4*omega_4*w1)^(-1)
    //   a3 = (t0 - t1) * (4*omega_4*w3)^(-1)
    //
    // To preserve the existing tf_rom output muxing:
    //   w0_o <- bits[23:12] = (2*w2)^(-1)
    //   w1_o <- bits[35:24] = (4*omega_4*w1)^(-1)
    //   w2_o <- bits[11:0]  = (4*omega_4*w3)^(-1)
    //
    // Address order mirrors the inverse of the forward radix-4 passes:
    //   Pass 1 (stages 6&7): forward R4NTT entries [5..20]
    //   Pass 2 (stages 4&5): forward R4NTT entries [1..4]
    //   Pass 3 (stages 2&3): forward R4NTT entry  [0]

    localparam r4_rom_t R4INTT_ROM = '{
        {12'd358,  12'd20,   12'd1137},  // [ 0] INTT Pass 1
        {12'd1196, 12'd1774, 12'd649 },  // [ 1]
        {12'd462,  12'd225,  12'd1972},  // [ 2]
        {12'd223,  12'd1648, 12'd3140},  // [ 3]
        {12'd2363, 12'd1699, 12'd2211},  // [ 4]
        {12'd139,  12'd1895, 12'd2035},  // [ 5]
        {12'd204,  12'd3301, 12'd2403},  // [ 6]
        {12'd12,   12'd2177, 12'd423 },  // [ 7]
        {12'd784,  12'd3014, 12'd1234},  // [ 8]
        {12'd2396, 12'd356,  12'd2262},  // [ 9]
        {12'd2295, 12'd2282, 12'd2231},  // [10]
        {12'd135,  12'd676,  12'd480 },  // [11]
        {12'd2162, 12'd705,  12'd2628},  // [12]
        {12'd323,  12'd947,  12'd2071},  // [13]
        {12'd19,   12'd441,  12'd2933},  // [14]
        {12'd2351, 12'd1499, 12'd828 },  // [15]
        {12'd1171, 12'd2456, 12'd1803},  // [16] INTT Pass 2
        {12'd1048, 12'd2128, 12'd467 },  // [17]
        {12'd2999, 12'd998,  12'd1142},  // [18]
        {12'd1743, 12'd637,  12'd2757},  // [19]
        {12'd611,  12'd2874, 12'd3262}   // [20] INTT Pass 3
    };

    // =========================================================================
    // ROM Storage: OMEGA_INV_ROM (64 x 12-bit)
    // =========================================================================
    // Inverse-half constants for INTT Radix-2 pass (stage 1).
    // Each entry stores (2*omega)^(-1) so PE0/PE2 reconstruct the original
    // pair directly from the final forward radix-2 stage outputs.

    localparam r2_rom_t OMEGA_INV_ROM = '{
        12'd800,  12'd1222, 12'd1051, 12'd3195, 12'd2342, 12'd2096, 12'd2669, 12'd157,
        12'd1380, 12'd277,  12'd2562, 12'd934,  12'd2209, 12'd2284, 12'd526,  12'd2185,
        12'd716,  12'd2392, 12'd924,  12'd446,  12'd1397, 12'd278,  12'd408,  12'd24,
        12'd1568, 12'd1463, 12'd1261, 12'd270,  12'd995,  12'd646,  12'd38,   12'd1373,
        12'd2039, 12'd2274, 12'd2092, 12'd1298, 12'd468,  12'd615,  12'd232,  12'd2951,
        12'd1936, 12'd1093, 12'd2610, 12'd741,  12'd1806, 12'd1477, 12'd1066, 12'd846,
        12'd2008, 12'd2468, 12'd341,  12'd1195, 12'd2616, 12'd1133, 12'd3004, 12'd960,
        12'd2798, 12'd1927, 12'd505,  12'd813,  12'd3181, 12'd2537, 12'd1520, 12'd1656
    };

    // =========================================================================
    // ROM Read Logic
    // =========================================================================
    logic [35:0] r4_data;
    logic [11:0] r2_data;
    logic [11:0] cwm_data;

    // HARDWARE CLAMP: Protects the 21-element R4 ROM from Out-of-Bounds access
    // by forcing the address to 0 during Radix-2 passes (where tf_addr_i goes up to 63).
    logic [4:0] safe_r4_addr;
    assign safe_r4_addr = is_radix2_i ? 5'd0 : tf_addr_i[4:0];

    // Radix-4 ROM read (Now 100% safe)
    assign r4_data = is_intt_i ? R4INTT_ROM[safe_r4_addr] : R4NTT_ROM[safe_r4_addr];

    // Radix-2 ROM read (Safely uses the full 6-bit bus)
    assign r2_data = is_intt_i ? OMEGA_INV_ROM[tf_addr_i] : OMEGA_ROM[tf_addr_i];

    // CWM uses the forward 64-entry omega stream addressed by block index.
    // It is not a radix-4 pass, even though pass_is_radix2 stays low in the controller.
    assign cwm_data = OMEGA_ROM[tf_addr_i];

    // Unpack the 36-bit Radix-4 word into three 12-bit twiddle factors
    coeff_t r4_w1, r4_w2, r4_w3;
    assign r4_w1 = r4_data[35:24];  // INTT: (4*omega_4*w1)^(-1)
    assign r4_w2 = r4_data[23:12];  // INTT: (2*w2)^(-1)
    assign r4_w3 = r4_data[11:0];   // INTT: (4*omega_4*w3)^(-1)

    // =========================================================================
    // Output Multiplexing & Registration (1 Clock Cycle Read Latency)
    // =========================================================================
    // CWM    : w0 = omega, w1 = 0, w2 = 0
    // Radix-4: w0 = w2 (Stage B→PE0), w1 = w1 (Stage A top→PE2 W1),
    //          w2 = w3 (Stage A bot→PE2 W2)
    // Radix-2: w0 = omega (PE0), w1 = 1|1665 (PE2 W1 constant),
    //          w2 = omega (PE2 W2, same twiddle as PE0)

    always_ff @(posedge clk) begin
        if (rst) begin
            w0_o <= '0;
            w1_o <= '0;
            w2_o <= '0;
        end else if (is_cwm_i) begin
            // CWM mode: PE_Unit only consumes op_b0_i as the per-block omega.
            w0_o <= cwm_data;
            w1_o <= '0;
            w2_o <= '0;
        end else if (is_radix2_i) begin
            // Radix-2 pass: PE0 and PE2 run parallel butterflies
            w0_o <= r2_data;                              // PE0 twiddle (NTT: omega, INTT: (2*omega)^(-1))
            w1_o <= is_intt_i ? INV_2_MOD_Q : 12'd1;     // PE2 W1: 2^-1 (INTT) or 1 (NTT bypass)
            w2_o <= r2_data;                              // PE2 W2: same twiddle as PE0
        end else begin
            // Radix-4 pass: three twiddle factors
            w0_o <= r4_w2;      // w2 -> PE0 (Stage B)
            w1_o <= r4_w1;      // w1 -> PE2 mul1 (Stage A top)
            w2_o <= r4_w3;      // w3 -> PE2 mul2 (Stage A bot)
        end
    end

    // =========================================================================
    // Fixed Omega_4 Output (PE3 Twiddle Factor)
    // =========================================================================
    // omega_4 = zeta^(N/4) = 1729.
    //
    // The merged radix-4 INTT PE factorization keeps the same fixed omega_4
    // multiplier on the PE3 difference path and folds the required inverse
    // scaling into R4INTT_ROM's per-pass constants.

    always_ff @(posedge clk) begin
        if (rst) begin
            w3_o <= '0;
        end else if (is_cwm_i) begin
            // Unused in CWM; drive 0 so the waveform is less misleading.
            w3_o <= '0;
        end else begin
            w3_o <= OMEGA_4_NTT;
        end
    end

endmodule
