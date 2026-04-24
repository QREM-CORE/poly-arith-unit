package poly_arith_pkg;

    import qrem_global_pkg::*;

    typedef enum logic [2:0] {
        PE_MODE_IDLE   = 3'd0,
        PE_MODE_NTT    = 3'd1,
        PE_MODE_INTT   = 3'd2,
        PE_MODE_CWM    = 3'd3,
        PE_MODE_ADDSUB = 3'd4,
        PE_MODE_COMP   = 3'd5,
        PE_MODE_DECOMP = 3'd6
    } pe_mode_e;
    // =========================================================================
    // 1. Bus & Interface Configuration (AXI4-Stream)
    // =========================================================================
    parameter int PAU_DWIDTH    = 256;          // Main data bus width (32 bytes)
    parameter int KEEP_WIDTH    = PAU_DWIDTH / 8; // TKEEP width (32 bits)
    parameter int BYTE_SIZE     = 8;

    // =========================================================================
    // 2. ML-KEM Specific Constants (FIPS 203)
    // =========================================================================
    parameter int LOG_N         = 8;            // log2(256)

    // Coefficient sizing
    parameter int STORE_WIDTH   = 16;           // Storage width (aligned for hardware)

    // Throughput Calculations
    // How many coefficients fit in one AXI beat? (256 / 16 = 16 coefficients)
    parameter int COEFFS_PER_BEAT = PAU_DWIDTH / STORE_WIDTH;

    // How many beats to transfer one full polynomial? (256 / 16 = 16 beats)
    parameter int BEATS_PER_POLY  = NCOEFF / COEFFS_PER_BEAT;

    typedef logic [11:0] coeff_t; // I/O type for coefficients (NTT and non-NTT values)

    // =========================================================================
    // 3. Zeta Pre-computed Values
    // =========================================================================
    //
    // Earlier revisions duplicated long zeta lookup tables in this package.
    // The active PAU datapath now sources live twiddle constants from tf_rom.sv,
    // and those package-level copies were unused. They also caused repeated
    // Icarus elaboration failures in the PAU compatibility testbenches.
    //
    // For this branch we keep the active twiddle storage in tf_rom.sv and leave
    // the package focused on shared scalar constants and typedefs.

    // =========================================================================
    // 4. Twiddle Factor Constants
    // =========================================================================

    // Radix-4 root of unity: omega_4 = zeta^(N/4) = 17^64 mod 3329
    // Used by PE3 during Radix-4 NTT/INTT butterfly passes.
    parameter logic [11:0] OMEGA_4_NTT  = 12'd1729;   // zeta^64 mod Q
    parameter logic [11:0] OMEGA_4_INTT = 12'd1600;   // zeta^(-64) mod Q

    // Modular inverse of 2:  2 * 1665 = 3330 ≡ 1 (mod 3329)
    // Used by PE2 in INTT Radix-2 mode to perform (A+B)/2 via multiplication.
    parameter logic [11:0] INV_2_MOD_Q  = 12'd1665;   // 2^(-1) mod Q

    // =========================================================================
    // 5. PE Constants
    // =========================================================================

    // -------------------------------------------------------------------------
    // Processing Element (PE) Operating Modes (Moved to qrem_global_pkg)
    // -------------------------------------------------------------------------

endpackage : poly_arith_pkg
