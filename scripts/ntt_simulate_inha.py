"""
================================================================================
SCRIPT: ML-KEM Mixed-Radix NTT (Algorithm 5) Software Simulator
AUTHOR: Kiet Le
PURPOSE:
    Provides a mathematical implementation and verification of the forward
    Number Theoretic Transform (NTT) using Inha University's Mixed-Radix-4/2
    algorithm.

ALGORITHMIC FLOW (Algorithm 5):
    1. Pass 1-3 (Radix-4): Combines Stages 1 & 2, Stages 3 & 4, and Stages 5 & 6
       respectively. Performs Cooley-Tukey butterfly operations in parallel
       using stride factors.
    2. Pass 4 (Radix-2): Processes Stage 7. Only 2 parallel processing elements
       are active for the final radix-2 transformation.

USAGE:
    Run this script directly to simulate the forward Mixed-Radix NTT and
    validate correctness against standard FIPS 203 vectors.
================================================================================
"""
import os

Q = 3329
N = 256

# Standard FIPS 203 Bit Reversal table for 7 bits
def bit_rev_7(val):
    # 7-bit reversal: reverse the binary string of length 7
    return int(f"{val:07b}"[::-1], 2)

def get_zeta(i):
    # Standard Kyber/FIPS-203 primitive 256-th root of unity is 17
    # zeta_i = 17^(br_7(i)) mod Q
    power = bit_rev_7(i)
    return pow(17, power, Q)

def get_omega_rom():
    # Final Radix-2 pass uses zetas 64 to 127
    return [get_zeta(64 + b) for b in range(64)]

def get_r4_rom():
    # Forward twiddles for Radix-4 stages
    # Returns 21 triplets: (omega1, omega2, omega3)
    rom = []

    # Pass 1: Stage 1 & 2
    # b = 0 -> wA = zeta_1, wB_top = zeta_2
    wA = get_zeta(1)
    wB_top = get_zeta(2)
    rom.append((wB_top, wA, (wB_top * wA) % Q))

    # Pass 2: Stage 3 & 4
    # b in 0..3 -> wA = zeta_(4+b), wB_top = zeta_(8+2*b)
    for b in range(4):
        wA = get_zeta(4 + b)
        wB_top = get_zeta(8 + 2 * b)
        rom.append((wB_top, wA, (wB_top * wA) % Q))

    # Pass 3: Stage 5 & 6
    # b in 0..15 -> wA = zeta_(16+b), wB_top = zeta_(32+2*b)
    for b in range(16):
        wA = get_zeta(16 + b)
        wB_top = get_zeta(32 + 2 * b)
        rom.append((wB_top, wA, (wB_top * wA) % Q))

    return rom

def load_mem(filepath):
    vals = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                vals.append(int(line, 16))
    return vals

def forward_ntt_inha(f):
    a = list(f)
    OMEGA_4 = 1729  # zeta^64 mod q

    R4_ROM = get_r4_rom()
    OMEGA_ROM = get_omega_rom()

    # Radix-4 Passes (Pass 1, 2, 3)
    rom_idx = 0
    for p in range(3, 0, -1):
        stride = 4 ** p  # 64, 16, 4

        # Number of blocks in this pass
        num_blocks = 256 // (4 * stride)

        for k in range(num_blocks):
            # Fetch twiddles for this block
            omega1, omega2, omega3 = R4_ROM[rom_idx]
            rom_idx += 1

            for j in range(stride):
                m = 4 * k * stride + j

                # Radix-4 Butterfly Equations
                r0 = (a[m]          + a[m + 2*stride] * omega2) % Q
                r1 = (a[m]          - a[m + 2*stride] * omega2) % Q
                r2 = (a[m + stride] * omega1 + a[m + 3*stride] * omega3) % Q
                r3 = (a[m + stride] * omega1 - a[m + 3*stride] * omega3) % Q

                # Write back
                a[m]            = (r0 + r2) % Q
                a[m + stride]   = (r0 - r2) % Q
                a[m + 2*stride] = (r1 + r3 * OMEGA_4) % Q
                a[m + 3*stride] = (r1 - r3 * OMEGA_4) % Q

    # Final Radix-2 Pass (Pass 4)
    for j in range(0, 256, 4):
        omega = OMEGA_ROM[j // 4]

        # Parallel Radix-2 butterflies
        u0 = a[j]
        u1 = a[j + 1]
        v0 = (a[j + 2] * omega) % Q
        v1 = (a[j + 3] * omega) % Q

        a[j]     = (u0 + v0) % Q
        a[j + 2] = (u0 - v0) % Q
        a[j + 1] = (u1 + v1) % Q
        a[j + 3] = (u1 - v1) % Q

    return a

if __name__ == "__main__":
    import sys
    print("--- Simulating Inha Algorithm 5 NTT from Math Specs ---")

    # Read vectors
    in_path = "verif/vectors/k2/ntt_in.mem"
    out_path = "verif/vectors/k2/ntt_out.mem"

    if not os.path.exists(in_path):
        print(f"Error: {in_path} not found.")
        sys.exit(1)

    ntt_in = load_mem(in_path)
    ntt_out = load_mem(out_path)

    ntt_res = forward_ntt_inha(ntt_in)

    mismatches = 0
    for i in range(256):
        if ntt_res[i] != ntt_out[i]:
            mismatches += 1
            if mismatches <= 10:
                print(f"Mismatch at coeff[{i}]: got {ntt_res[i]:03x}, expected {ntt_out[i]:03x}")

    print(f"Verification results: {mismatches} mismatches out of 256.")
    if mismatches == 0:
        print("SUCCESS: Simulator output matches FIPS 203 standard!")
    else:
        print("FAILURE: Simulator output does not match FIPS 203 standard.")
