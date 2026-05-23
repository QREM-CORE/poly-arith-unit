"""
================================================================================
SCRIPT: ML-KEM Component-Wise Multiplication (CWM) Software Simulator
AUTHOR: Kiet Le
PURPOSE:
    Provides a mathematical implementation and verification of the Component-Wise
    Multiplication (CWM) using Inha University's block-based twiddle factor scheme.

ALGORITHMIC FLOW:
    1. Loads the 64 twiddle factors (omega) used by the hardware for CWM.
    2. Performs block-by-block MultiplyNTTs.
       - Even pairs use gamma = omega.
       - Odd pairs use gamma = -omega mod Q.
    3. Accumulates results over k polynomials and adds the error term.

USAGE:
    Run this script directly to simulate CWM and validate correctness against
    standard FIPS 203 vectors generated in verif/vectors.
================================================================================
"""
import os
import sys

Q = 3329
N = 256

# Standard FIPS 203 Bit Reversal table for 7 bits
def bit_rev_7(val):
    return int(f"{val:07b}"[::-1], 2)

def get_zeta(i):
    power = bit_rev_7(i)
    return pow(17, power, Q)

def get_omega_rom():
    # CWM twiddle factor ROM has 64 entries (zetas 64 to 127)
    return [get_zeta(64 + b) for b in range(64)]

def load_mem(filepath):
    vals = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                vals.append(int(line, 16))
    return vals

def forward_cwm_inha(A_row, s, e):
    """
    Simulates CWM with accumulation:
    t_hat = sum_j (A_row[j] * s[j]) + e
    Uses Inha's 64-entry twiddle factor ROM approach.
    """
    OMEGA_ROM = get_omega_rom()
    t_hat = list(e)
    k = len(A_row)

    for j in range(k):
        a = A_row[j]
        b = s[j]

        # 64 blocks of 4 elements (two pairs per block)
        for i in range(64):
            omega = OMEGA_ROM[i]

            # 1. BaseCaseMultiply for EVEN pair (2*i)
            # gamma = omega
            a0, a1 = a[4*i], a[4*i+1]
            b0, b1 = b[4*i], b[4*i+1]
            c0 = (a0 * b0 + a1 * b1 * omega) % Q
            c1 = (a0 * b1 + a1 * b0) % Q

            t_hat[4*i]   = (t_hat[4*i]   + c0) % Q
            t_hat[4*i+1] = (t_hat[4*i+1] + c1) % Q

            # 2. BaseCaseMultiply for ODD pair (2*i + 1)
            # gamma = -omega mod Q
            gamma_odd = (-omega) % Q
            a2, a3 = a[4*i+2], a[4*i+3]
            b2, b3 = b[4*i+2], b[4*i+3]
            c2 = (a2 * b2 + a3 * b3 * gamma_odd) % Q
            c3 = (a2 * b3 + a3 * b2) % Q

            t_hat[4*i+2] = (t_hat[4*i+2] + c2) % Q
            t_hat[4*i+3] = (t_hat[4*i+3] + c3) % Q

    return t_hat

if __name__ == "__main__":
    print("--- Simulating Inha CWM from Math Specs ---")

    base_dir = "verif/vectors"
    all_success = True

    for k in [2, 3, 4]:
        k_dir = os.path.join(base_dir, f"k{k}")
        if not os.path.exists(k_dir):
            print(f"Skipping k={k}, directory not found: {k_dir}")
            continue

        print(f"\nProcessing k={k}...")

        try:
            A_row = [load_mem(os.path.join(k_dir, f"cwm_a{j}.mem")) for j in range(k)]
            s = [load_mem(os.path.join(k_dir, f"cwm_s{j}.mem")) for j in range(k)]
            e = load_mem(os.path.join(k_dir, "cwm_e.mem"))
            expected_out = load_mem(os.path.join(k_dir, "cwm_out.mem"))
        except FileNotFoundError as ex:
            print(f"Error loading vectors for k={k}: {ex}")
            all_success = False
            continue

        sim_out = forward_cwm_inha(A_row, s, e)

        mismatches = 0
        for i in range(256):
            if sim_out[i] != expected_out[i]:
                mismatches += 1
                if mismatches <= 10:
                    print(f"Mismatch at coeff[{i}]: got {sim_out[i]:03x}, expected {expected_out[i]:03x}")

        print(f"k={k} Verification results: {mismatches} mismatches out of 256.")
        if mismatches == 0:
            print(f"SUCCESS: k={k} CWM simulator output matches FIPS 203 standard!")
        else:
            print(f"FAILURE: k={k} CWM simulator output does not match FIPS 203 standard.")
            all_success = False

    if all_success:
        print("\nOVERALL SUCCESS: All K configurations passed.")
        sys.exit(0)
    else:
        print("\nOVERALL FAILURE: Some K configurations failed.")
        sys.exit(1)
