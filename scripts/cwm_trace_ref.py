"""
CWM Reference Trace Generator
Prints per-pair input/output for the first N pairs of a single k=1 CWM pass.
Used to compare against RTL debug trace from poly_arith_unit_tb.
"""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
from cwm_simulate_inha import load_mem, get_omega_rom

Q = 3329
TRACE_PAIRS = 8  # Number of pairs to trace

def run_trace(a, b, e, n_pairs=TRACE_PAIRS):
    OMEGA_ROM = get_omega_rom()

    print("=" * 70)
    print("CWM Python Reference Trace (k=1, first {} pairs)".format(n_pairs))
    print("=" * 70)

    t_hat = list(e)

    for pair_idx in range(128):
        block = pair_idx // 2
        omega = OMEGA_ROM[block]

        if pair_idx % 2 == 0:
            # Even pair — gamma = omega
            gamma = omega
            a0, a1 = a[4*(pair_idx//2)],   a[4*(pair_idx//2)+1]
            b0, b1 = b[4*(pair_idx//2)],   b[4*(pair_idx//2)+1]
        else:
            # Odd pair  — gamma = -omega mod Q
            gamma = (-omega) % Q
            a0, a1 = a[4*(pair_idx//2)+2], a[4*(pair_idx//2)+3]
            b0, b1 = b[4*(pair_idx//2)+2], b[4*(pair_idx//2)+3]

        c0 = (a0 * b0 + a1 * b1 * gamma) % Q
        c1 = (a0 * b1 + a1 * b0) % Q

        coeff_hi = 2 * pair_idx
        coeff_lo = 2 * pair_idx + 1

        if pair_idx < n_pairs:
            print(f"Pair {pair_idx:3d} | block={block:2d} | gamma={gamma:4d}({gamma:03x})")
            print(f"  A:  a0={a0:4d}({a0:03x})  a1={a1:4d}({a1:03x})")
            print(f"  B:  b0={b0:4d}({b0:03x})  b1={b1:4d}({b1:03x})")
            print(f"  C:  c0={c0:4d}({c0:03x})  c1={c1:4d}({c1:03x})")
            print(f"  Coeff idx: {coeff_hi}, {coeff_lo}")
            print()

    print("First 16 expected output coefficients:")
    t_hat_check = list(e)
    for i in range(64):
        block = i // 2
        omega = OMEGA_ROM[block]
        if i % 2 == 0:
            gamma = omega
            a0, a1 = a[4*(i//2)],   a[4*(i//2)+1]
            b0, b1 = b[4*(i//2)],   b[4*(i//2)+1]
        else:
            gamma = (-omega) % Q
            a0, a1 = a[4*(i//2)+2], a[4*(i//2)+3]
            b0, b1 = b[4*(i//2)+2], b[4*(i//2)+3]
        c0 = (a0 * b0 + a1 * b1 * gamma) % Q
        c1 = (a0 * b1 + a1 * b0) % Q
        t_hat_check[2*i]   = (t_hat_check[2*i]   + c0) % Q
        t_hat_check[2*i+1] = (t_hat_check[2*i+1] + c1) % Q

    for i in range(16):
        print(f"  coeff[{i:3d}] = {t_hat_check[i]:4d} ({t_hat_check[i]:03x})")

if __name__ == "__main__":
    base_dir = "verif/vectors/k2"
    a = load_mem(f"{base_dir}/cwm_a0.mem")
    b = load_mem(f"{base_dir}/cwm_s0.mem")
    e = load_mem(f"{base_dir}/cwm_e.mem")
    run_trace(a, b, e)
