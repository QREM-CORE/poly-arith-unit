"""
================================================================================
SCRIPT: Kyber/ML-KEM Radix-4 Twiddle Factor ROM Generator
AUTHOR: Kiet Le
PURPOSE:
    Generates bit-reversed, packed twiddle factor entries for the Radix-4
    stages in tf_rom.sv, ensuring compatibility with FIPS 203 standard.

ALGORITHMIC FLOW:
    1. Computes Bit-Reversal: Permutes indices using bit-reversal for 7 bits.
    2. Calculates Zetas: Powers of the 256-th root of unity (17) mod 3329.
    3. Formats Output: Groups twiddles into `{w1, w2, w3}` triplets for
       R4NTT_ROM and R4INTT_ROM arrays in SystemVerilog syntax.

USAGE:
    python3 scripts/generate_rom.py
================================================================================
"""

Q = 3329

def bit_rev_7(val):
    return int(f"{val:07b}"[::-1], 2)

def get_zeta(i):
    return pow(17, bit_rev_7(i), Q)

# R4NTT_ROM
r4_ntt_rom = []
# Pass 1
w2 = get_zeta(1)
w1 = get_zeta(2)
r4_ntt_rom.append((w1, w2, (w1 * w2) % Q))
# Pass 2
for k in range(4):
    w2 = get_zeta(4 + k)
    w1 = get_zeta(8 + 2 * k)
    r4_ntt_rom.append((w1, w2, (w1 * w2) % Q))
# Pass 3
for k in range(16):
    w2 = get_zeta(16 + k)
    w1 = get_zeta(32 + 2 * k)
    r4_ntt_rom.append((w1, w2, (w1 * w2) % Q))

print("=== R4NTT_ROM ===")
for idx, (w1, w2, w3) in enumerate(r4_ntt_rom):
    pass_label = "Pass 1" if idx == 0 else ("Pass 2" if idx <= 4 else "Pass 3")
    print(f"        {{12'd{w1:<4}, 12'd{w2:<4}, 12'd{w3:<4}}},  // [{idx:>2}] {pass_label}")

# R4INTT_ROM
INV2 = 1665
INV4 = 2497
r4_intt_rom = []
# Pass 1
for k in range(16):
    za = get_zeta(62 - 2 * k)
    zb = get_zeta(31 - k)
    r4_intt_rom.append(((-za * INV4) % Q, (-zb * INV2) % Q, (za * zb * INV4) % Q))
# Pass 2
for k in range(4):
    za = get_zeta(14 - 2 * k)
    zb = get_zeta(7 - k)
    r4_intt_rom.append(((-za * INV4) % Q, (-zb * INV2) % Q, (za * zb * INV4) % Q))
# Pass 3
za = get_zeta(2)
zb = get_zeta(1)
r4_intt_rom.append(((-za * INV4) % Q, (-zb * INV2) % Q, (za * zb * INV4) % Q))

print("\n=== R4INTT_ROM ===")
for idx, (w1, w0, w2) in enumerate(r4_intt_rom):
    pass_label = "Pass 1" if idx < 16 else ("Pass 2" if idx < 20 else "Pass 3")
    print(f"        {{12'd{w1:<4}, 12'd{w0:<4}, 12'd{w2:<4}}},  // [{idx:>2}] {pass_label}")

# OMEGA_ROM (Used for NTT Radix-2 and CWM)
omega_rom = [get_zeta(64 + b) for b in range(64)]
print("\n=== OMEGA_ROM ===")
for i in range(0, 64, 8):
    chunk = omega_rom[i:i+8]
    formatted = ", ".join([f"12'd{w:<4}" for w in chunk])
    print(f"        {formatted},")

# OMEGA_INV_ROM (Used for INTT Radix-2)
omega_inv_rom = [(-get_zeta(127 - b) * INV2) % Q for b in range(64)]
print("\n=== OMEGA_INV_ROM ===")
for i in range(0, 64, 8):
    chunk = omega_inv_rom[i:i+8]
    formatted = ", ".join([f"12'd{w:<4}" for w in chunk])
    print(f"        {formatted},")
