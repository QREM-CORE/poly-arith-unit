# Script to generate correct, bit-reversed twiddle factor tables for tf_rom.sv
# Derived from FIPS 203 standard definitions.

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
