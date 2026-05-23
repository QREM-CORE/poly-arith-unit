import sys
import os
import random

# Add mlkem to path
# We assume the script is in verif/scripts/
sys.path.append(os.path.join(os.path.dirname(__file__), '../mlkem-python/src'))

try:
    from mlkem.auxiliaries import NTT, NTT_inv, MultiplyNTTs, q
except ImportError:
    print("Error: Could not import mlkem auxiliaries. Check if verif/mlkem-python/src exists.")
    sys.exit(1)

def write_mem(filename, coefficients):
    with open(filename, 'w') as f:
        for c in coefficients:
            # Write hex 4-digits (12-bit coeffs fit in 4 hex digits)
            f.write(f"{c:04x}\n")

def gen_ntt_vectors(k_dir):
    # Random
    f = [random.randint(0, q-1) for _ in range(256)]
    f_hat = NTT(f)
    write_mem(f"{k_dir}/ntt_in.mem", f)
    write_mem(f"{k_dir}/ntt_out.mem", f_hat)

    # Boundary (Max)
    f_max = [q-1] * 256
    f_max_hat = NTT(f_max)
    write_mem(f"{k_dir}/ntt_max_in.mem", f_max)
    write_mem(f"{k_dir}/ntt_max_out.mem", f_max_hat)

def gen_intt_vectors(k_dir):
    f_hat = [random.randint(0, q-1) for _ in range(256)]
    f = NTT_inv(f_hat)
    write_mem(f"{k_dir}/intt_in.mem", f_hat)
    write_mem(f"{k_dir}/intt_out.mem", f)

    # Boundary (Max)
    f_max_hat = [q-1] * 256
    f_max = NTT_inv(f_max_hat)
    write_mem(f"{k_dir}/intt_max_in.mem", f_max_hat)
    write_mem(f"{k_dir}/intt_max_out.mem", f_max)

def gen_addsub_vectors(k_dir):
    a = [random.randint(0, q-1) for _ in range(256)]
    b = [random.randint(0, q-1) for _ in range(256)]

    # Include some edge cases for modular reduction
    a[0] = q-1; b[0] = 1 # add -> 0
    a[1] = 0; b[1] = 1 # sub -> q-1

    c_add = [(ai + bi) % q for ai, bi in zip(a, b)]
    c_sub = [(ai - bi) % q for ai, bi in zip(a, b)]

    write_mem(f"{k_dir}/add_a.mem", a)
    write_mem(f"{k_dir}/add_b.mem", b)
    write_mem(f"{k_dir}/add_out.mem", c_add)

    write_mem(f"{k_dir}/sub_a.mem", a)
    write_mem(f"{k_dir}/sub_b.mem", b)
    write_mem(f"{k_dir}/sub_out.mem", c_sub)

def gen_cwm_vectors(k_dir, k):
    # Generate A_row (k polynomials), s vector (k polynomials), and e error polynomial
    A_row = [[random.randint(0, q-1) for _ in range(256)] for _ in range(k)]
    s = [[random.randint(0, q-1) for _ in range(256)] for _ in range(k)]
    e = [random.randint(0, q-1) for _ in range(256)]

    # Accumulate t_hat = sum(A_ij * s_j) + e_i
    t_hat = [0] * 256
    for j in range(k):
        prod = MultiplyNTTs(A_row[j], s[j])
        t_hat = [(t + p) % q for t, p in zip(t_hat, prod)]
        write_mem(f"{k_dir}/cwm_a{j}.mem", A_row[j])
        write_mem(f"{k_dir}/cwm_s{j}.mem", s[j])

    t_hat_final = [(t + ej) % q for t, ej in zip(t_hat, e)]
    write_mem(f"{k_dir}/cwm_e.mem", e)
    write_mem(f"{k_dir}/cwm_out.mem", t_hat_final)

    # Boundary (Max/Min mix)
    bound_vals = [0, q-1, q]
    A_row_max = [[bound_vals[(i + j) % 3] for i in range(256)] for j in range(k)]
    s_max = [[bound_vals[(i + j + 1) % 3] for i in range(256)] for j in range(k)]
    e_max = [bound_vals[(i + 2) % 3] for i in range(256)]

    t_hat_max = [0] * 256
    for j in range(k):
        prod = MultiplyNTTs(A_row_max[j], s_max[j])
        t_hat_max = [(t + p) % q for t, p in zip(t_hat_max, prod)]
        write_mem(f"{k_dir}/cwm_max_a{j}.mem", A_row_max[j])
        write_mem(f"{k_dir}/cwm_max_s{j}.mem", s_max[j])

    t_hat_final_max = [(t + ej) % q for t, ej in zip(t_hat_max, e_max)]
    write_mem(f"{k_dir}/cwm_max_e.mem", e_max)
    write_mem(f"{k_dir}/cwm_max_out.mem", t_hat_final_max)

    # Zero Noise (e = 0)
    A_row_zero = [[random.randint(0, q-1) for _ in range(256)] for _ in range(k)]
    s_zero = [[random.randint(0, q-1) for _ in range(256)] for _ in range(k)]
    e_zero = [0] * 256

    t_hat_zero = [0] * 256
    for j in range(k):
        prod = MultiplyNTTs(A_row_zero[j], s_zero[j])
        t_hat_zero = [(t + p) % q for t, p in zip(t_hat_zero, prod)]
        write_mem(f"{k_dir}/cwm_zero_a{j}.mem", A_row_zero[j])
        write_mem(f"{k_dir}/cwm_zero_s{j}.mem", s_zero[j])

    t_hat_final_zero = [(t + ej) % q for t, ej in zip(t_hat_zero, e_zero)]
    write_mem(f"{k_dir}/cwm_zero_e.mem", e_zero)
    write_mem(f"{k_dir}/cwm_zero_out.mem", t_hat_final_zero)

    if k == 2:
        # Generate the special k=1 verification vector for the testbench
        # t_hat_k1 = A_0 * s_0 + e
        prod_k1 = MultiplyNTTs(A_row[0], s[0])
        t_hat_k1 = [(p + ej) % q for p, ej in zip(prod_k1, e)]
        write_mem(f"{base_dir}/vectors/cwm_k1_out.mem", t_hat_k1)

if __name__ == "__main__":
    print("Generating PAU test vectors...")
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) # back to verif/

    for k in [2, 3, 4]:
        k_dir = os.path.join(base_dir, f"vectors/k{k}")
        os.makedirs(k_dir, exist_ok=True)
        print(f"  Processing k={k}...")
        gen_ntt_vectors(k_dir)
        gen_intt_vectors(k_dir)
        gen_addsub_vectors(k_dir)
        gen_cwm_vectors(k_dir, k)

    print("Done. Vectors stored in verif/vectors/kX/")
