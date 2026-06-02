#!/usr/bin/env python3
"""
gen_test_cases.py
=================
Generates multiple test cases for the IEEE 802.11ad rate-13/16 LDPC decoder.
For each case, runs the quantized min-sum reference and prints:
  - expected C_hat (672 bits)
  - the quantized 5-bit Qv values
Also generates a Verilog $readmemh-compatible hex dump for use in a testbench.

Usage:
  python3 gen_test_cases.py

Outputs per test case:
  case_N_llr.txt    — 672 lines, one decimal int per line (the Qv values fed to RTL)
  case_N_chat.txt   — 672 lines of 0/1 (expected C_hat)
  case_N_info.txt   — human-readable summary

All-zero codeword tests: vary SNR and seed.
Non-zero codeword tests: pick random info bits, encode, then test.
"""

import numpy as np
import os

# ─── Parameters ───────────────────────────────────────────────────────────────
Z        = 42
N        = 672          # block length
K        = 546          # info bits (rate 13/16)
M        = N - K        # = 126 parity bits
MAX_ITER = 10
MAX_MAG  = 15           # 4-bit magnitude → max value for min1/min2
LV_MAX   = 63           # 7-bit signed accumulator clamp (optional, not applied here)

# Rate 13/16 QC base matrix (3 layers × 16 cols), -1 = zero submatrix
base = np.array([
    [29,30, 0, 8,33,22,17, 4,27,28,20,27,24,23,-1,-1],
    [37,31,18,23,11,21, 6,20,32, 9,12,29,10, 0,13,-1],
    [25,22, 4,34,31, 3,14,15, 4, 2,14,18,13,22,22,24],
], dtype=int)

NUM_LAYERS = 3
NUM_COLS   = 16

OUT_DIR = "test_cases"
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Build parity check matrix H (negative-shift convention) ──────────────────
def build_H():
    H = np.zeros((M, N), dtype=int)
    for l in range(NUM_LAYERS):
        for c in range(NUM_COLS):
            s = base[l, c]
            if s < 0:
                continue
            for n in range(Z):
                r = (n - s + Z) % Z          # H[l*Z+r, c*Z+n] = 1
                H[l*Z + r, c*Z + n] = 1
    return H

H = build_H()

def syndrome(H, cw):
    return H @ cw % 2

# ─── Simple encoder (systematic, parity by Gaussian elimination) ──────────────
# For all-zero codeword tests we don't need the encoder.
# For non-zero tests we use it.

def gf2_row_reduce(A):
    """In-place GF(2) row reduction; returns rank and pivot columns."""
    A = A.copy() % 2
    rows, cols = A.shape
    pivots = []
    r = 0
    for c in range(cols):
        found = -1
        for rr in range(r, rows):
            if A[rr, c]:
                found = rr
                break
        if found == -1:
            continue
        A[[r, found]] = A[[found, r]]
        for rr in range(rows):
            if rr != r and A[rr, c]:
                A[rr] = (A[rr] + A[r]) % 2
        pivots.append(c)
        r += 1
    return A, pivots

def encode(info_bits, H, K, N):
    """Systematic encoding: find parity bits p s.t. H*[info|p]^T = 0 mod 2."""
    # Identify parity columns (last M columns, since H is in systematic form for 802.11ad)
    # H = [H_info | H_parity]; solve H_parity * p = H_info * info (mod 2)
    M_ = N - K
    H_info   = H[:, :K]
    H_parity = H[:, K:]
    rhs = H_info @ info_bits % 2

    # Solve H_parity * p = rhs via GF2 back-substitution
    # Use numpy for simplicity (small matrix)
    # Build augmented system
    Aug = np.hstack([H_parity.copy() % 2, rhs.reshape(-1, 1)])
    Aug_rr, pivots = gf2_row_reduce(Aug)

    # Extract solution
    p = np.zeros(M_, dtype=int)
    for i, col in enumerate(pivots):
        if col < M_:
            p[col] = Aug_rr[i, -1]
    cw = np.concatenate([info_bits, p])
    assert np.all(syndrome(H, cw) == 0), "Encoding failed — syndrome non-zero"
    return cw

# ─── Quantised channel model ───────────────────────────────────────────────────
def channel_llr_quantized(codeword, snr_db, seed, bits_mag=4):
    """
    BPSK over AWGN.  Returns 5-bit 2's-complement LLR values in [-16..+15].
    Sign convention: positive LLR → more likely 0.
    """
    rng   = np.random.default_rng(seed)
    snr   = 10 ** (snr_db / 10.0)
    sigma = np.sqrt(1.0 / (2.0 * snr))
    bpsk  = 1.0 - 2.0 * codeword.astype(float)      # 0→+1, 1→-1
    y     = bpsk + sigma * rng.standard_normal(len(codeword))
    llr_f = 2.0 * y / (sigma ** 2)                  # soft LLR (float)
    clip  = 2 ** bits_mag                            # 16
    llr_q = np.clip(np.round(llr_f).astype(int), -clip, clip - 1)
    return llr_q

# ─── Quantised min-sum decoder (matches RTL) ──────────────────────────────────
def min_sum_quantized(llr_q, max_iter=MAX_ITER):
    """
    Piecewise flooding min-sum that mirrors the RTL:
      - differential-shift systolic pipeline
      - 4-bit magnitude for Lvc / min1 / min2
      - 7-bit accumulator for Lv (no overflow in practice for 3 layers)
    Returns (C_hat, num_errors_per_iter).
    """
    def perm(buf, shift):
        if shift <= 0:
            return buf.copy()
        out = np.zeros_like(buf)
        for n in range(Z):
            out[n] = buf[(n - shift + Z) % Z]
        return out

    def diff_shift(l, c):
        curr = base[l, c]
        if curr < 0:
            return -1           # bypass → identity
        if c == 0:
            prev = 0
        else:
            prev = base[l, c - 1]
            if prev < 0:
                prev = 0
        return (curr - prev + Z) % Z

    # Last active column per layer
    last_act = []
    for l in range(NUM_LAYERS):
        for c in range(NUM_COLS - 1, -1, -1):
            if base[l, c] >= 0:
                last_act.append(c)
                break

    def vn_wrap_shift(l):
        la = last_act[l]
        return (base[l, 0] - base[l, la] + Z) % Z

    # lvc stored as (sign=0/1, magnitude=0..15)
    lvc_sign = np.zeros((NUM_COLS, Z, NUM_LAYERS), dtype=int)
    lvc_mag  = np.zeros((NUM_COLS, Z, NUM_LAYERS), dtype=int)
    C_hat    = np.zeros(N, dtype=int)
    errs_per_iter = []

    for iteration in range(1, max_iter + 1):
        first = (iteration == 1)

        # ── CN PASS ────────────────────────────────────────────────────────
        cur_sc = np.zeros((Z, NUM_LAYERS), dtype=int)
        cur_m1 = np.full((Z, NUM_LAYERS), MAX_MAG, dtype=int)
        cur_m2 = np.full((Z, NUM_LAYERS), MAX_MAG, dtype=int)

        for c in range(NUM_COLS):
            # Permute
            for l in range(NUM_LAYERS):
                sh = diff_shift(l, c)
                if sh > 0:
                    cur_sc[:, l] = perm(cur_sc[:, l], sh)
                    cur_m1[:, l] = perm(cur_m1[:, l], sh)
                    cur_m2[:, l] = perm(cur_m2[:, l], sh)

            new_sc = cur_sc.copy()
            new_m1 = cur_m1.copy()
            new_m2 = cur_m2.copy()

            for v in range(Z):
                qv   = int(llr_q[c * Z + v])
                qv_s = 1 if qv < 0 else 0
                qv_m = min(abs(qv), MAX_MAG)
                for l in range(NUM_LAYERS):
                    if base[l, c] < 0:
                        continue                    # bypass
                    lvc_s = qv_s if first else int(lvc_sign[c, v, l])
                    lvc_m = qv_m if first else int(lvc_mag [c, v, l])
                    sc0, m10, m20 = int(cur_sc[v, l]), int(cur_m1[v, l]), int(cur_m2[v, l])
                    new_sc[v, l] = sc0 ^ lvc_s
                    if   lvc_m < m10: new_m1[v, l], new_m2[v, l] = lvc_m, m10
                    elif lvc_m < m20: new_m1[v, l], new_m2[v, l] = m10,   lvc_m
                    # else: m1/m2 unchanged

            cur_sc, cur_m1, cur_m2 = new_sc, new_m1, new_m2

        final_sc = cur_sc.copy()
        final_m1 = cur_m1.copy()
        final_m2 = cur_m2.copy()

        # ── VN PASS ────────────────────────────────────────────────────────
        cur_sc2 = final_sc.copy()
        cur_m12 = final_m1.copy()
        cur_m22 = final_m2.copy()
        new_lvc_sign = lvc_sign.copy()
        new_lvc_mag  = lvc_mag.copy()

        for c in range(NUM_COLS):
            for l in range(NUM_LAYERS):
                sh = vn_wrap_shift(l) if c == 0 else diff_shift(l, c)
                if sh > 0:
                    cur_sc2[:, l] = perm(cur_sc2[:, l], sh)
                    cur_m12[:, l] = perm(cur_m12[:, l], sh)
                    cur_m22[:, l] = perm(cur_m22[:, l], sh)

            for v in range(Z):
                qv   = int(llr_q[c * Z + v])
                qv_s = 1 if qv < 0 else 0
                qv_m = min(abs(qv), MAX_MAG)
                Lv   = qv
                mcvs = [0, 0, 0]

                for l in range(NUM_LAYERS):
                    if base[l, c] < 0:
                        continue
                    lvc_s = qv_s if first else int(lvc_sign[c, v, l])
                    lvc_m = qv_m if first else int(lvc_mag [c, v, l])
                    sc_f  = int(cur_sc2[v, l])
                    m1_f  = int(cur_m12[v, l])
                    m2_f  = int(cur_m22[v, l])
                    is_m1 = (lvc_m <= m1_f)        # RTL uses <=
                    mcv_m = m2_f if is_m1 else m1_f
                    mcv   = -mcv_m if (sc_f ^ lvc_s) else mcv_m
                    mcvs[l] = mcv
                    Lv += mcv

                C_hat[c * Z + v] = 1 if Lv < 0 else 0

                for l in range(NUM_LAYERS):
                    if base[l, c] < 0:
                        continue
                    lvc_new = Lv - mcvs[l]
                    new_lvc_sign[c, v, l] = 1 if lvc_new < 0 else 0
                    new_lvc_mag [c, v, l] = min(abs(lvc_new), MAX_MAG)

        lvc_sign = new_lvc_sign
        lvc_mag  = new_lvc_mag

        # Check syndrome
        syn = syndrome(H, C_hat)
        errs = int(np.sum(syn))       # number of unsatisfied checks (not bit errors)
        errs_per_iter.append(errs)
        if errs == 0:
            break                     # early termination

    return C_hat, errs_per_iter

# ─── Test case definitions ─────────────────────────────────────────────────────
# Each entry: (label, codeword_type, snr_db, seed)
#   codeword_type = 'zero'   → all-zero codeword
#   codeword_type = 'random' → random info bits, encoded

cases = [
    # All-zero codeword, varying SNR and seed
    ("allzero_snr5_seed5",    "zero",   5.0,  5),
    ("allzero_snr4_seed1",    "zero",   4.0,  1),
    ("allzero_snr4_seed2",    "zero",   4.0,  2),
    ("allzero_snr3p5_seed10", "zero",   3.5, 10),
    ("allzero_snr3_seed7",    "zero",   3.0,  7),
    ("allzero_snr6_seed99",   "zero",   6.0, 99),
    # Non-zero codewords
    ("random_snr5_seed42",    "random", 5.0, 42),
    ("random_snr4_seed123",   "random", 4.0, 123),
    ("random_snr6_seed7",     "random", 6.0,  7),
    ("random_snr3_seed55",    "random", 3.0, 55),
]

print(f"{'Case':<30}  {'SNR':>6}  {'Converged?':>10}  {'Iters':>5}  {'Bit errors':>10}")
print("-" * 70)

for label, cw_type, snr_db, seed in cases:
    rng = np.random.default_rng(seed * 1000)   # separate RNG for info bits

    if cw_type == "zero":
        codeword = np.zeros(N, dtype=int)
    else:
        info = rng.integers(0, 2, K)
        codeword = encode(info, H, K, N)
        assert np.all(syndrome(H, codeword) == 0)

    llr_q = channel_llr_quantized(codeword, snr_db, seed)
    C_hat, errs_per_iter = min_sum_quantized(llr_q)

    converged  = (errs_per_iter[-1] == 0)
    iters_used = len(errs_per_iter)
    bit_errors = int(np.sum(C_hat != codeword))

    print(f"{label:<30}  {snr_db:>6.1f}  {'YES' if converged else 'NO':>10}  "
          f"{iters_used:>5}  {bit_errors:>10}")

    # ── Write output files ────────────────────────────────────────────────
    base_path = os.path.join(OUT_DIR, label)

    # LLR file: one value per line in decimal (signed)
    with open(base_path + "_llr.txt", "w") as f:
        for v in llr_q:
            f.write(f"{v}\n")

    # C_hat expected: 672 lines of 0 or 1
    with open(base_path + "_chat.txt", "w") as f:
        for b in codeword:        # expected = the transmitted codeword
            f.write(f"{b}\n")

    # Hex dump for $readmemh (5-bit 2's complement, packed as 2 hex digits)
    with open(base_path + "_llr.hex", "w") as f:
        for v in llr_q:
            f.write(f"{v & 0x1F:02x}\n")   # 5-bit mask

    # Human-readable summary
    with open(base_path + "_info.txt", "w") as f:
        f.write(f"Label      : {label}\n")
        f.write(f"CW type    : {cw_type}\n")
        f.write(f"SNR (dB)   : {snr_db}\n")
        f.write(f"Seed       : {seed}\n")
        f.write(f"Converged  : {converged}\n")
        f.write(f"Iters used : {iters_used}\n")
        f.write(f"Bit errors : {bit_errors}\n")
        f.write(f"Errs/iter  : {errs_per_iter}\n")
        f.write(f"\nTransmitted codeword (first 42 bits = col0):\n")
        f.write("".join(map(str, codeword[:42])) + "\n")
        f.write(f"\nDecoded C_hat (first 42 bits = col0):\n")
        f.write("".join(map(str, C_hat[:42])) + "\n")
        f.write(f"\nQv LLR values (first 42 = col0):\n")
        f.write(" ".join(map(str, llr_q[:42])) + "\n")

print(f"\nFiles written to ./{OUT_DIR}/")
print("\nHow to use in a testbench:")
print("  1. Load <case>_llr.hex with $readmemh into a 5-bit reg array of size 672")
print("  2. Drive them into the decoder's Qv memory the same way ldpc_decoder_tb_py.v does")
print("  3. After done, compare C_hat against <case>_chat.txt")
print("\nNote: cases at SNR ≤ 3.5 dB may not converge in 10 iterations — this is expected.")
