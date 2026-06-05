"""
cir_check.py — Quick CIR stability check for recorded CSV files.

Usage:
    python cir_check.py                              # use the newest .csv in py/
    python cir_check.py cir_empty_20260101.csv       # specific file
    python cir_check.py --plot                       # also show a plot
"""

import sys
import os
import glob
import csv
import numpy as np

SAVE_DIR = os.path.dirname(os.path.abspath(__file__))

HEADER_BASELINE = [
    "timestamp_ms", "seq", "fp_index", "rx_power",
    "fp_power", "noise"
]
CIR_COUNT_ASSUMED = 128

def _is_data_row_cell(cell: str) -> bool:
    s = cell.strip()
    return s.lstrip("-").isdigit()

def _load_csv(path: str) -> dict:
    with open(path, "r") as f:
        reader = csv.reader(f)
        raw = list(reader)

    if not raw:
        raise ValueError("empty CSV")

    # ---- decide: has header or not ----
    first = raw[0]
    if first[0].strip().startswith("timestamp_ms"):
        # header present
        headers = [h.strip() for h in first]
        data    = raw[1:]
    elif _is_data_row_cell(first[0]):
        # no header — reconstruct
        n_extra = len(first) - len(HEADER_BASELINE) - 1  # -1 for label column
        if n_extra >= 1:
            cir_cols = [f"cir{i}" for i in range(n_extra)]
        else:
            cir_cols = [f"cir{i}" for i in range(CIR_COUNT_ASSUMED)]
            # pad or truncate to match actual row length
            cir_cols = cir_cols[:max(0, len(first) - len(HEADER_BASELINE) - 1)]
        headers = HEADER_BASELINE + cir_cols + ["label"]
        data    = raw
        print(f"[!] No header found, inferred {len(cir_cols)} CIR bins.")
    else:
        raise ValueError(f"unexpected first line: {first[0][:60]}")

    # ---- find cir columns ----
    cir_indices = [i for i, h in enumerate(headers) if h.startswith("cir")]
    n_bins      = len(cir_indices)
    n_frames    = len(data)

    if n_bins == 0:
        raise ValueError("no 'cirXX' columns found in header.  "
                         "The CSV may be missing a header and the recorder did not re-request it.\n"
                         "Re-run the MCU and recorder so the CSV header line is captured, "
                         "or edit cir_check.py to add an explicit column map.")

    # ---- allocate ----
    ts   = np.zeros(n_frames, dtype=np.uint32)
    seq  = np.zeros(n_frames, dtype=np.uint32)
    fp   = np.zeros(n_frames, dtype=np.uint32)
    rxp  = np.zeros(n_frames, dtype=np.uint32)
    noise= np.zeros(n_frames, dtype=np.uint32)
    cir  = np.zeros((n_frames, n_bins), dtype=np.float64)
    label_str = ""

    for i, row in enumerate(data):
        if len(row) < 6 + n_bins:
            continue
        ts[i]    = int(row[0])
        seq[i]   = int(row[1])
        fp[i]    = int(row[2])
        rxp[i]   = int(row[3])
        noise[i] = int(row[5])
        for j, ci in enumerate(cir_indices):
            if ci < len(row):
                cir[i, j] = float(row[ci])
        label_str = row[-1].strip() if row[-1].strip() else label_str

    return {
        "headers":      headers,
        "timestamp_ms": ts,
        "seq":          seq,
        "fp_index":     fp,
        "rx_power":     rxp,
        "noise":        noise,
        "cir":          cir,
        "label":        label_str,
        "path":         path,
    }

# ---- print_stats ---- uses BASELINE-SUBTRACTED CIR for SNR
def print_stats(d: dict):
    print(f"\n{'='*60}")
    print(f" File: {os.path.basename(d['path'])}")
    print(f" Label: {d['label']}")
    print(f" Frames: {d['cir'].shape[0]}   CIR bins: {d['cir'].shape[1]}")
    print(f"{'='*60}")

    cir = d["cir"]
    fp  = d["fp_index"]

    # ---- noise floor: mean of first 20 bins, per frame then averaged ----
    noise_per_frame = cir[:, :20].mean(axis=1)   # shape (N,)
    # baseline-subtracted CIR
    cir_bs = cir - noise_per_frame[:, np.newaxis]
    # clamp negative to zero
    cir_bs = np.maximum(cir_bs, 0)

    cir_mean    = cir.mean(axis=0)
    cir_bs_mean = cir_bs.mean(axis=0)
    cir_bs_std  = cir_bs.std(axis=0)

    peak_bin = np.argmax(cir_bs_mean)
    peak_mag = cir_bs_mean[peak_bin]
    noise_floor = noise_per_frame.mean()

    print(f"\n--- First Path Index ---")
    print(f"  mean={fp.mean():.1f}  std={fp.std():.1f}  min={fp.min()}  max={fp.max()}")
    jumps = np.sum(np.abs(np.diff(fp.astype(int))) > 3)
    print(f"  jumps >3 bins: {jumps}  ({jumps/max(1,len(fp)-1)*100:.1f}%)")

    print(f"\n--- CIR (baseline-subtracted) ---")
    print(f"  noise floor (bins 0-20):  {noise_floor:,.0f}")
    print(f"  peak bin:                 {peak_bin}")
    print(f"  peak magnitude:           {peak_mag:,.0f}")
    print(f"  Peak-to-Noise Ratio:      {peak_mag/(noise_floor+1):.1f}x")

    # ---- SNR (baseline-corrected) ----
    bs_snr = np.zeros(cir.shape[1])
    for j in range(cir.shape[1]):
        m = cir_bs_mean[j]; s = cir_bs_std[j]
        bs_snr[j] = m / s if s > 0 else 0
    top10 = np.argsort(bs_snr)[-10:][::-1]
    print(f"\n--- Top 10 bins (baseline-corrected SNR) ---")
    for rank, b in enumerate(top10):
        print(f"  bin{b:4d}  mag={cir_bs_mean[b]:9.0f}  SNR={bs_snr[b]:5.1f}")

    good = np.sum(bs_snr > 3)
    print(f"\n--- Overall ---")
    print(f"  Bins with SNR >3: {good}/{cir.shape[1]}  ({good/cir.shape[1]*100:.1f}%)")
    if good > 5:
        print(f"  -> CIR DETECTED (usable peaks)")
    elif good > 2:
        print(f"  -> CIR marginal — check distance / alignment")
    else:
        print(f"  -> No clear peak — adjust TX power or distance")

    rp = d["rx_power"]
    print(f"\n--- RX Power ---")
    print(f"  mean={rp.mean():.1f}  std={rp.std():.1f}")

    ts = d["timestamp_ms"]
    if len(ts) > 1:
        intervals = np.diff(ts.astype(np.int64))
        print(f"\n--- Timing ---")
        print(f"  duration: {(ts[-1]-ts[0])/1000:.1f} s")
        print(f"  interval: mean={intervals.mean():.0f} ms  std={intervals.std():.1f} ms")
    print()

# ---- main ----
def main():
    path    = None
    do_plot = False
    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a == "--plot":
            do_plot = True
        elif a.endswith(".csv"):
            path = a
        else:
            print(f"[!] unknown arg: {a}")
            return

    if path is None:
        csvs = glob.glob(os.path.join(SAVE_DIR, "cir_*.csv"))
        if not csvs:
            print("[!] No cir_*.csv found in py/")
            return
        path = max(csvs, key=os.path.getmtime)

    if not os.path.isabs(path):
        path = os.path.join(SAVE_DIR, path)

    d = _load_csv(path)
    print_stats(d)

    if do_plot:
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[!] pip install matplotlib for plots")
            return

        cir = d["cir"]
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        ax = axes[0, 0]
        ax.plot(cir.mean(axis=0), "b-", alpha=0.8, label="mean CIR")
        ax.fill_between(range(cir.shape[1]),
                         cir.mean(axis=0)-cir.std(axis=0),
                         cir.mean(axis=0)+cir.std(axis=0),
                         alpha=0.2, label="+-1 sigma")
        ax.axvline(d["fp_index"].mean(), color="r", ls="--", label="mean FP")
        ax.set_xlabel("CIR bin"); ax.set_ylabel("Magnitude")
        ax.set_title(f"CIR Profile - {d['label']} ({cir.shape[0]} frames)")
        ax.legend()

        ax = axes[0, 1]
        N = min(20, cir.shape[0])
        for i in range(N): ax.plot(cir[i], alpha=0.4, lw=0.5)
        ax.set_xlabel("CIR bin"); ax.set_title(f"First {N} frames overlay")

        ax = axes[1, 0]
        ax.plot(d["timestamp_ms"]/1000.0, d["fp_index"], "g.-", ms=2)
        ax.set_xlabel("Time (s)"); ax.set_ylabel("FP index")
        ax.set_title("First Path index stability")

        ax = axes[1, 1]
        ax.plot(d["timestamp_ms"]/1000.0, d["rx_power"], "m.-", ms=2)
        ax.set_xlabel("Time (s)"); ax.set_ylabel("RX Power")
        ax.set_title("RX Power trend")

        plt.tight_layout(); plt.show()

if __name__ == "__main__":
    main()
