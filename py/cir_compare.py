"""
cir_compare.py — 空车 vs 有人 CIR 对比分析

Usage:
    python cir_compare.py cir_empty.csv cir_human.csv
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')  # no GUI
import matplotlib.pyplot as plt
from pathlib import Path

def load_cir(filepath):
    """Load CIR CSV, return dict of arrays."""
    with open(filepath) as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(filepath, delimiter=",", skip_header=1, filling_values=0)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Column indices
    cols = {h: i for i, h in enumerate(header)}

    # Extract I/Q columns
    n_bins = 0
    while f"i{n_bins}" in cols:
        n_bins += 1

    result = {
        "file": Path(filepath).name,
        "data": data,
        "cols": cols,
        "n_bins": n_bins,
    }
    return result


def compute_phase(cir):
    """Compute phase = atan2(Q, I) for each frame, shape (n_frames, n_bins)."""
    data = cir["data"]
    n_bins = cir["n_bins"]
    cols = cir["cols"]
    phase = np.zeros((data.shape[0], n_bins), dtype=np.float64)
    for j in range(n_bins):
        i_idx = cols[f"i{j}"]
        q_idx = cols[f"q{j}"]
        phase[:, j] = np.arctan2(data[:, q_idx], data[:, i_idx])
    return phase


def compute_magnitude(cir):
    """Compute |I+jQ|."""
    data = cir["data"]
    n_bins = cir["n_bins"]
    cols = cir["cols"]
    mag = np.zeros((data.shape[0], n_bins), dtype=np.float64)
    for j in range(n_bins):
        i_idx = cols[f"i{j}"]
        q_idx = cols[f"q{j}"]
        mag[:, j] = np.sqrt(data[:, i_idx]**2 + data[:, q_idx]**2)
    return mag


def phase_variance(phase):
    """Per-bin circular variance of phase [0..1], lower=more stable."""
    mu_cos = np.mean(np.cos(phase), axis=0)
    mu_sin = np.mean(np.sin(phase), axis=0)
    R = np.sqrt(mu_cos**2 + mu_sin**2)  # resultant length
    return 1.0 - R  # circular variance, 0=perfectly stable, 1=uniform


def main():
    if len(sys.argv) < 3:
        print("Usage: python cir_compare.py <empty.csv> <human.csv>")
        sys.exit(1)

    empty = load_cir(sys.argv[1])
    human = load_cir(sys.argv[2])

    print(f"Empty: {empty['data'].shape[0]} frames, {empty['n_bins']} CIR bins")
    print(f"Human: {human['data'].shape[0]} frames, {human['n_bins']} CIR bins")

    # Phase
    phase_e = compute_phase(empty)
    phase_h = compute_phase(human)
    pvar_e = phase_variance(phase_e)
    pvar_h = phase_variance(phase_h)

    # Magnitude
    mag_e = compute_magnitude(empty)
    mag_h = compute_magnitude(human)
    mag_mean_e = np.mean(mag_e, axis=0)
    mag_mean_h = np.mean(mag_h, axis=0)
    mag_std_e  = np.std(mag_e, axis=0)
    mag_std_h  = np.std(mag_h, axis=0)

    # ── Plot ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"UWB Radar: Empty vs Human\n"
                 f"{empty['file'][:40]}\n{human['file'][:40]}",
                 fontsize=10)

    bins = np.arange(empty["n_bins"])

    # 1. Phase variance per bin
    ax = axes[0, 0]
    ax.plot(bins, pvar_e, "b-", alpha=0.7, label="Empty")
    ax.plot(bins, pvar_h, "r-", alpha=0.7, label="Human")
    ax.set_xlabel("CIR bin")
    ax.set_ylabel("Circular phase variance")
    ax.set_title("Phase Variance (↓ stable, ↑ disturbed)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Mean magnitude
    ax = axes[0, 1]
    ax.plot(bins, mag_mean_e, "b-", alpha=0.7, label="Empty")
    ax.plot(bins, mag_mean_h, "r-", alpha=0.7, label="Human")
    ax.set_xlabel("CIR bin")
    ax.set_ylabel("|I+jQ|")
    ax.set_title("Mean Magnitude Profile")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Magnitude std (variability indicator)
    ax = axes[1, 0]
    ax.plot(bins, mag_std_e, "b-", alpha=0.7, label="Empty")
    ax.plot(bins, mag_std_h, "r-", alpha=0.7, label="Human")
    ax.set_xlabel("CIR bin")
    ax.set_ylabel("Std |I+jQ|")
    ax.set_title("Magnitude Variability (↑ = more change)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Phase variance difference (Human - Empty)
    ax = axes[1, 1]
    diff = pvar_h - pvar_e
    colors = ["green" if d > 0.05 else ("red" if d < -0.05 else "gray")
              for d in diff]
    ax.bar(bins, diff, color=colors, width=1.0)
    ax.set_xlabel("CIR bin")
    ax.set_ylabel("Δ Phase Variance (Human - Empty)")
    ax.set_title("Phase Variance Difference\n(green=disturbed by human, red=stabilized)")
    ax.axhline(0, color="black", linewidth=0.5)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = Path(sys.argv[1]).parent / "cir_compare.png"
    plt.savefig(out, dpi=150)
    print(f"\nPlot saved: {out}")

    # ── Summary stats ──
    print(f"\n{'='*50}")
    print(f"Summary")
    print(f"{'='*50}")
    print(f"               Empty         Human")
    print(f"Frames:        {empty['data'].shape[0]:5d}         {human['data'].shape[0]:5d}")
    print(f"CIR bins:      {empty['n_bins']:5d}         {human['n_bins']:5d}")
    print(f"Mean phase var: {np.mean(pvar_e):.4f}       {np.mean(pvar_h):.4f}")
    print(f"Peak phase var: {np.max(pvar_e):.4f}       {np.max(pvar_h):.4f}")
    print(f"Mean magnitude: {np.mean(mag_mean_e):.0f}       {np.mean(mag_mean_h):.0f}")

    # Per-bin breathing candidate bins (highest phase variance change)
    top_bins = np.argsort(-np.abs(diff))[:5]
    print(f"\nTop-5 most-changed bins:")
    for i in top_bins:
        print(f"  bin={i:3d}  Δvar={diff[i]:+.4f}  (empty={pvar_e[i]:.4f} → human={pvar_h[i]:.4f})")


if __name__ == "__main__":
    main()
