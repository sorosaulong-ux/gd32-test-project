"""
cir_breath.py — Breathing detection via phase FFT

Loads I/Q CSV, tracks phase of peak bin over time,
runs FFT to find breathing frequency (0.1-0.5 Hz).

Usage:
    python cir_breath.py human.csv
    python cir_breath.py empty.csv human.csv   (compare two)
"""

import sys, os, glob, csv
import numpy as np

DIR = os.path.dirname(os.path.abspath(__file__))
FS   = 4.0   # sample rate (Hz) — matches 250ms TX interval


def load_iq(path):
    with open(path) as f:
        raw = list(csv.reader(f))
    hdr  = [c.strip() for c in raw[0]]
    data = raw[1:]
    lbl  = data[0][-1].strip() if data else "?"
    # find i0,i1,... columns
    i_cols = []
    for idx, name in enumerate(hdr):
        if name.startswith("i") and name[1:].isdigit():
            i_cols.append((int(name[1:]), idx))
    i_cols.sort()
    nb = len(i_cols)
    nf = len(data)
    I = np.zeros((nf, nb), dtype=np.float64)
    Q = np.zeros((nf, nb), dtype=np.float64)
    ts = np.zeros(nf, dtype=np.float64)
    for fi, row in enumerate(data):
        ts[fi] = float(row[0]) / 1000.0 if row[0] else fi / FS
        for bj, (_, ci) in enumerate(i_cols):
            if ci < len(row):     I[fi, bj] = float(row[ci])
            if ci+1 < len(row):   Q[fi, bj] = float(row[ci+1])
    print(f"  load: {os.path.basename(path)} -> {nf}f x {nb}bins  label={lbl}")
    return dict(i=I, q=Q, ts=ts, n_frames=nf, n_bins=nb, label=lbl, fname=os.path.basename(path))


def phase_of_bin(d, bin_idx):
    """Extract unwrapped phase time series for a specific CIR bin."""
    ph = np.arctan2(d["q"][:, bin_idx], d["i"][:, bin_idx])
    return np.unwrap(ph)


def fft_breathing(phase, fs=FS):
    """Return (freqs, psd) — PSD of phase time series, focus on 0.05-1 Hz."""
    n = len(phase)
    if n < 60:
        return np.array([]), np.array([])
    # detrend
    ph = phase - np.polyval(np.polyfit(np.arange(n), phase, 2), np.arange(n))
    # hamming window
    w = np.hamming(n)
    ph_w = ph * w
    # FFT
    fft = np.fft.rfft(ph_w)
    freq = np.fft.rfftfreq(n, d=1.0/fs)
    psd  = np.abs(fft)**2 / n
    return freq, psd


def main():
    args = sys.argv[1:]
    files = []
    for a in args:
        if a.endswith(".csv"):
            p = a if os.path.isabs(a) else os.path.join(DIR, a)
            files.append(p)

    if not files:
        # auto: latest human + latest empty
        for pat in ["cir_human_*.csv", "cir_breath_*.csv", "cir_empty_*.csv"]:
            for f in sorted(glob.glob(os.path.join(DIR, pat))):
                if f not in files:
                    files.append(f)
                if len(files) >= 2:
                    break
            if len(files) >= 2:
                break

    if len(files) < 1:
        print("Usage: python cir_breath.py <human.csv> [empty.csv]")
        return

    datasets = [load_iq(f) for f in files]

    d = datasets[0]  # primary
    nb = d["n_bins"]
    nf = d["n_frames"]
    duration = (d["ts"][-1] - d["ts"][0])

    print(f"\n{'='*60}")
    print(f" Breathing Analysis — {d['fname']}")
    print(f" {nf} frames  {nb} bins  {duration:.0f}s  ({FS} Hz)")
    print(f"{'='*60}")

    # ---- find bin with largest magnitude variance (most breathing-sensitive) ----
    mag_all = np.sqrt(d["i"]**2 + d["q"]**2)
    mag_cv  = mag_all.std(axis=0) / (mag_all.mean(axis=0) + 1)  # CV per bin
    top_bin = int(np.argmax(mag_cv))
    print(f"\n Most sensitive bin: {top_bin}  (mag CV={mag_cv[top_bin]:.3f})")

    # ---- phase extraction ----
    ph = phase_of_bin(d, top_bin)
    freq, psd = fft_breathing(ph)

    if len(freq) == 0:
        print("\n[!] Not enough frames for FFT (need 60+). Record at least 2 min.")
        return

    # find peaks in breathing band
    mask = (freq >= 0.10) & (freq <= 1.0)
    f_band = freq[mask]
    p_band = psd[mask]
    if len(f_band) == 0:
        print("\n[!] No frequency bins in 0.1-1 Hz band.")
        return

    pk_idx = np.argmax(p_band)
    pk_freq = f_band[pk_idx]
    pk_power = p_band[pk_idx]

    # estimate noise floor in breathing band (median)
    noise_floor = np.median(p_band)
    snr = pk_power / (noise_floor + 1e-9)

    print(f"\n--- Breathing Band (0.1-1.0 Hz) ---")
    print(f" Peak frequency: {pk_freq:.3f} Hz  ({pk_freq*60:.1f} bpm)")
    print(f" Peak power:     {pk_power:.1e}")
    print(f" Noise floor:    {noise_floor:.1e}")
    print(f" SNR:            {snr:.1f}x")

    if snr > 3.0:
        print(f"\n >>> BREATHING DETECTED <<<")
        print(f" Estimated rate: {pk_freq*60:.0f} breaths/min")
    elif snr > 1.5:
        print(f"\n Marginal — try longer recording (3-5 min)")
    else:
        print(f"\n No clear breathing peak (SNR < 1.5)")
        print(f" Ensure person is stationary between TX/RX antennas.")
        print(f" Record 3-5 minutes for reliable FFT.")

    # ---- compare with empty if available ----
    if len(datasets) > 1:
        de = datasets[1]
        ph_e = phase_of_bin(de, top_bin)
        fe, pe = fft_breathing(ph_e)
        if len(fe) > 0:
            me = (fe >= 0.10) & (fe <= 1.0)
            pi_e = np.argmax(pe[me]) if np.any(me) else 0
            print(f"\n--- Comparison with {de['fname']} ---")
            print(f" {d['label']}  peak: {pk_freq:.3f} Hz  power={pk_power:.1e}")
            print(f" {de['label']} peak: {fe[me][pi_e]:.3f} Hz  power={pe[me][pi_e]:.1e}")
            ratio = pk_power / (pe[me][pi_e] + 1e-9)
            print(f" Power ratio (human/empty): {ratio:.1f}x")
            if ratio > 3:
                print(f" Strong breathing signal relative to empty scene")

    # print top 5 peaks
    top5 = np.argsort(p_band)[-5:][::-1]
    print(f"\n--- Top 5 frequency peaks (0.1-1 Hz) ---")
    for rank, idx in enumerate(top5):
        print(f"  {f_band[idx]:.3f} Hz  ({f_band[idx]*60:.1f} bpm)  power={p_band[idx]:.1e}")

    print()


if __name__ == "__main__":
    main()
