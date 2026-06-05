"""
cir_phase.py — Phase-variance CIR analysis for object detection

Loads I/Q CSV, computes per-bin phase stability across frames.
Stationary scene → stable phase.  Object present → phase jitter.

Usage:
    python cir_phase.py empty.csv metal.csv
    python cir_phase.py                     # latest empty + metal
"""

import sys, os, glob, csv
import numpy as np

DIR = os.path.dirname(os.path.abspath(__file__))


def load_iq(path):
    """Return dict: i(N,bins), q(N,bins), label, fname, n_frames, n_bins"""
    with open(path) as f:
        raw = list(csv.reader(f))

    hdr  = [c.strip() for c in raw[0]]
    data = raw[1:]
    lbl  = data[0][-1].strip() if data else "?"

    # find all 'iN' columns
    i_cols = []
    for idx, name in enumerate(hdr):
        if name.startswith("i") and name[1:].isdigit():
            i_cols.append((int(name[1:]), idx))
    i_cols.sort()  # by bin number
    nb = len(i_cols)
    if nb == 0:
        raise ValueError(f"No iN columns found.  Header: {hdr[:10]}...")

    nf = len(data)
    I  = np.zeros((nf, nb), dtype=np.float64)
    Q  = np.zeros((nf, nb), dtype=np.float64)

    for fi, row in enumerate(data):
        for bj, (_, ci) in enumerate(i_cols):
            qi = ci + 1  # q column is right after i
            if ci < len(row):
                I[fi, bj] = float(row[ci])
            if qi < len(row):
                Q[fi, bj] = float(row[qi])

    print(f"  load: {os.path.basename(path)} -> {nf}f x {nb}bins I/Q  label={lbl}")
    return dict(i=I, q=Q, n_frames=nf, n_bins=nb, label=lbl, fname=os.path.basename(path))


def phase_stability(d):
    """Compute per-bin phase variance and other metrics."""
    I = d["i"]
    Q = d["q"]
    nb = d["n_bins"]

    # per-sample phase: atan2(Q, I) in radians
    phase = np.arctan2(Q, I)

    # per-bin phase circular variance:  1 - |mean(e^{j*phase})|
    # 0 = perfectly stable, 1 = completely random
    complex_sum = np.sum(np.exp(1j * phase), axis=0)
    circ_var = 1.0 - np.abs(complex_sum) / d["n_frames"]

    # also compute simple std for comparison
    phase_std = np.std(phase, axis=0)   # linear std (degrades near ±π)

    # magnitude variance per bin
    mag = np.sqrt(I**2 + Q**2)
    mag_mean = mag.mean(axis=0)
    mag_cv   = mag.std(axis=0) / (mag_mean + 1)   # coefficient of variation

    # peak bin
    pk = int(np.argmax(mag_mean))

    return dict(
        phase=phase, circ_var=circ_var, phase_std=phase_std,
        mag=mag, mag_mean=mag_mean, mag_cv=mag_cv,
        peak_bin=pk, n_bins=nb, n_frames=d["n_frames"],
        label=d["label"], fname=d["fname"]
    )


def main():
    args = sys.argv[1:]
    fa, fb = None, None
    for a in args:
        if a.endswith(".csv"):
            if fa is None: fa = a
            else:          fb = a

    if not fa or not fb:
        # auto: latest empty + latest hand/metal
        empties = sorted(glob.glob(os.path.join(DIR, "cir_empty_*.csv")))
        others  = sorted(glob.glob(os.path.join(DIR, "cir_hand_*.csv")))
        if not others:
            others = sorted(glob.glob(os.path.join(DIR, "cir_metal_*.csv")))
        if empties and others:
            fa, fb = empties[-1], others[-1]
            print(f"Auto: empty={os.path.basename(fa)}  other={os.path.basename(fb)}")
        else:
            print("Usage: python cir_phase.py <empty.csv> <test.csv>")
            return

    if not os.path.isabs(fa): fa = os.path.join(DIR, fa)
    if not os.path.isabs(fb): fb = os.path.join(DIR, fb)

    da = load_iq(fa)
    db = load_iq(fb)

    sa = phase_stability(da)
    sb = phase_stability(db)

    ca = sa["circ_var"]
    cb = sb["circ_var"]
    nb = min(sa["n_bins"], sb["n_bins"])
    ca, cb = ca[:nb], cb[:nb]

    print(f"\n{'='*60}")
    print(f" Empty ({sa['fname']}): {sa['n_frames']}f  peak=bin{sa['peak_bin']}")
    print(f" Test  ({sb['fname']}): {sb['n_frames']}f  peak=bin{sb['peak_bin']}")
    print(f" Phase comparison — {nb} bins")
    print(f"{'='*60}")

    # circular variance summary
    print(f"\n--- Circular Phase Variance (0=stable, 1=random) ---")
    print(f" Empty  mean={ca.mean():.4f}  max={ca.max():.4f}")
    print(f" Test   mean={cb.mean():.4f}  max={cb.max():.4f}")
    print(f" Delta  mean={(cb-ca).mean():+.4f}  max={(cb-ca).max():+.4f}")

    # top bins where phase variance increased the most
    delta_v = cb - ca
    top = np.argsort(delta_v)[-10:][::-1]
    print(f"\n--- Top 10 bins where phase variance INCREASED ---")
    for b in top:
        print(f"  bin{b:4d}  empty={ca[b]:.4f}  test={cb[b]:.4f}  delta={delta_v[b]:+.4f}")

    # top bins where phase variance decreased (object absorbs that path)
    bot = np.argsort(delta_v)[:10]
    print(f"\n--- Top 10 bins where phase variance DECREASED ---")
    for b in bot:
        print(f"  bin{b:4d}  empty={ca[b]:.4f}  test={cb[b]:.4f}  delta={delta_v[b]:+.4f}")

    # z-score of delta_v across bins
    all_delta = cb - ca
    z = all_delta / (np.std(all_delta) + 1e-9)
    nsig = int(np.sum(np.abs(z) > 3))
    print(f"\n--- Detection ---")
    print(f"  z>3 bins: {nsig}/{nb}")
    print(f"  max |z|: {np.abs(z).max():.1f}")
    if nsig > 3:
        print(f"  >>> PHASE CHANGE DETECTED <<<")
    elif nsig > 0:
        print(f"  Marginal — collect more frames")
    else:
        print(f"  No significant phase change between scenes")

    # magnitude CV comparison
    print(f"\n--- Magnitude CV (std/mean, lower=more stable) ---")
    print(f" Empty  mean CV={sa['mag_cv'][:nb].mean():.4f}")
    print(f" Test   mean CV={sb['mag_cv'][:nb].mean():.4f}")

    print()


if __name__ == "__main__":
    main()
