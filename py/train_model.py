"""
train_model.py — 滑动窗口逻辑回归: 空车 vs 有人

8帧窗口(2s) → 29维统计特征 → StandardScaler → LR → C 权重导出

特征(29维):
  f[0..7]:   mean/std of {fp_power, noise, mag_mean, mag_std}
  f[8..21]:  mean/std of 7 key bins' magnitude
  f[22..28]: circular variance of 7 key bins' phase

Usage:
    python train_model.py cir_empty.csv cir_human1.csv [cir_human2.csv ...]
"""

import sys, csv, os
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, recall_score, confusion_matrix

WINDOW = 8
BINS   = [2, 8, 25, 31, 63, 69, 103]
NF     = 4*2 + len(BINS)*2 + len(BINS)  # = 29
RFEAT  = 4 + len(BINS)*2               # = 18 瞬时特征

def load(filepath):
    rows = []
    with open(filepath, encoding='utf-8', errors='replace') as f:
        for row in csv.reader(f):
            if row and row[0].lstrip('-').isdigit():
                vals = [float(v) for v in row[:-1]]
                if len(vals) >= 7: rows.append(vals[1:])
    return np.array(rows, dtype=np.float64)

def extract_raw(frames):
    n = frames.shape[0]; nb = max(4, (frames.shape[1]-5)//2)
    raw = np.zeros((n, RFEAT), dtype=np.float32)
    raw[:,0] = frames[:,3]; raw[:,1] = frames[:,4]
    for i in range(n):
        mgs = [np.sqrt(frames[i,5+j*2]**2+frames[i,5+j*2+1]**2) for j in range(nb)
               if 5+j*2+1 < frames.shape[1]]
        mgs = np.array(mgs); raw[i,2]=np.mean(mgs); raw[i,3]=np.std(mgs)
    for k, bj in enumerate(BINS):
        if bj < nb:
            ic, qc = 5+bj*2, 5+bj*2+1
            if qc < frames.shape[1]:
                raw[:,4+k]   = np.sqrt(frames[:,ic]**2+frames[:,qc]**2)
                raw[:,11+k]  = np.arctan2(frames[:,qc], frames[:,ic])
    return raw

def windowify(raw):
    n = raw.shape[0]; out = np.zeros((n-WINDOW+1, NF), dtype=np.float32)
    for i in range(WINDOW-1, n):
        w = raw[i-WINDOW+1:i+1, :]
        for k in range(4): out[i-WINDOW+1,k*2]=np.mean(w[:,k]); out[i-WINDOW+1,k*2+1]=np.std(w[:,k])
        for k in range(len(BINS)):
            out[i-WINDOW+1,8+k*2]=np.mean(w[:,4+k]); out[i-WINDOW+1,8+k*2+1]=np.std(w[:,4+k])
            ph=w[:,11+k]; mc=np.mean(np.cos(ph)); ms=np.mean(np.sin(ph))
            out[i-WINDOW+1,22+k]=1.0-np.sqrt(mc**2+ms**2)
    return out

def main():
    if len(sys.argv) < 3:
        print("Usage: python train_model.py <empty.csv> <human1.csv> [human2.csv ...]")
        sys.exit(1)

    all_w, all_y = [], []
    for fp in sys.argv[1:]:
        lb = 0 if 'empty' in fp.lower() else 1
        w = windowify(extract_raw(load(fp)))
        all_w.append(w); all_y.append(np.full(w.shape[0], lb, dtype=np.int32))
        print(f"  {w.shape[0]:5d} win  {os.path.basename(fp)}")

    X = np.vstack(all_w); y = np.hstack(all_y)
    X = np.nan_to_num(X, nan=0, posinf=0, neginf=0)

    sc = StandardScaler(); Xs = sc.fit_transform(X)
    Xtr, Xte, ytr, yte = train_test_split(Xs, y, test_size=0.2, random_state=42, stratify=y)
    m = LogisticRegression(C=1.0, max_iter=2000, solver='lbfgs')
    m.fit(Xtr, ytr); yp = m.predict(Xte)
    acc = accuracy_score(yte, yp); rec = recall_score(yte, yp, pos_label=1)
    cm = confusion_matrix(yte, yp)
    print(f"\n  W={WINDOW} NF={NF}  acc={acc*100:.1f}%  rec={rec*100:.1f}%  FP={cm[0,1]}  FN={cm[1,0]}")

    W, b = m.coef_.flatten(), m.intercept_[0]
    mu, sg = sc.mean_, sc.scale_

    out = os.path.join(os.path.dirname(__file__) or '.', 'model_weights.h')
    with open(out, 'w') as f:
        f.write(f"/* Sliding window LR — W={WINDOW}, NF={NF}, {len(sys.argv)-1} datasets */\n")
        f.write(f"#define ML_WINDOW_SIZE {WINDOW}\n#define ML_N_FEAT {NF}\n#define ML_N_BINS {len(BINS)}\n")
        f.write(f"static const int ml_bins[{len(BINS)}] = {{")
        f.write(','.join(str(x) for x in BINS)+'};\n\n')
        f.write(f"static const float ml_W[{NF}] = {{")
        for i, v in enumerate(W): f.write(f"{v:.8f}f"+(", " if i<NF-1 else ""))
        if (i+1) % 6 == 0: f.write("\n    ")
        f.write('\n};\n\n')
        f.write(f"static const float ml_b = {b:.8f}f;\n\n")
        for name, arr in [("ml_mu", mu), ("ml_sigma", sg)]:
            f.write(f"static const float {name}[{NF}] = {{")
            for i, v in enumerate(arr): f.write(f"{v:.6f}f"+(", " if i<NF-1 else ""))
            if (i+1) % 6 == 0: f.write("\n    ")
            f.write('\n};\n\n')
    print(f"  Exported: {out}")

if __name__ == "__main__":
    main()
