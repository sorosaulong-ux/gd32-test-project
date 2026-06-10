"""
train_model.py — 16帧(4s)窗口逻辑回归: 空车 vs 有人

22维特征 (无 bin 幅值均值 — 抗信号幅值漂移):
  f[0..3]:   4 全局均值  {fp_power, noise, mag_mean, mag_std}
  f[4..7]:   4 全局标准差
  f[8..14]:  7 bin 幅值标准差
  f[15..21]: 7 bin 相位循环方差

Usage: python train_model.py <empty.csv> <human.csv>
"""

import sys, csv, os
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, recall_score, confusion_matrix

WINDOW = 16
BINS = [2, 8, 25, 31, 63, 69, 103]
NF = 4 + 4 + len(BINS) + len(BINS)   # = 22

def load_raw(fp):
    rows = []
    with open(fp, encoding='utf-8', errors='replace') as f:
        for row in csv.reader(f):
            if row and row[0].lstrip('-').isdigit():
                vals = [float(v) for v in row[:-1]]
                if len(vals) >= 7: rows.append(vals[1:])
    data = np.array(rows, dtype=np.float64)
    nb = max(4, (data.shape[1] - 5) // 2)
    raw = np.zeros((data.shape[0], 4 + len(BINS)*2), dtype=np.float32)
    raw[:, 0] = data[:, 3]; raw[:, 1] = data[:, 4]
    for i in range(data.shape[0]):
        mg = np.sqrt(data[i, 5::2]**2 + data[i, 6::2]**2)[:nb]
        raw[i, 2] = np.mean(mg); raw[i, 3] = np.std(mg)
    for k, bj in enumerate(BINS):
        if bj < nb:
            raw[:, 4+k] = np.sqrt(data[:, 5+bj*2]**2 + data[:, 6+bj*2]**2)
            raw[:, 4+len(BINS)+k] = np.arctan2(data[:, 6+bj*2], data[:, 5+bj*2])
    return raw

def windowify(raw):
    n = raw.shape[0]
    windows = []
    for i in range(0, n - WINDOW + 1, WINDOW):
        w = raw[i:i+WINDOW, :]
        f = np.zeros(NF, dtype=np.float32)
        off = 0
        # 4 全局均值
        for k in range(4): f[off+k] = np.mean(w[:, k])
        off += 4
        # 4 全局标准差
        for k in range(4): f[off+k] = np.std(w[:, k])
        off += 4
        # 7 bin 幅值标准差
        for k in range(len(BINS)): f[off+k] = np.std(w[:, 4+k])
        off += len(BINS)
        # 7 bin 相位循环方差
        for k in range(len(BINS)):
            ph = w[:, 4+len(BINS)+k]
            mc, ms = np.mean(np.cos(ph)), np.mean(np.sin(ph))
            f[off+k] = 1.0 - np.sqrt(mc**2 + ms**2)
        windows.append(f)
    return np.array(windows, dtype=np.float32)

def main():
    if len(sys.argv) < 3:
        print("Usage: python train_model.py <empty.csv> <human.csv>")
        sys.exit(1)

    all_w, all_y = [], []
    for fp in sys.argv[1:]:
        lb = 0 if 'empty' in fp.lower() else 1
        windows = windowify(load_raw(fp))
        all_w.append(windows); all_y.append(np.full(windows.shape[0], lb, dtype=np.int32))
        print(f"  {windows.shape[0]:4d} windows  {os.path.basename(fp)}")

    X = np.vstack(all_w); y = np.hstack(all_y)
    X = np.nan_to_num(X, nan=0, posinf=0, neginf=0)

    sc = StandardScaler(); Xs = sc.fit_transform(X)
    Xtr, Xte, ytr, yte = train_test_split(Xs, y, test_size=0.2, random_state=42, stratify=y)
    m = LogisticRegression(C=1.0, max_iter=2000, solver='lbfgs')
    m.fit(Xtr, ytr); yp = m.predict(Xte)
    print(f"\n  W={WINDOW} ({WINDOW/4:.0f}s)  NF={NF}  Acc={accuracy_score(yte,yp)*100:.1f}%  Rec={recall_score(yte,yp,pos_label=1)*100:.1f}%")
    cm = confusion_matrix(yte, yp)
    print(f"  FP(empty->human)={cm[0,1]}  FN(human->empty)={cm[1,0]}")

    W, b = m.coef_.flatten(), m.intercept_[0]
    mu, sg = sc.mean_, sc.scale_

    out = os.path.join(os.path.dirname(__file__) or '.', 'model_weights.h')
    with open(out, 'w') as f:
        f.write(f"/* Window LR, W={WINDOW}fr ({WINDOW/4:.0f}s), NF={NF} (no bin means) */\n")
        f.write(f"#define ML_WINDOW_SIZE {WINDOW}\n#define ML_N_FEAT {NF}\n#define ML_N_BINS {len(BINS)}\n")
        f.write(f"static const int ml_bins[{len(BINS)}] = {{")
        f.write(','.join(str(b) for b in BINS)+'};\n\n')
        f.write(f"static const float ml_W[{NF}] = {{")
        for i, v in enumerate(W): f.write(f"{v:.8f}f"+(", " if i < NF-1 else ""))
        if (i+1) % 6 == 0: f.write("\n    ")
        f.write('\n};\n\n')
        f.write(f"static const float ml_b = {b:.8f}f;\n\n")
        for name, arr in [("ml_mu", mu), ("ml_sigma", sg)]:
            f.write(f"static const float {name}[{NF}] = {{")
            for i, v in enumerate(arr): f.write(f"{v:.6f}f"+(", " if i < NF-1 else ""))
            if (i+1) % 6 == 0: f.write("\n    ")
            f.write('\n};\n\n')
    print(f"  Exported: {out}")

if __name__ == "__main__":
    main()
