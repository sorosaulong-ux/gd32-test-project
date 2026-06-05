"""cir_diff.py — Compare two CIR datasets"""
import sys, os, csv
import numpy as np

DIR = os.path.dirname(os.path.abspath(__file__))

if len(sys.argv) < 3:
    print("Usage: python cir_diff.py <empty.csv> <book.csv>")
    sys.exit(1)

def load(p):
    path = p if os.path.isabs(p) else os.path.join(DIR, p)
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr  = [c.strip() for c in rows[0]]
    data = rows[1:]
    ci   = [i for i, c in enumerate(hdr) if c.startswith("cir") and c[3:].isdigit()]
    nb   = len(ci)
    nf   = len(data)
    lbl  = data[0][-1].strip() if data else "?"
    cir  = np.zeros((nf, nb), np.float64)
    for i, r in enumerate(data):
        for j, c in enumerate(ci):
            if c < len(r):
                cir[i, j] = float(r[c])
    print(f"  load: {os.path.basename(path)} -> {nf}f x {nb}bins  label={lbl}")
    return cir, nb, nf, lbl

print()
ca, na, nfa, la = load(sys.argv[1])
cb, nb, nfb, lb = load(sys.argv[2])
n = min(na, nb)
ca, cb = ca[:, :n], cb[:, :n]

# noise floor per frame, baseline subtract
nf_a = ca[:, :20].mean(axis=1, keepdims=True)
nf_b = cb[:, :20].mean(axis=1, keepdims=True)
ba = np.maximum(ca - nf_a, 0)
bb = np.maximum(cb - nf_b, 0)
ma, mb = ba.mean(axis=0), bb.mean(axis=0)
sa, sb = ba.std(axis=0),  bb.std(axis=0)
diff = mb - ma
pool = np.sqrt(sa**2 + sb**2) + 1
z = np.abs(diff) / pool

pka, pkb = int(np.argmax(ma)), int(np.argmax(mb))
nsig = int(np.sum(z > 3))

print(f"\n{'='*55}")
print(f" A ({la}): {nfa}f  peak=bin{pka}  {ma[pka]:,.0f}")
print(f" B ({lb}): {nfb}f  peak=bin{pkb}  {mb[pkb]:,.0f}")
print(f" Shift: {pkb-pka:+d} bins")
print(f" Z>3: {nsig}/{n}  |  max_z={z.max():.1f} at bin {int(np.argmax(z))}")
print(f"{'='*55}")

if nsig > 0:
    print(f" >>> CHANGE DETECTED <<<")
    for idx in np.argsort(z)[-min(10, nsig):][::-1]:
        print(f"  bin{idx:4d} z={z[idx]:.1f}  A={ma[idx]:,.0f}  B={mb[idx]:,.0f}  delta={diff[idx]:+,.0f}")
else:
    print(f" Top 5 |delta|:")
    for idx in np.argsort(np.abs(diff))[-5:][::-1]:
        print(f"  bin{idx:4d} A={ma[idx]:,.0f}  B={mb[idx]:,.0f}  delta={diff[idx]:+,.0f}")
print()
