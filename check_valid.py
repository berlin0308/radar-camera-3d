"""Quick on-site check of one trace: duration per sensor and radar dropped-frame ratio.

Usage: python check_valid.py data/raw/<trace>
radar/valid holds one byte per frame; 0 = packets dropped, frame zero-filled.
"""
import json, lzma, os, sys
import numpy as np

t = sys.argv[1]
ok = True

def load_u8(p):
    raw = open(p, "rb").read()
    try:
        raw = lzma.decompress(raw)
    except lzma.LZMAError:
        pass
    return np.frombuffer(raw, dtype=np.uint8)

v = load_u8(os.path.join(t, "radar", "valid"))
bad = int((v == 0).sum())
ratio = bad / max(len(v), 1)
print(f"radar frames {len(v)}  dropped {bad} ({ratio:.1%})  ~{len(v)/20:.1f}s @20Hz")
if ratio > 0.05:
    print("!! >5% radar frames dropped: move / re-record")
    ok = False

for s in ["radar", "lidar", "camera", "imu"]:
    p = os.path.join(t, s, "ts")
    if not os.path.exists(p):
        print(f"!! {s}: no ts"); ok = False; continue
    try:
        ts = np.fromfile(p, dtype=np.float64)
        print(f"{s:7s} n={len(ts):6d}  span={ts[-1]-ts[0]:7.1f}s  rate={(len(ts)-1)/max(ts[-1]-ts[0],1e-9):6.1f} Hz")
    except Exception as e:
        print(f"{s}: could not read ts ({e})")

m = os.path.join(t, "radar", "radar.json")
if os.path.exists(m):
    print("radar.json present (copy chirp params into math-range table at home)")
sys.exit(0 if ok else 1)
