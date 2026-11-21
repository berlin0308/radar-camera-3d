"""CFAR and GRT scored the same way: on the same frames, against the same LiDAR label, from above.

    .venv/bin/python -m src.grt.vs_cfar /data/berlin/grt/2026-11-19.17-55-35 --trace data/raw/2026-11-19.17-55-35 --out figures/grt_vs_cfar_bev_scotty_1119.json

`align` scores GRT the way its paper does, as a depth map per (elevation, azimuth) column. CFAR
points have no usable elevation, so that score cannot be applied to them. Here both are reduced to
what both can give: points in the horizontal plane of the LiDAR frame. Per scored frame (every
`stride`th of the dump), three point sets:

    label  nrdk's LiDAR label, first occupied cell per column, above z_min (structures, not the ground)
    GRT    the model's first occupied cell per column at p > threshold, above z_min
    CFAR   our detections of the radar frame nearest in time, through the joint extrinsics, SNR > snr_min

and two medians of the nearest-neighbour distance in xy: method -> label (are the points on a
surface?) and label -> method (is every surface seen?). Also the points per frame. One frame of
CFAR against one GRT output: GRT sees a short history of frames, so this is close but not exact.
"""
import argparse, json, os

import numpy as np
from scipy.spatial import cKDTree

from .occupancy import GridGeometry
from .align import first_return
from ..radar import RadarTrace, detect, to_xyz
from ..calib.transform import RadarToLidar


def first_xyz(occ, G, rows, z_min):
    idx = first_return(occ[None])[0]; el, az = np.nonzero(idx[rows] >= 0); el = el + rows.start
    p = G.xyz(np.stack([el, az, idx[el, az]], axis=1)); return p[p[:, 2] > z_min]


def med_nn(a, b):
    return float(np.median(cKDTree(b[:, :2]).query(a[:, :2])[0])) if len(a) and len(b) else np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--trace", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5); ap.add_argument("--max-frames", type=int, default=300)
    ap.add_argument("--rows", default="4:44"); ap.add_argument("--z-min", type=float, default=-1.0, help="LiDAR frame (m); the ground is ~1.5 m down")
    ap.add_argument("--snr-min", type=float, default=12.0); ap.add_argument("--reach", type=float, default=15.0, help="score within this horizontal range (m)")
    a = ap.parse_args()
    r0, r1 = (int(x) for x in a.rows.split(":")); rows = slice(r0, r1)
    logit = np.load(os.path.join(a.dump, "occ3d_logit.npy"), mmap_mode="r"); lab = np.load(os.path.join(a.dump, "occ3d_label.npy"), mmap_mode="r")
    ts = np.load(os.path.join(a.dump, "ts.npy")); sel = np.arange(0, min(len(ts), a.max_frames * a.stride), a.stride)
    G = GridGeometry(a.trace); tr = RadarTrace(a.trace); T = RadarToLidar.load_joint(trace=a.trace)
    near = lambda p: p[np.hypot(p[:, 0], p[:, 1]) < a.reach]
    rows_out = {k: [] for k in ["cfar", "grt_0.5", "grt_0.9"]}
    for i in sel:
        L = near(first_xyz(np.asarray(lab[i]), G, rows, a.z_min))
        if len(L) < 20:
            continue
        prob = 1 / (1 + np.exp(-np.asarray(logit[i], dtype=np.float32)))
        k = tr.index_at(ts[i]); pts, _ = detect(tr.frame(k), tr.params); pts = pts[pts["snr"] > a.snr_min]
        c = to_xyz(pts); c[:, 2] = 0; c = T(c)
        sets = {"cfar": near(c), "grt_0.5": near(first_xyz(prob > 0.5, G, rows, a.z_min)), "grt_0.9": near(first_xyz(prob > 0.9, G, rows, a.z_min))}
        for n, P in sets.items():
            rows_out[n].append((med_nn(P, L), med_nn(L, P), len(P), len(L)))
    res = {}
    for n, v in rows_out.items():
        v = np.array(v, float)
        res[n] = dict(frames=int(len(v)), to_label_m=float(np.nanmedian(v[:, 0])), label_to_m=float(np.nanmedian(v[:, 1])),
                      points=float(np.median(v[:, 2])), label_points=float(np.median(v[:, 3])))
        print(f"{n:8s} {len(v)} frames: to label {res[n]['to_label_m']:.2f} m, label to it {res[n]['label_to_m']:.2f} m, {res[n]['points']:.0f} points/frame (label {res[n]['label_points']:.0f})")
    res["params"] = dict(dump=a.dump, trace=os.path.basename(a.trace.rstrip("/")), stride=a.stride, rows=[r0, r1], z_min=a.z_min, snr_min=a.snr_min, reach=a.reach)
    json.dump(res, open(a.out, "w"), indent=1); print("saved", a.out)


if __name__ == "__main__":
    main()
