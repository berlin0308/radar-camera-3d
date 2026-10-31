"""Sanity check of the extrinsics on any trace: radar points over the LiDAR scan of the same instant.

    python -m src.calib.overlay data/raw/2026-10-29.16-22-36 --time 200 --accumulate 10 --out figures/x.png
"""
import argparse, os

import numpy as np

from ..radar import RadarTrace, detect, to_xyz
from .lidar import LidarTrace
from .transform import RadarToLidar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--time", type=float, required=True, help="seconds into the camera video")
    ap.add_argument("--accumulate", type=int, default=10); ap.add_argument("--rmax", type=float, default=15)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    tr = RadarTrace(a.trace); lt = LidarTrace(a.trace); T = RadarToLidar.load()
    cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8"); t0 = cam_ts[int(a.time * 30)]
    k0 = tr.index_at(t0); kl = lt.index_at(t0)
    pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(k0, k0 + a.accumulate)])
    xyz_l = T(to_xyz(pts)); lid = lt.xyz(kl)
    print(f"radar frames {k0}..{k0 + a.accumulate - 1}, lidar scan {kl} ({lt.ts[kl] - tr.ts[k0]:+.2f} s), {len(pts)} radar points")

    # nearest LiDAR point (xy) for each radar point: the number that says whether the transform holds
    keep = lid[:, 2] > -0.9                                                     # drop the ground
    d = np.array([np.hypot(*(lid[keep, :2] - p[:2]).T).min() for p in xyz_l])
    print(f"nearest LiDAR point (xy): median {np.median(d)*100:.0f} cm, within 0.5 m: {(d < 0.5).mean()*100:.0f}% of radar points")

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(16.64, 7.4), constrained_layout=True)
    for axx, R in zip(ax, (a.rmax, 6)):
        axx.scatter(lid[keep, 0], lid[keep, 1], s=.5, c="#B9C2CC", label="LiDAR (above ground)")
        sc = axx.scatter(xyz_l[:, 0], xyz_l[:, 1], s=22, c=pts["snr"], cmap="magma", vmin=5, vmax=30, label="radar, transformed")
        axx.scatter(0, 0, marker="^", s=100, color="#15202B"); axx.set_xlim(-R, R); axx.set_ylim(-R, R); axx.set_aspect("equal")
        axx.set_xlabel("x lidar (m)"); axx.set_ylabel("y lidar (m)")
    ax[0].set_title(f"{os.path.basename(a.trace.rstrip('/'))} t = {a.time:.0f} s: radar points in the LiDAR frame, {a.accumulate} frames")
    ax[1].set_title(f"zoom; nearest LiDAR point median {np.median(d)*100:.0f} cm, {(d < 0.5).mean()*100:.0f}% within 0.5 m")
    ax[0].legend(loc="upper left", fontsize=10); fig.colorbar(sc, ax=ax[1], shrink=.7, label="radar SNR (dB)")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=120); print("saved", a.out)


if __name__ == "__main__":
    main()
