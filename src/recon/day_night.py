"""The same object orbited by day and by night: accumulated radar points on the LiDAR, side by side.

    python -m src.recon.day_night --day data/raw/2026-11-03.17-01-12 30 165 95 --night data/raw/2026-11-19.17-55-35 10 143 70 \
        --centre-day -13.17 -9.88 --centre-night -2.76 0.08 --out figures/day_night_scotty.png

Each orbit is given as trace, start, end (s from the first camera frame) and the time of the camera frame
to show. Reads data/recon/<trace>/ (written by src.recon.accumulate over the same window) and the poses.
Nothing in the radar chain or the odometry uses the camera, so the light should change only the photograph.
Numbers per orbit, as in accumulate: share of detections kept by the static-world test, share of the kept
points within 0.5 m of the LiDAR layer (xy), coverage of the LiDAR cells; plus the orbit's radius and speed.
"""
import argparse, os

import numpy as np
from scipy.spatial import cKDTree

from .closeup import camera_frame


def orbit_numbers(trace, t0, t1, centre, near=15.0):
    name = os.path.basename(trace.rstrip("/")); ts0 = np.fromfile(os.path.join(trace, "camera", "ts"), "<f8")[0]
    r = np.load(f"data/recon/{name}/radar_points.npz"); L = np.load(f"data/recon/{name}/lidar_points.npz")["xyz"]
    p = np.load(f"data/poses/{name}/radar.npz"); w = (p["t"] >= ts0 + t0) & (p["t"] <= ts0 + t1); path = p["T"][w, :2, 3]
    rad = np.hypot(*(path - centre).T); speed = np.linalg.norm(np.diff(path, axis=0), axis=1).sum() / (t1 - t0)
    xy, kept = r["xyz"][:, :2], r["static"].astype(bool)
    close = cKDTree(path).query(xy)[0] < near
    d = cKDTree(L[:, :2]).query(xy[close & kept])[0]
    return dict(n=int(len(xy)), kept=float(kept.mean()), within=float((d < 0.5).mean()), radius=float(np.median(rad)),
                speed=float(speed), xy=xy, keep=kept, snr=r["snr"], lidar=L, path=path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--day", nargs=4, required=True, metavar=("TRACE", "T0", "T1", "T_IMG"))
    ap.add_argument("--night", nargs=4, required=True, metavar=("TRACE", "T0", "T1", "T_IMG"))
    ap.add_argument("--centre-day", type=float, nargs=2, required=True); ap.add_argument("--centre-night", type=float, nargs=2, required=True)
    ap.add_argument("--half", type=float, default=7.0, help="half-width of the top view around the object (m)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    cols = [("day", a.day, np.array(a.centre_day)), ("night", a.night, np.array(a.centre_night))]
    fig, ax = plt.subplots(2, 2, figsize=(9.2, 7.0), gridspec_kw=dict(height_ratios=[9 / 16 * 1.0, 1.0]), constrained_layout=True)
    for j, (label, (tr, t0, t1, ti), c) in enumerate(cols):
        t0, t1, ti = float(t0), float(t1), float(ti); name = os.path.basename(tr.rstrip("/"))
        o = orbit_numbers(tr, t0, t1, c)
        ts0 = np.fromfile(os.path.join(tr, "camera", "ts"), "<f8")[0]
        img, _ = camera_frame(tr, ts0 + ti, a.out + f"._{label}.png")
        ax[0, j].imshow(img); ax[0, j].set_title(f"{label}, {name[5:10].replace('-', '/')}", fontsize=10, fontweight="bold")
        A = ax[1, j]; L = o["lidar"][:, :2] - c; m = (np.abs(L[:, 0]) < a.half) & (np.abs(L[:, 1]) < a.half)
        A.scatter(L[m, 0], L[m, 1], s=0.2, c="0.8", linewidths=0, rasterized=True)
        P = o["xy"][o["keep"]] - c; s = o["snr"][o["keep"]]; k = (np.abs(P[:, 0]) < a.half) & (np.abs(P[:, 1]) < a.half)
        im = A.scatter(P[k, 0], P[k, 1], s=1.0, c=s[k], cmap="magma", vmin=10, vmax=35, linewidths=0, rasterized=True)
        A.plot(*(o["path"] - c).T, color="#1f9fff", lw=0.8)
        A.set_xlim(-a.half, a.half); A.set_ylim(-a.half, a.half); A.set_aspect("equal")
        A.set_xlabel(f"{o['kept']:.0%} of {o['n']:,} detections kept · {o['within']:.0%} within 0.5 m of the LiDAR\n"
                     f"orbit radius {o['radius']:.1f} m · {o['speed']:.2f} m/s", fontsize=8.5)
        print(f"{label} {name} {t0:.0f}-{t1:.0f} s: {o['n']} detections, {o['kept']:.1%} kept, {o['within']:.1%} within 0.5 m, "
              f"orbit radius {o['radius']:.2f} m, speed {o['speed']:.2f} m/s")
    ax[0, 0].set_ylabel("camera", fontsize=10, fontweight="bold"); ax[1, 0].set_ylabel("radar on LiDAR, top view", fontsize=10, fontweight="bold")
    for A in ax.ravel():
        A.set_xticks([]); A.set_yticks([])
    fig.colorbar(im, ax=ax[1, :], location="right", shrink=0.8, label="radar SNR (dB); grey: LiDAR; blue: path")
    fig.savefig(a.out, dpi=150); print("saved", a.out)


if __name__ == "__main__":
    main()
