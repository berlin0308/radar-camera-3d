"""Radar points on a camera frame: the co-located guess next to the calibrated projection.

    python -m src.calib.project_image data/raw/2026-10-29.15-49-40 --time 720 --accumulate 10 --out figures/proj_jeep.png
"""
import argparse, json, os, subprocess

import numpy as np

from ..radar import RadarTrace, detect, to_xyz
from .board_image import project_fisheye, radar_to_cam_axes
from .transform import RadarToCamera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--time", type=float, required=True); ap.add_argument("--accumulate", type=int, default=10)
    ap.add_argument("--snr", type=float, default=12, help="keep points above this SNR (dB)"); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    tr = RadarTrace(a.trace); C = RadarToCamera.load()
    cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8"); kc = int(a.time * 30); t0 = cam_ts[kc]; k0 = tr.index_at(t0)
    frame_png = os.path.join(os.path.dirname(a.out) or ".", "_frame.png")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kc / 30:.3f}", "-i", os.path.join(a.trace, "camera", "video.avi"), "-frames:v", "1", frame_png], check=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    img = plt.imread(frame_png); os.remove(frame_png)
    pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(k0, k0 + a.accumulate)])
    pts = pts[pts["snr"] > a.snr]
    xyz = to_xyz(pts); xyz[:, 2] = 0.0                                     # radar elevation is unusable: put points at radar height
    uv0 = project_fisheye(radar_to_cam_axes(xyz), C.cam); uv1 = C.pixels(xyz)
    inside = lambda uv: (uv[:, 0] >= 0) & (uv[:, 0] < 1920) & (uv[:, 1] >= 0) & (uv[:, 1] < 1080)
    print(f"{len(pts)} points over {a.accumulate} frames at t = {a.time:.0f} s; in frame: {inside(uv0).sum()} before, {inside(uv1).sum()} after")
    fig, ax = plt.subplots(1, 2, figsize=(16.64, 5.0), constrained_layout=True)
    for axx, uv, title in [(ax[0], uv0, "a · co-located guess (camera at the radar, axes aligned)"), (ax[1], uv1, "b · calibrated: RadarToCamera.pixels()")]:
        axx.imshow(img); m = inside(uv)
        sc = axx.scatter(uv[m, 0], uv[m, 1], s=36, c=pts["r"][m], cmap="turbo", vmin=0, vmax=20, edgecolors="white", linewidths=.5)
        axx.set_xlim(0, 1920); axx.set_ylim(1080, 0); axx.axis("off"); axx.set_title(title, fontsize=12)
    fig.colorbar(sc, ax=ax[1], shrink=.8, label="radar range (m)")
    fig.suptitle(f"{os.path.basename(a.trace.rstrip('/'))} t = {a.time:.0f} s: radar points (SNR > {a.snr:.0f} dB, {a.accumulate} frames, z set to 0) on the camera frame", fontsize=12)
    fig.savefig(a.out, dpi=120); print("saved", a.out)


if __name__ == "__main__":
    main()
