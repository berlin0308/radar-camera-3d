"""One camera frame, the same radar points, four calibrations side by side (the report's qualitative figure).

    python -m src.calib.compare data/raw/2026-10-29.15-49-40 --time 720 --accumulate 10 --out figures/calib_compare_jeep.png
"""
import argparse, json, os, subprocess

import numpy as np

from ..radar import RadarTrace, detect, to_xyz
from .board_image import project_fisheye, radar_to_cam_axes
from .transform import RadarToCamera, RadarToLidar


def calibrations():
    """name -> function (N,3) radar xyz -> (N,2) pixels."""
    cam = RadarToCamera.load().cam
    direct = RadarToCamera.load()
    lc = json.load(open("calib/extrinsics_lidar_camera.json")); T = RadarToLidar.load()
    R_lc, t_lc = np.array(lc["R"]), np.array(lc["t_m"])
    composed = RadarToCamera(R_lc @ T.R, R_lc @ T.t + t_lc, cam)
    joint = RadarToCamera.load_joint()
    return [("a · none: camera at the radar, axes aligned", lambda x: project_fisheye(radar_to_cam_axes(x), cam)),
            ("b · pairwise PnP radar→camera (35 px)", direct.pixels),
            ("c · composed: Kabsch radar→LiDAR, PnP LiDAR→camera", composed.pixels),
            ("d · joint solve in the LiDAR frame", joint.pixels)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--time", type=float, required=True); ap.add_argument("--accumulate", type=int, default=10)
    ap.add_argument("--snr", type=float, default=12); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    tr = RadarTrace(a.trace)
    cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8"); kc = int(a.time * 30); k0 = tr.index_at(cam_ts[kc])
    frame_png = os.path.join(os.path.dirname(a.out) or ".", "_frame.png")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kc / 30:.3f}", "-i", os.path.join(a.trace, "camera", "video.avi"), "-frames:v", "1", frame_png], check=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    img = plt.imread(frame_png); os.remove(frame_png)
    pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(k0, k0 + a.accumulate)]); pts = pts[pts["snr"] > a.snr]
    xyz = to_xyz(pts); xyz[:, 2] = 0.0
    fig, axes = plt.subplots(2, 2, figsize=(16.64, 9.6), constrained_layout=True)
    for ax, (title, f) in zip(axes.flat, calibrations()):
        uv = f(xyz); m = (uv[:, 0] >= 0) & (uv[:, 0] < 1920) & (uv[:, 1] >= 0) & (uv[:, 1] < 1080)
        ax.imshow(img); sc = ax.scatter(uv[m, 0], uv[m, 1], s=40, c=pts["r"][m], cmap="turbo", vmin=0, vmax=20, edgecolors="white", linewidths=.5)
        ax.set_xlim(0, 1920); ax.set_ylim(1080, 0); ax.axis("off"); ax.set_title(title, fontsize=13)
    fig.colorbar(sc, ax=axes.ravel().tolist(), shrink=.6, label="radar range (m)")
    fig.suptitle(f"{os.path.basename(a.trace.rstrip('/'))} t = {a.time:.0f} s · {len(pts)} radar points of {a.accumulate} frames (SNR > {a.snr:.0f} dB, drawn at radar height) under four calibrations", fontsize=13)
    fig.savefig(a.out, dpi=120); print("saved", a.out)


if __name__ == "__main__":
    main()
