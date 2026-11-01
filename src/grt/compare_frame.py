"""One instant, four views in the LiDAR frame: camera, our CFAR points on the LiDAR scan, GRT occupancy, nrdk's LiDAR label.

    .venv/bin/python -m src.grt.compare_frame /data/berlin/grt/2026-10-29.16-22-36 --trace data/raw/2026-10-29.16-22-36 --time 200 --out figures/grt_vs_cfar_scotty.png
"""
import argparse, os, subprocess

import numpy as np

from ..radar import RadarTrace, detect, to_xyz
from ..calib.lidar import LidarTrace
from ..calib.transform import RadarToLidar
from .occupancy import GridGeometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--trace", required=True); ap.add_argument("--time", type=float, required=True, help="seconds into the camera video")
    ap.add_argument("--accumulate", type=int, default=10); ap.add_argument("--threshold", type=float, default=0.5); ap.add_argument("--z-min", type=float, default=-0.8)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ts_dump = np.load(os.path.join(a.dump, "ts.npy")); logit = np.load(os.path.join(a.dump, "occ3d_logit.npy"), mmap_mode="r"); lab = np.load(os.path.join(a.dump, "occ3d_label.npy"), mmap_mode="r")
    tr = RadarTrace(a.trace); lt = LidarTrace(a.trace); T = RadarToLidar.load_joint(); G = GridGeometry(a.trace)
    cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8"); kc = int(a.time * 30); t0 = cam_ts[kc]
    kd = int(np.argmin(np.abs(ts_dump - t0))); kr = tr.index_at(t0); kl = lt.index_at(t0)
    print(f"camera t={a.time:.0f}s -> dump frame {kd} ({ts_dump[kd]-t0:+.2f}s), radar {kr}, lidar {kl}")
    png = os.path.join(os.path.dirname(a.out) or ".", "_cf.png")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kc / 30:.3f}", "-i", os.path.join(a.trace, "camera", "video.avi"), "-frames:v", "1", "-vf", "scale=960:-1", png], check=True)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    img = plt.imread(png); os.remove(png)

    pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(kr, kr + a.accumulate)]); pts = pts[pts["snr"] > 12]
    xyz_c = T(to_xyz(pts)); lid = lt.xyz(kl); lid = lid[lid[:, 2] > -0.9]
    prob = 1 / (1 + np.exp(-np.asarray(logit[kd], dtype=np.float32)))
    occ_p = prob > a.threshold; occ_l = np.asarray(lab[kd])
    def first_return_xyz(occ):                                             # first occupied cell per (el, az) column -> xyz, above ground
        any_ = occ.any(axis=-1); k = occ.argmax(axis=-1); el, az = np.nonzero(any_)
        xyz = G.xyz(np.stack([el, az, k[el, az]], axis=1)); return xyz[xyz[:, 2] > a.z_min], prob[el, az, k[el, az]][xyz[:, 2] > a.z_min]
    xyz_p, p_p = first_return_xyz(occ_p); xyz_l, _ = first_return_xyz(occ_l)

    fig, ax = plt.subplots(1, 4, figsize=(16.64, 4.9), constrained_layout=True, gridspec_kw=dict(width_ratios=[1.3, 1, 1, 1]))
    ax[0].imshow(img); ax[0].axis("off"); ax[0].set_title(f"a · camera, t = {a.time:.0f} s", fontsize=11)
    ax[1].scatter(-lid[:, 1], -lid[:, 0], s=.4, c="#B9C2CC"); sc = ax[1].scatter(-xyz_c[:, 1], -xyz_c[:, 0], s=18, c=pts["snr"], cmap="magma", vmin=5, vmax=30)
    ax[1].set_title(f"b · CFAR points ({a.accumulate} frames) on the LiDAR scan", fontsize=11)
    ax[2].scatter(-lid[:, 1], -lid[:, 0], s=.4, c="#B9C2CC"); ax[2].scatter(-xyz_p[:, 1], -xyz_p[:, 0], s=10, c=p_p, cmap="magma", vmin=0.5, vmax=1)
    ax[2].set_title(f"c · GRT first-return cells (p > {a.threshold}) on the LiDAR scan", fontsize=11)
    ax[3].scatter(-lid[:, 1], -lid[:, 0], s=.4, c="#B9C2CC"); ax[3].scatter(-xyz_l[:, 1], -xyz_l[:, 0], s=10, c="#E07B39")
    ax[3].set_title("d · nrdk LiDAR label, first-return cells, same frame", fontsize=11)
    for axx in ax[1:]:
        axx.set_aspect("equal"); axx.set_xlim(-15, 15); axx.set_ylim(0, 20); axx.set_xlabel("right (m)")
    ax[1].set_ylabel("forward (m)")
    fig.suptitle(f"{os.path.basename(a.trace.rstrip('/'))} t = {a.time:.0f} s · LiDAR frame, forward up · CFAR uses our joint calibration, GRT needs none", fontsize=12)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=130); print("saved", a.out)


if __name__ == "__main__":
    main()
