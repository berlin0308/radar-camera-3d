"""Several scenes, radar points on the camera frame, no calibration vs the joint solve (report examples).

    python -m src.calib.examples --out figures/calib_examples.png
"""
import argparse, os, subprocess

import numpy as np

from ..radar import RadarTrace, detect, to_xyz
from .board_image import project_fisheye, radar_to_cam_axes
from .transform import RadarToCamera

# (label, trace, seconds into the camera video, frames to accumulate, SNR floor dB)
SCENES = [
    ("board 40° right, 8 ft (10/21 angle shot)", "2026-10-21.15-36-20", None, 20, 15),      # None = last 30 s of the trace
    ("board 20° left, 8 ft (10/21 angle shot)", "2026-10-21.15-30-22", 15, 20, 15),
    ("board 26° right, 2.5 m (10/21 calib_07)", "2026-10-21.15-56-29", 5, 20, 15),
    ("food truck (10/29)", "2026-10-29.16-27-55", 253, 5, 15),
    ("crosswalk, cars and people (10/29)", "2026-10-29.16-12-38", 175, 5, 15),
    ("Scotty and the wall behind (10/29)", "2026-10-29.16-22-36", 200, 5, 15),
]


def frame_and_points(trace, t_sec, n, snr):
    d = os.path.join("data/raw", trace); tr = RadarTrace(d)
    cam_ts = np.fromfile(os.path.join(d, "camera", "ts"), "<f8")
    if t_sec is None: t_sec = (cam_ts[-1] - cam_ts[0]) - 15.0
    kc = int(t_sec * 30); k0 = tr.index_at(cam_ts[kc])
    png = "/tmp/_example_frame.png"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kc / 30:.3f}", "-i", os.path.join(d, "camera", "video.avi"), "-frames:v", "1", "-vf", "scale=960:-1", png], check=True)
    import matplotlib.image as mpimg
    img = mpimg.imread(png); os.remove(png)
    pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(k0, k0 + n)]); pts = pts[pts["snr"] > snr]
    xyz = to_xyz(pts); xyz[:, 2] = 0.0
    return img, pts, xyz, t_sec


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default="figures/calib_examples.png")
    ap.add_argument("--pick", type=int, nargs="*", help="scene indices; default all"); ap.add_argument("--wide", action="store_true", help="scenes as columns (slide layout)")
    a = ap.parse_args()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    C = RadarToCamera.load_joint(); cam = C.cam
    scenes = [SCENES[i] for i in a.pick] if a.pick else SCENES
    if a.wide:
        fig, axes = plt.subplots(2, len(scenes), figsize=(16.64, 6.4), constrained_layout=True); rows = axes.T
    else:
        fig, axes = plt.subplots(len(scenes), 2, figsize=(16.64, 4.7 * len(scenes)), constrained_layout=True); rows = axes
    for row, (label, trace, t_sec, n, snr) in zip(rows, scenes):
        img, pts, xyz, t_used = frame_and_points(trace, t_sec, n, snr)
        for ax, uv, title in [(row[0], project_fisheye(radar_to_cam_axes(xyz), cam), "no calibration"), (row[1], C.pixels(xyz), "joint calibration")]:
            uv = uv / 2.0                                                         # frame is scaled to 960 wide
            m = (uv[:, 0] >= 0) & (uv[:, 0] < 960) & (uv[:, 1] >= 0) & (uv[:, 1] < 540)
            ax.imshow(img); sc = ax.scatter(uv[m, 0], uv[m, 1], s=46, c=pts["r"][m], cmap="turbo", vmin=0, vmax=20, edgecolors="white", linewidths=.6)
            ax.set_xlim(0, 960); ax.set_ylim(540, 0); ax.axis("off")
            ax.set_title(f"{label}\n{title}" if a.wide else f"{label} · {title}", fontsize=12)
        print(f"{label}: {trace} t={t_used:.0f}s, {len(pts)} points")
    fig.colorbar(sc, ax=axes.ravel().tolist(), shrink=.4, label="radar range (m)")
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=110); print("saved", a.out)


if __name__ == "__main__":
    main()
