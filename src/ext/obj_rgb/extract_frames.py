"""Pull evenly spaced camera frames out of a trace, named so export_poses can time them.

    .venv/bin/python -m src.ext.obj_rgb.extract_frames data/raw/2026-11-03.17-01-12 \
        --orbit 30 165 --n 96 --out out/da3_scotty/frames96

Names are <i>_t<seconds>.png, the seconds being the frame's own camera timestamp relative to the
trace's first radar frame — not the time that was asked for. The camera starts about 2.3 s before the
radar, so an index computed from the nominal frame rate would be out by ~70 frames; everything is
matched on the one Unix clock instead.
"""
import argparse, os, subprocess

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--orbit", type=float, nargs=2, required=True)
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    t0 = np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8")[0]
    cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8")
    print(f"camera: {len(cam_ts)} frames, starts {cam_ts[0] - t0:+.2f} s relative to the radar, "
          f"{1 / np.median(np.diff(cam_ts)):.1f} fps")

    want = t0 + np.linspace(a.orbit[0], a.orbit[1], a.n)
    k = np.searchsorted(cam_ts, want).clip(0, len(cam_ts) - 1)
    k = np.where((k > 0) & (np.abs(cam_ts[np.maximum(k - 1, 0)] - want) < np.abs(cam_ts[k] - want)), k - 1, k)
    k = np.unique(k)
    print(f"{len(k)} distinct frames, {np.abs(cam_ts[k] - want[:len(k)]).max() * 1000:.0f} ms worst offset "
          f"from the times asked for")

    os.makedirs(a.out, exist_ok=True)
    video = os.path.join(a.trace, "camera", "video.avi")
    for i, kk in enumerate(k):
        sec = cam_ts[kk] - t0
        path = os.path.join(a.out, f"{i:03d}_t{sec:06.1f}.png")
        if os.path.exists(path):
            continue
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kk / a.fps:.3f}", "-i", video,
                        "-frames:v", "1", path], check=True)
    print(f"{len(os.listdir(a.out))} frames in {a.out}")


if __name__ == "__main__":
    main()
