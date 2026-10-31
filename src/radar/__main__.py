"""Run the chain on one frame and plot the intermediate products and the point cloud.

    python -m src.radar data/raw/2026-10-29.15-49-40 --frame 14356 --accumulate 20 --out figures/pc.png
    python -m src.radar data/raw/2026-10-29.15-49-40 --time 720     # seconds into the camera video
"""
import argparse, os

import numpy as np

from .adc import RadarTrace
from .pointcloud import detect, to_xyz
from .angle import AZIMUTHS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--frame", type=int)
    ap.add_argument("--time", type=float, help="seconds into camera/video.avi")
    ap.add_argument("--accumulate", type=int, default=1, help="frames to overlay")
    ap.add_argument("--pfa", type=float, default=1e-3)
    ap.add_argument("--out", default=None, help="PNG path; default figures/pc_<trace>_<frame>.png")
    a = ap.parse_args()

    tr = RadarTrace(a.trace)
    if a.time is not None:
        cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8")
        k0 = tr.index_at(cam_ts[int(a.time * 30)])
    else:
        k0 = a.frame if a.frame is not None else len(tr) // 2
    P = tr.params
    print(f"{os.path.basename(a.trace.rstrip('/'))}: {len(tr)} frames, dres {P.range_resolution:.4f} m, "
          f"vres {P.velocity_resolution:.4f} m/s, vmax ±{P.max_velocity:.2f} m/s, lambda {P.wavelength*1e3:.2f} mm")

    pts_all, info = [], None
    for k in range(k0, k0 + a.accumulate):
        pts, info = detect(tr.frame(k), P, pfa=a.pfa)
        pts_all.append(np.lib.recfunctions.append_fields(pts, "frame", np.full(len(pts), k - k0), usemask=False)
                       if False else np.rec.fromarrays([*(pts[n] for n in pts.dtype.names), np.full(len(pts), k - k0)],
                                                       names=[*pts.dtype.names, "frame"]))
    pts = np.concatenate(pts_all)
    xyz = to_xyz(pts)
    print(f"frame {k0}" + (f"..{k0 + a.accumulate - 1}" if a.accumulate > 1 else "") + f": {len(pts)} points")
    for p, q in sorted(zip(pts, xyz), key=lambda t: -t[0]["snr"])[:10]:
        print(f"  r {p['r']:5.2f} m  az {np.rad2deg(p['az']):+6.1f}°  el {np.rad2deg(p['el']):+6.1f}°  "
              f"v {p['v']:+5.2f} m/s  snr {p['snr']:4.1f} dB  xyz ({q[0]:+5.1f}, {q[1]:5.1f}, {q[2]:+5.1f})")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16, 5.2), constrained_layout=True)
    ax0 = fig.add_subplot(1, 3, 1); ax1 = fig.add_subplot(1, 3, 2); ax2 = fig.add_subplot(1, 3, 3, projection="3d")
    db = 10 * np.log10(info["power"] + 1e-9); db -= db.max()
    R, TH = np.meshgrid(P.ranges, AZIMUTHS)
    ax0.pcolormesh(-R * np.sin(TH), R * np.cos(TH), db, vmin=-35, vmax=0, cmap="magma", shading="auto")
    one = pts["frame"] == a.accumulate - 1
    ax0.scatter(xyz[one, 0], xyz[one, 1], s=28, facecolors="none", edgecolors="#4FC3F7", lw=1.2)
    ax0.set_xlim(-20, 20); ax0.set_ylim(0, 20); ax0.set_aspect("equal"); ax0.set_title(f"range-azimuth + CFAR, frame {k0 + a.accumulate - 1}")
    ax0.set_xlabel("x right (m)"); ax0.set_ylabel("y forward (m)")
    sc = ax1.scatter(xyz[:, 0], xyz[:, 1], c=pts["snr"], cmap="magma", s=12, vmin=5, vmax=30)
    ax1.set_xlim(-15, 15); ax1.set_ylim(0, 20); ax1.set_aspect("equal"); ax1.set_title(f"top view, {a.accumulate} frame(s), colour = SNR dB")
    fig.colorbar(sc, ax=ax1, shrink=.8)
    ax2.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=pts["frame"], cmap="viridis", s=10)
    ax2.set_xlim(-10, 10); ax2.set_ylim(0, 20); ax2.set_zlim(-3, 3); ax2.view_init(elev=28, azim=-60)
    ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_zlabel("z"); ax2.set_title(f"3D points ({len(pts)})")
    out = a.out or f"figures/pc_{os.path.basename(a.trace.rstrip('/'))}_{k0}.png"
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True); fig.savefig(out, dpi=130)
    print("saved", out)


if __name__ == "__main__":
    main()
