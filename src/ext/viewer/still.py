"""A still of what the viewer shows: the same clouds, one panel each, plus the overlay.

    .venv/bin/python -m src.ext.viewer.still --out out/scotty_models.png

Same frame, same height colour map as the viewer, so the figure and the page read alike. It is the
fallback for a machine with no browser, and the figure that goes in the experiment write-up.
"""
import argparse, os

import numpy as np

from ..obj_rgb.align_to_lidar import read_ply, voxel

# name, path, colour, alpha in the overlay panels. GRT is drawn faint because it is a fog that fills the
# whole cut-out: at any usable alpha it hides everything behind it, which is itself the result.
DEFAULT = [("radar · GRT", "out/scotty_radar.ply", "#E8534E", 0.05),
           ("radar · CFAR", "out/scotty_radar_cfar.ply", "#F2B134", 0.75),
           ("LiDAR", "out/scotty_lidar.ply", "#5B8DEF", 0.35),
           ("RGB · DA3, 96 frames", "out/da3_posed_n96_r504/object.ply", "#3FBF7F", 0.35),
           ("RGB · DA3, pose-free", "out/da3_scotty/da3_aligned.ply", "#8FD3A8", 0.35)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[], metavar="NAME=PATH[=COLOUR]")
    ap.add_argument("--radius", type=float, default=1.2)
    ap.add_argument("--height-max", type=float, default=2.2)
    ap.add_argument("--voxel", type=float, default=0.012)
    ap.add_argument("--azim", type=float, default=35.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    spec = ([tuple((m.split("=") + ["#999999"])[:3]) + (0.35,) for m in a.model] if a.model else DEFAULT)
    spec = [t for t in spec if os.path.isfile(t[1]) or print(f"  ! missing {t[1]}")]

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    clouds = []
    for name, path, colour, alpha in spec:
        xyz, _ = read_ply(path)
        m = (np.hypot(*xyz[:, :2].T) < a.radius) & (xyz[:, 2] > 0.03)
        clouds.append((name, voxel(xyz[m], a.voxel), colour, alpha))
        print(f"{name:14s} {len(clouds[-1][1]):8,} points, height to {clouds[-1][1][:, 2].max():.2f} m")

    n = len(clouds)
    fig = plt.figure(figsize=(4.15 * (n + 2), 5.3), constrained_layout=True)
    gs = fig.add_gridspec(1, n + 2)

    for i, (name, P, colour, _) in enumerate(clouds):
        ax = fig.add_subplot(gs[i], projection="3d")
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=.5, c=P[:, 2], cmap="turbo", vmin=0, vmax=a.height_max,
                   linewidths=0, depthshade=False)
        ax.set_xlim(-a.radius, a.radius); ax.set_ylim(-a.radius, a.radius); ax.set_zlim(0, 2.6)
        ax.set_box_aspect((1, 1, 1.1)); ax.view_init(elev=14, azim=a.azim)
        ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("h (m)", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.set_title(f"{chr(97 + i)} · {name}\n{len(P):,} points, to {P[:, 2].max():.2f} m", fontsize=10)

    ax = fig.add_subplot(gs[n], projection="3d")
    for name, P, colour, al in sorted(clouds, key=lambda c: c[3]):          # faintest first, so it sits behind
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=.4, c=colour, linewidths=0, depthshade=False,
                   alpha=al, label=name)
    ax.set_xlim(-a.radius, a.radius); ax.set_ylim(-a.radius, a.radius); ax.set_zlim(0, 2.6)
    ax.set_box_aspect((1, 1, 1.1)); ax.view_init(elev=14, azim=a.azim)
    ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("h (m)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=8, markerscale=12, loc="upper left").set_alpha(1.0)
    for h in ax.get_legend().legend_handles: h.set_alpha(1.0)
    ax.set_title(f"{chr(97 + n)} · all {n}, overlaid\n(no fit between them: one object frame)", fontsize=10)

    ax = fig.add_subplot(gs[n + 1])
    for name, P, colour, al in sorted(clouds, key=lambda c: c[3]):
        ax.scatter(P[:, 0], P[:, 2], s=1.0, c=colour, linewidths=0, alpha=al, label=name)
    ax.set_aspect("equal"); ax.set_xlim(-a.radius, a.radius); ax.set_ylim(0, 2.6)
    ax.set_xlabel("x (m)"); ax.set_ylabel("height (m)"); ax.legend(fontsize=8, markerscale=5, loc="upper right")
    ax.set_title(f"{chr(98 + n)} · side view, overlaid", fontsize=10)

    fig.suptitle("Scotty, 2026-11-03.17-01-12, 30–165 s orbit · the same object from radar, LiDAR and RGB, "
                 "in one object frame", fontsize=12)
    fig.savefig(a.out, dpi=110)
    print("saved", a.out)


if __name__ == "__main__":
    main()
