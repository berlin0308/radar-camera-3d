"""The whole scene around the object, not just the cut-out, as a PLY for the viewer.

    python -m src.ext.obj_lidar.export_scene 2026-11-03.17-01-12 --centre -13.07 -9.86 \
        --source lidar --out out/scotty_lidar_scene.ply

`src.ext.obj_lidar.export_ply` deliberately keeps only a 1.2 m cylinder about the pedestal, because the
frame it writes is defined by a per-scan re-centring on that pedestal: each scan's points are shifted
by that scan's own centre correction, which takes the odometry's jitter out of the object and would put
it straight into anything further away. That frame is only meaningful near the object.

The background therefore has to come from the world-frame accumulation instead — `data/recon/<trace>/`,
written by `src.recon.accumulate` — shifted to the same origin (the refined pedestal centre) with no
per-scan correction at all. It is the same scene, built the same way the accumulation figures are, so
it lines up with the cut-out to the odometry's own accuracy rather than exactly.

One limit worth knowing: `accumulate` keeps only the layer 0.3–2.5 m above the ground and within 15 m
of the path, because that is roughly what the radar can see. So the LiDAR scene here is a slab: no
ground, no roofline. Widen it in `accumulate`, not here.
"""
import argparse, os

import numpy as np

from .export_ply import voxel_mean, write_ply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", help="trace name, e.g. 2026-11-03.17-01-12")
    ap.add_argument("--centre", type=float, nargs=2, required=True, help="refined pedestal centre, world xy")
    ap.add_argument("--source", choices=("lidar", "radar"), default="lidar")
    ap.add_argument("--recon-dir", default="data/recon")
    ap.add_argument("--radius", type=float, default=15.0)
    ap.add_argument("--static-only", action="store_true", help="--source radar: keep the static-filtered points")
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--height-max", type=float, default=2.5, help="top of the turbo colour map")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    f = os.path.join(a.recon_dir, a.trace, f"{a.source}_points.npz")
    d = np.load(f)
    xyz = d["xyz"]
    if a.source == "lidar":
        h = d["height"]
    else:
        if a.static_only:
            m = d["static"]
            xyz = xyz[m]
        h = np.full(len(xyz), 1.2)             # radar z is unusable; draw it at the radar's height
    print(f"{len(xyz):,} points from {f}")

    xy = xyz[:, :2] - np.array(a.centre)
    keep = np.hypot(*xy.T) < a.radius
    xy, h = xy[keep], h[keep]
    out = np.column_stack([xy, h])

    import matplotlib
    rgb = (matplotlib.colormaps["turbo"](np.clip(h / a.height_max, 0, 1))[:, :3] * 255).astype(np.uint8)
    if a.voxel > 0:
        out, rgb = voxel_mean(out, rgb.astype(float), a.voxel)

    write_ply(a.out, out, rgb)
    print(f"{int(keep.sum()):,} within {a.radius} m -> {len(out):,} after {a.voxel * 100:.0f} cm voxels; "
          f"extent x {out[:, 0].min():+.1f}..{out[:, 0].max():+.1f}, y {out[:, 1].min():+.1f}..{out[:, 1].max():+.1f}, "
          f"height {out[:, 2].min():.2f}..{out[:, 2].max():.2f} m")
    print("saved", a.out, f"({os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
