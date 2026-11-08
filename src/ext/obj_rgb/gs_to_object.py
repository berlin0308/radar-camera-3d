"""Bring a splat's output into the object frame the other clouds live in, and write a PLY.

    .venv/bin/python -m src.ext.obj_rgb.gs_to_object /data/christineh/gs_scotty/exports/splat/splat.ply \
        --poses out/da3_scotty/poses96.npz --out out/scotty_gs.ply

`ns-export` writes in the world frame of `transforms.json`, which `export_nerfstudio` wrote in the
world frame of `data/poses/<trace>` — so this only has to shift to the pedestal and to the ground:
x, y about the refined centre, z above the ground. That ground is one number here, the median over the
orbit, because a splat is a single world model with no per-frame association; the LiDAR and DA3 clouds
take theirs per scan, and the orbit's ground wanders 0.15 m with the odometry, so expect that much
slack in the height.

Reads either export: `gaussian-splat` (the Gaussians' centres, with colour from the degree-0 spherical
harmonic) or `pointcloud` (points with plain red/green/blue). Opacity, where the file has it, filters
the floaters a splat leaves in empty space.
"""
import argparse, os

import numpy as np

SH_C0 = 0.28209479177387814            # Y00, the constant spherical harmonic


def read_ply_any(path):
    """Binary little-endian PLY with arbitrary properties -> dict of named columns."""
    with open(path, "rb") as f:
        props, n, fmt = [], 0, None
        while True:
            line = f.readline()
            s = line.decode("ascii", "replace").strip()
            if s.startswith("format"):
                fmt = s.split()[1]
            elif s.startswith("element vertex"):
                n = int(s.split()[-1])
            elif s.startswith("property") and "list" not in s:
                _, t, name = s.split()
                props.append((name, {"float": "<f4", "float32": "<f4", "double": "<f8",
                                     "uchar": "u1", "uint8": "u1", "int": "<i4",
                                     "short": "<i2", "ushort": "<u2"}[t]))
            elif s == "end_header":
                break
        if fmt != "binary_little_endian":
            raise SystemExit(f"{path}: only binary_little_endian is read here, got {fmt}")
        rec = np.frombuffer(f.read(n * np.dtype(props).itemsize), dtype=np.dtype(props), count=n)
    return {k: rec[k] for k, _ in props}, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("--poses", required=True, help="npz from src.ext.obj_rgb.export_poses")
    ap.add_argument("--radius", type=float, default=1.2)
    ap.add_argument("--z", type=float, nargs=2, default=(0.03, 3.0))
    ap.add_argument("--opacity", type=float, default=0.3, help="drop Gaussians below this opacity")
    ap.add_argument("--voxel", type=float, default=0.01)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    col, n = read_ply_any(a.ply)
    xyz = np.stack([col["x"], col["y"], col["z"]], -1).astype(np.float64)
    print(f"{n:,} points from {os.path.basename(a.ply)}")

    if "red" in col:
        rgb = np.stack([col["red"], col["green"], col["blue"]], -1).astype(np.uint8)
    elif "f_dc_0" in col:
        dc = np.stack([col["f_dc_0"], col["f_dc_1"], col["f_dc_2"]], -1).astype(np.float32)
        rgb = np.clip(0.5 + SH_C0 * dc, 0, 1)
        rgb = (rgb * 255).astype(np.uint8)
    else:
        rgb = np.full((n, 3), 200, np.uint8)

    keep = np.ones(n, bool)
    if "opacity" in col:
        alpha = 1.0 / (1.0 + np.exp(-col["opacity"].astype(np.float32)))     # stored as logit
        keep &= alpha > a.opacity
        print(f"opacity > {a.opacity}: {keep.sum():,} of {n:,} ({100 * keep.mean():.0f}%)")

    P = np.load(a.poses, allow_pickle=True)
    c_ref, ground = P["c_ref"], float(np.median(P["ground_frame"]))
    obj = np.column_stack([xyz[:, 0] - c_ref[0], xyz[:, 1] - c_ref[1], xyz[:, 2] - ground])
    print(f"object frame: centre {c_ref.round(2)}, ground z {ground:.3f} m "
          f"(the orbit's spread is {np.ptp(P['ground_frame']):.2f} m)")

    m = keep & (np.hypot(*obj[:, :2].T) < a.radius) & (obj[:, 2] > a.z[0]) & (obj[:, 2] < a.z[1])
    obj, rgb = obj[m], rgb[m]
    if not len(obj):
        raise SystemExit("nothing inside the cut-out; check --poses and the export's world frame")

    if a.voxel > 0:
        ij = np.floor(obj / a.voxel).astype(np.int64)
        _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
        q = np.stack([np.bincount(inv, obj[:, d]) / cnt for d in range(3)], -1)
        g = np.stack([np.bincount(inv, rgb[:, d].astype(float)) / cnt for d in range(3)], -1)
        obj, rgb = q, g.astype(np.uint8)

    from .align_to_lidar import write_ply
    write_ply(a.out, obj, rgb)
    print(f"{len(obj):,} points inside r < {a.radius} m, height {obj[:, 2].min():.2f}..{obj[:, 2].max():.2f} m")
    print("saved", a.out, f"({os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
