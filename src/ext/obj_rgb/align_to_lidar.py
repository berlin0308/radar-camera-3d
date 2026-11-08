"""Fit a similarity (scale, yaw, translation) from the DA3 object cloud onto the LiDAR one.

    .venv/bin/python -m src.ext.obj_rgb.align_to_lidar \
        --da3 out/da3_scotty/object.ply --lidar out/scotty_lidar.ply --out-dir out/da3_scotty

Both clouds are of the same object and both are levelled and centred, but DA3 is pose-free: its metre
scale is only as good as whatever run_da3.py scaled it by, and its yaw about the object is arbitrary.
Solving for (scale, yaw, translation) against the LiDAR gives the scale DA3 was actually off by and,
with it fixed, how far the RGB shape sits from the LiDAR one.

The score is a **symmetric** trimmed Chamfer distance, not ICP's own fitness. Scaled ICP driven by
one-directional fitness collapses here: shrinking the DA3 cloud until it sits inside the LiDAR one
scores perfectly (x0.48, "median 1.2 cm", object height 0.65 m against the true 1.79 m). Scoring the
LiDAR-to-DA3 direction as well removes that optimum, because a shrunken cloud leaves the LiDAR's
upper half unexplained.

Rotation is yaw only: both clouds were levelled on their own data, so a free rotation only lets the
fit lean the statue over to hide a scale error.

Writes <out-dir>/align_to_lidar.json, da3_aligned.ply and da3_vs_lidar.png.
"""
import argparse, json, os

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree


def read_ply(path):
    """Binary little-endian PLY with x,y,z[,red,green,blue] -> (xyz, rgb)."""
    with open(path, "rb") as f:
        hdr, n, props = b"", 0, []
        while True:
            line = f.readline(); hdr += line
            s = line.decode().strip()
            if s.startswith("element vertex"): n = int(s.split()[-1])
            elif s.startswith("property"): props.append(s.split()[-1])
            elif s == "end_header": break
        if b"binary_little_endian" not in hdr:
            raise SystemExit(f"{path}: only binary_little_endian PLY is read here")
        dt = [(p, "<f4" if p in "xyz" else "u1") for p in props]
        rec = np.frombuffer(f.read(), dtype=np.dtype(dt), count=n)
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], -1).astype(np.float64)
    rgb = (np.stack([rec[c] for c in ("red", "green", "blue")], -1)
           if "red" in props else np.full((n, 3), 200, np.uint8))
    return xyz, rgb


def write_ply(path, xyz, rgb):
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = rgb.T
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 f"element vertex {len(xyz)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                 "end_header\n").encode())
        f.write(rec.tobytes())


def voxel(xyz, v):
    ij = np.floor(xyz / v).astype(np.int64)
    _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    return np.stack([np.bincount(inv, xyz[:, d]) / cnt for d in range(3)], -1)


def sim3(p, s, yaw, t):
    c, z = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -z, 0.0], [z, c, 0.0], [0.0, 0.0, 1.0]])
    return s * p @ R.T + t


def trimmed(d, keep=0.8):
    k = max(1, int(keep * len(d)))
    return float(np.sort(d)[:k].mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--da3", required=True); ap.add_argument("--lidar", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--radius", type=float, default=1.2)
    ap.add_argument("--z", type=float, nargs=2, default=(0.08, 2.2))
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--keep", type=float, default=0.8, help="share of nearest pairs the score averages")
    ap.add_argument("--scale-range", type=float, nargs=2, default=(0.6, 3.0))
    a = ap.parse_args()

    def prep(path):
        xyz, rgb = read_ply(path)
        m = (np.hypot(*xyz[:, :2].T) < a.radius) & (xyz[:, 2] > a.z[0]) & (xyz[:, 2] < a.z[1])
        return voxel(xyz[m], a.voxel), xyz, rgb

    src, src_full, src_rgb = prep(a.da3)
    dst, _, _ = prep(a.lidar)
    print(f"DA3   {len(src):,} points, height {src[:, 2].min():.2f}..{src[:, 2].max():.2f} m")
    print(f"LiDAR {len(dst):,} points, height {dst[:, 2].min():.2f}..{dst[:, 2].max():.2f} m")

    t_dst = cKDTree(dst)
    mu_s, mu_d = src.mean(0), dst.mean(0)

    def score(x):
        s, yaw, t = np.exp(x[0]), x[1], x[2:]
        if not (a.scale_range[0] <= s <= a.scale_range[1]):
            return 1e3
        q = sim3(src, s, yaw, t)
        d1 = t_dst.query(q, workers=-1)[0]
        d2 = cKDTree(q).query(dst, workers=-1)[0]
        return 0.5 * (trimmed(d1, a.keep) + trimmed(d2, a.keep))

    best = None
    for yaw in np.radians(np.arange(0, 360, 10.0)):
        for s in np.geomspace(*a.scale_range, 11):
            t = mu_d - sim3(mu_s[None], s, yaw, 0.0)[0]
            x = np.r_[np.log(s), yaw, t]
            v = score(x)
            if best is None or v < best[0]:
                best = (v, x)
    print(f"coarse best: scale x{np.exp(best[1][0]):.3f}, yaw {np.degrees(best[1][1]) % 360:.0f} deg, "
          f"symmetric trimmed Chamfer {best[0] * 100:.2f} cm")

    res = minimize(score, best[1], method="Nelder-Mead",
                   options=dict(xatol=1e-4, fatol=1e-6, maxiter=4000, maxfev=4000))
    s, yaw, t = np.exp(res.x[0]), res.x[1], res.x[2:]
    print(f"refined:     scale x{s:.3f}, yaw {np.degrees(yaw) % 360:.1f} deg, t = {t.round(3)} m, "
          f"score {res.fun * 100:.2f} cm")

    q = sim3(src, s, yaw, t)
    d1 = t_dst.query(q, workers=-1)[0]
    d2 = cKDTree(q).query(dst, workers=-1)[0]
    print(f"\nDA3 -> LiDAR : median {np.median(d1) * 100:5.1f} cm, within 5 cm {(d1 < .05).mean() * 100:3.0f}%, within 10 cm {(d1 < .10).mean() * 100:3.0f}%")
    print(f"LiDAR -> DA3 : median {np.median(d2) * 100:5.1f} cm, within 5 cm {(d2 < .05).mean() * 100:3.0f}%, within 10 cm {(d2 < .10).mean() * 100:3.0f}%")
    print(f"object height: DA3 {q[:, 2].max():.2f} m after the fit, LiDAR {dst[:, 2].max():.2f} m")
    print(f"DA3's own metres were off by x{s:.2f}")

    # crop AFTER the transform, to the cylinder the LiDAR cloud occupies. The DA3 cut-out was made in
    # DA3's own frame, so the fit's 0.8 m translation drags its ground sheet out to a 2.2 m skirt that
    # is nowhere near the object and makes the overlay look broken when it is not.
    aligned = sim3(src_full, s, yaw, t)
    keep = (np.hypot(*aligned[:, :2].T) < a.radius) & (aligned[:, 2] > 0.03)
    print(f"aligned cloud: {len(aligned):,} points -> {keep.sum():,} inside r < {a.radius} m "
          f"(the rest is DA3's ground and background, which the fit did not use)")
    write_ply(os.path.join(a.out_dir, "da3_aligned.ply"), aligned[keep], src_rgb[keep])
    out = dict(scale=float(s), yaw_deg=float(np.degrees(yaw) % 360), t_m=t.tolist(), score_cm=float(res.fun * 100),
               da3_to_lidar_median_cm=float(np.median(d1) * 100), lidar_to_da3_median_cm=float(np.median(d2) * 100),
               da3_within_5cm=float((d1 < .05).mean()), da3_within_10cm=float((d1 < .10).mean()),
               lidar_within_5cm=float((d2 < .05).mean()), lidar_within_10cm=float((d2 < .10).mean()),
               da3_height_m=float(q[:, 2].max()), lidar_height_m=float(dst[:, 2].max()),
               n_da3=len(src), n_lidar=len(dst))
    json.dump(out, open(os.path.join(a.out_dir, "align_to_lidar.json"), "w"), indent=1)

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16.6, 5.0), constrained_layout=True)
    gs = fig.add_gridspec(1, 4)
    for i, (title, P, c) in enumerate([("a · LiDAR", dst, "#2C43B0"), ("b · DA3, aligned", q, "#E07B39")]):
        ax = fig.add_subplot(gs[i], projection="3d")
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=.8, c=c, linewidths=0, depthshade=False)
        ax.set_xlim(-a.radius, a.radius); ax.set_ylim(-a.radius, a.radius); ax.set_zlim(0, 2.2)
        ax.set_box_aspect((1, 1, 1.05)); ax.view_init(elev=14, azim=35)
        ax.set_title(f"{title}, {len(P):,} pts", fontsize=10)
        ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("h (m)", fontsize=8)
    ax = fig.add_subplot(gs[2])
    ax.scatter(dst[:, 0], dst[:, 2], s=1.5, c="#2C43B0", linewidths=0, label="LiDAR")
    ax.scatter(q[:, 0], q[:, 2], s=1.5, c="#E07B39", linewidths=0, alpha=.6, label="DA3")
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("height (m)"); ax.legend(fontsize=9, markerscale=6)
    ax.set_title("c · side view, overlaid", fontsize=10)
    ax = fig.add_subplot(gs[3])
    ax.hist(d1 * 100, bins=70, range=(0, 35), color="#E07B39", alpha=.75, label=f"DA3→LiDAR, median {np.median(d1)*100:.1f} cm")
    ax.hist(d2 * 100, bins=70, range=(0, 35), color="#2C43B0", alpha=.55, label=f"LiDAR→DA3, median {np.median(d2)*100:.1f} cm")
    ax.set_xlabel("nearest-point distance (cm)"); ax.set_ylabel("points"); ax.legend(fontsize=8)
    ax.set_title("d · residual, both directions", fontsize=10)
    fig.suptitle(f"DA3 (RGB, pose-free) against the LiDAR reconstruction of Scotty · "
                 f"DA3's scale was off by x{s:.2f} · symmetric trimmed Chamfer {res.fun * 100:.1f} cm", fontsize=12)
    p = os.path.join(a.out_dir, "da3_vs_lidar.png"); fig.savefig(p, dpi=120)
    print("saved", p)


if __name__ == "__main__":
    main()
