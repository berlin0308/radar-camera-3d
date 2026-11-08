"""Depth Anything 3 (pose-free, any-view) on an orbit's camera frames -> point cloud of the object.

    CUDA_VISIBLE_DEVICES=1 ~/miniconda3/envs/da3/bin/python src/ext/obj_rgb/run_da3.py \
        out/da3_scotty/frames --out-dir out/da3_scotty --scale-from-orbit 2.6

Runs in the `da3` conda env (torch 2.5.1+cu124), not the repo's .venv: DA3 is installed there from
~/Desktop/depth_models/Depth-Anything-3. The script is standalone (numpy + torch + DA3 only) so it
does not need the repo's own packages.

Pose-free means DA3 estimates the cameras itself, so the reconstruction is up to a similarity: the
shape is right, the metre scale is not. `--scale-from-orbit R` fixes that with one number we know from
the session log (the orbit radius in metres): the camera centres are fitted with a circle and every
length is multiplied so that circle has radius R. The object then sits at the circle's centre, which is
how the statue is cut out of the scene.

Writes, in --out-dir:
  da3_pred.npz    depth [N,h,w], conf, extrinsics (w2c 4x4), intrinsics, the fitted scale and centre
  scene.ply       the whole scene, metric, z up, origin at the object
  object.ply      the cylinder of radius --radius about the object
  da3_object.png  camera frame, depth, top view and two elevations of the cut-out
"""
import argparse, os, sys

import numpy as np


def circle_fit(xy):
    """Algebraic circle fit: returns (centre, radius)."""
    A = np.c_[2 * xy, np.ones(len(xy))]
    sol = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)[0]
    return sol[:2], float(np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2))


def write_ply(path, xyz, rgb):
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = rgb.T
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 f"element vertex {len(xyz)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                 "end_header\n").encode())
        f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", help="directory of orbit frames, sorted by name = by time")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--model", default="depth-anything/DA3-GIANT-1.1")
    ap.add_argument("--res", type=int, default=504)
    ap.add_argument("--scale-from-orbit", type=float, default=None, metavar="R",
                    help="known orbit radius (m); scales the reconstruction to metres")
    # NOTE: scaling on the pedestal radius was tried and dropped. The accumulated cloud gives a filled
    # disc rather than the single-scan arc `recon.closeup` fits, and the algebraic circle fit degenerates
    # on it (r = 0.61 units against the 0.10 it should be). Scale from the orbit here and, for a number
    # you can quote, solve it against the LiDAR with src/ext/obj_rgb/align_to_lidar.py.
    ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius about the object (m)")
    ap.add_argument("--conf-pct", type=float, default=40.0, help="drop the least confident this %% of pixels")
    ap.add_argument("--voxel", type=float, default=0.01)
    a = ap.parse_args()

    import torch
    from depth_anything_3.api import DepthAnything3

    paths = sorted(os.path.join(a.frames, f) for f in os.listdir(a.frames)
                   if f.lower().endswith((".png", ".jpg", ".jpeg")))
    print(f"{len(paths)} frames from {a.frames}")

    model = DepthAnything3.from_pretrained(a.model).to("cuda").eval()
    torch.cuda.reset_peak_memory_stats()
    pred = model.inference(paths, process_res=a.res, process_res_method="upper_bound_resize")
    torch.cuda.synchronize()
    print(f"inference done, peak {torch.cuda.max_memory_allocated() / 1e9:.1f} GB")

    depth = pred.depth.astype(np.float32)                      # [N, h, w]
    conf = pred.conf.astype(np.float32)
    K = np.asarray(pred.intrinsics, np.float64)                # [N, 3, 3]
    w2c = np.stack([np.vstack([e[:3], [0, 0, 0, 1]]) for e in pred.extrinsics])
    N, h, w = depth.shape
    print(f"depth {depth.shape}, intrinsics f={K[0,0,0]:.1f} px at {w}x{h}")

    c2w = np.linalg.inv(w2c)
    cams = c2w[:, :3, 3]

    # --- put the scene on its feet: the cameras of a level handheld orbit share a height, so the
    # plane they span is horizontal. Its normal is the up axis (sign fixed by the scene being below).
    cc = cams - cams.mean(0)
    up = np.linalg.svd(cc)[2][2]
    R_up = np.eye(3)
    R_up[2] = up
    R_up[0] = np.cross([0.0, 0.0, 1.0], up) if abs(up[2]) < 0.9 else np.cross([0.0, 1.0, 0.0], up)
    R_up[0] /= np.linalg.norm(R_up[0])
    R_up[1] = np.cross(R_up[2], R_up[0])
    cams_u = cams @ R_up.T

    centre_xy, radius = circle_fit(cams_u[:, :2])
    scale = 1.0
    if a.scale_from_orbit:
        scale = a.scale_from_orbit / radius
        print(f"orbit radius {radius:.3f} DA3 units -> {a.scale_from_orbit} m, scale x{scale:.2f}")
    print(f"camera circle fit: radius {radius * scale:.2f} m, height spread "
          f"{(cams_u[:, 2].max() - cams_u[:, 2].min()) * scale:.2f} m")

    # --- unproject every pixel
    yy, xx = np.mgrid[0:h, 0:w]
    keep_thr = np.percentile(conf, a.conf_pct)
    pts, cols = [], []
    from PIL import Image
    for i in range(N):
        m = (conf[i] > keep_thr) & (depth[i] > 0)
        z = depth[i][m]
        x = (xx[m] - K[i, 0, 2]) / K[i, 0, 0] * z
        y = (yy[m] - K[i, 1, 2]) / K[i, 1, 1] * z
        p_cam = np.stack([x, y, z], -1)
        pts.append(p_cam @ c2w[i, :3, :3].T + c2w[i, :3, 3])
        img = np.asarray(Image.open(paths[i]).resize((w, h), Image.BILINEAR))[..., :3]
        cols.append(img[m])
    pts = np.concatenate(pts); cols = np.concatenate(cols).astype(np.uint8)
    print(f"{len(pts):,} points above the {a.conf_pct:.0f}th confidence percentile")

    # into the levelled, object-centred frame (still DA3 units)
    pts = pts @ R_up.T
    origin = np.r_[centre_xy, np.percentile(cams_u[:, 2], 50)]
    pts -= origin
    cams_o = cams_u - origin
    rho = np.hypot(*pts[:, :2].T)

    def ground_level(s):
        """Height of the ground in DA3 units: the low points of the annulus just outside the object."""
        ring = (rho > 1.0 / s) & (rho < 3.0 / s)
        return float(np.percentile(pts[ring][:, 2], 2.0)) if ring.sum() > 1000 else float(np.percentile(pts[:, 2], 2.0))

    ground = ground_level(scale)
    pts = (pts - np.r_[0, 0, ground]) * scale
    cams_o = (cams_o - np.r_[0, 0, ground]) * scale
    print(f"scale x{scale:.2f}; camera height {cams_o[:, 2].mean():.2f} m, orbit radius {radius * scale:.2f} m")

    os.makedirs(a.out_dir, exist_ok=True)
    np.savez_compressed(os.path.join(a.out_dir, "da3_pred.npz"), depth=depth, conf=conf,
                        w2c=w2c, K=K, R_up=R_up, origin=origin, scale=scale, ground=ground,
                        orbit_radius=radius * scale, cams=cams_o, frames=np.array(paths))

    def vox(p, c, v):
        ij = np.floor(p / v).astype(np.int64)
        _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
        q = np.stack([np.bincount(inv, p[:, d]) / cnt for d in range(3)], -1)
        g = np.stack([np.bincount(inv, c[:, d].astype(float)) / cnt for d in range(3)], -1)
        return q, g.astype(np.uint8)

    near = np.hypot(*pts[:, :2].T) < 15.0
    p_s, c_s = vox(pts[near], cols[near], max(a.voxel * 3, 0.03))
    write_ply(os.path.join(a.out_dir, "scene.ply"), p_s, c_s)
    print(f"scene.ply {len(p_s):,} points")

    obj = (np.hypot(*pts[:, :2].T) < a.radius) & (pts[:, 2] > 0.05) & (pts[:, 2] < 3.0)
    p_o, c_o = vox(pts[obj], cols[obj], a.voxel)
    write_ply(os.path.join(a.out_dir, "object.ply"), p_o, c_o)
    print(f"object.ply {len(p_o):,} points, height {p_o[:, 2].min():.2f}..{p_o[:, 2].max():.2f} m, "
          f"width {p_o[:, 0].ptp():.2f} x {p_o[:, 1].ptp():.2f} m")

    # --- figure
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16.6, 4.6), constrained_layout=True)
    gs = fig.add_gridspec(1, 4)
    ax = fig.add_subplot(gs[0]); ax.imshow(np.asarray(Image.open(paths[N // 2]))); ax.axis("off")
    ax.set_title(f"a · frame {N // 2} of {N}", fontsize=10)
    ax = fig.add_subplot(gs[1]); im = ax.imshow(depth[N // 2], cmap="turbo"); ax.axis("off")
    ax.set_title("b · DA3 depth, same frame", fontsize=10); fig.colorbar(im, ax=ax, shrink=.8)
    ax = fig.add_subplot(gs[2])
    st = max(1, len(p_s) // 200_000)
    ax.scatter(p_s[::st, 0], p_s[::st, 1], s=.3, c=c_s[::st] / 255, linewidths=0)
    ax.plot(cams_o[:, 0], cams_o[:, 1], "-o", c="#E07B39", ms=3, lw=1, label="DA3 cameras")
    ax.add_patch(plt.Circle((0, 0), a.radius, fill=False, color="#15202B", ls="--"))
    ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title(f"c · top view, metric (orbit r = {radius * scale:.2f} m)", fontsize=10)
    ax = fig.add_subplot(gs[3], projection="3d")
    sb = max(1, len(p_o) // 150_000)
    ax.scatter(p_o[::sb, 0], p_o[::sb, 1], p_o[::sb, 2], s=.6, c=c_o[::sb] / 255, linewidths=0, depthshade=False)
    ax.set_xlim(-a.radius, a.radius); ax.set_ylim(-a.radius, a.radius); ax.set_zlim(0, 2.4)
    ax.set_box_aspect((1, 1, 1.05)); ax.view_init(elev=14, azim=35)
    ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("height (m)", fontsize=8)
    ax.set_title(f"d · the object, {len(p_o):,} points", fontsize=10)
    fig.suptitle(f"Depth Anything 3 (pose-free) on {N} orbit frames · {a.model}", fontsize=12)
    out = os.path.join(a.out_dir, "da3_object.png"); fig.savefig(out, dpi=120)
    print("saved", out)


if __name__ == "__main__":
    sys.exit(main())
