"""Depth Anything 3 told where the cameras were -> a metric cloud, no Sim3 afterwards.

    CUDA_VISIBLE_DEVICES=1 ~/miniconda3/envs/da3/bin/python src/ext/obj_rgb/run_da3_posed.py \
        out/da3_scotty/poses.npz --out-dir out/da3_scotty_posed

Runs in the `da3` conda env, like run_da3.py, and takes the npz `src.ext.obj_rgb.export_poses` writes
in the repo's .venv (poses, intrinsics, and the object frame). The difference from run_da3.py is the
whole point: `inference(..., extrinsics=w2c, intrinsics=K, align_to_input_ext_scale=True)` returns
depth already in the world frame of data/poses/<trace>, in metres, so there is no orbit radius to
guess and no fit onto the LiDAR to set the scale.

**The frames are undistorted first.** The lens is OPENCV_FISHEYE and, though k1..k4 are small, the
model is equidistant, not pinhole: a corner point lands 273 px from where a pinhole with the same
focal length would put it (16 px at DA3's 504-wide working size). Handing DA3 a pinhole K for a
fisheye image would be worse than telling it nothing. cv2.fisheye.undistortImage with a new matrix
from estimateNewCameraMatrixForUndistortRectify(balance=--balance) does it; balance 0 crops to pixels
that are all valid, 1 keeps the whole field and pads with black.

Writes, in --out-dir: da3_pred.npz, undistorted/, scene.ply, object.ply (both in the object frame of
the LiDAR and radar clouds, z = height above the ground of that frame's moment) and da3_posed.png.
"""
import argparse, os, sys

import numpy as np


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


def vox(p, c, v):
    ij = np.floor(p / v).astype(np.int64)
    _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    q = np.stack([np.bincount(inv, p[:, d]) / cnt for d in range(3)], -1)
    g = np.stack([np.bincount(inv, c[:, d].astype(float)) / cnt for d in range(3)], -1)
    return q, g.astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("poses", help="npz from src.ext.obj_rgb.export_poses")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--model", default="depth-anything/DA3-GIANT-1.1")
    ap.add_argument("--res", type=int, default=504)
    ap.add_argument("--every", type=int, default=1, help="use every n-th frame (memory at high --res)")
    ap.add_argument("--balance", type=float, default=0.0, help="fisheye undistort: 0 crops, 1 keeps the field")
    ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius about the object (m)")
    ap.add_argument("--conf-pct", type=float, default=40.0)
    ap.add_argument("--voxel", type=float, default=0.01)
    ap.add_argument("--no-poses", action="store_true", help="ablation: undistort but run pose-free")
    ap.add_argument("--no-undistort", action="store_true",
                    help="ablation: keep the fisheye frames and pass the fisheye K as if it were a pinhole")
    a = ap.parse_args()

    import cv2
    import torch
    from PIL import Image
    from depth_anything_3.api import DepthAnything3

    P = np.load(a.poses, allow_pickle=True)
    sub = slice(None, None, a.every)
    paths = [str(p) for p in P["frames"]][sub]
    w2c, K, D = P["w2c"][sub], P["K"], P["D"]
    W, H = (int(x) for x in P["wh"])
    c_ref, ground_frame = P["c_ref"], P["ground_frame"][sub]
    print(f"{len(paths)} frames, {W}x{H}; object centre {c_ref.round(2)}, "
          f"ground z {ground_frame.min():.3f}..{ground_frame.max():.3f} m")

    # --- undistort to a pinhole, once
    und_dir = os.path.join(a.out_dir, "undistorted")
    os.makedirs(und_dir, exist_ok=True)
    K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (W, H), np.eye(3), balance=a.balance)
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K_new, (W, H), cv2.CV_16SC2)
    fov_in = 2 * np.degrees(np.arctan(W / 2 / K[0, 0]))
    fov_out = 2 * np.degrees(np.arctan(W / 2 / K_new[0, 0]))
    print(f"undistort balance {a.balance}: f {K[0,0]:.0f} -> {K_new[0,0]:.0f} px, "
          f"horizontal field {fov_in:.0f} -> {fov_out:.0f} deg")
    if a.no_undistort:
        print("  (ablation) --no-undistort: feeding the raw fisheye frames and the fisheye K")
        und, K_new = list(paths), K
    else:
        und = []
        for p in paths:
            q = os.path.join(und_dir, os.path.basename(p))
            if not os.path.exists(q):
                cv2.imwrite(q, cv2.remap(cv2.imread(p), m1, m2, interpolation=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_CONSTANT))
            und.append(q)

    model = DepthAnything3.from_pretrained(a.model).to("cuda").eval()
    torch.cuda.reset_peak_memory_stats()
    kw = {} if a.no_poses else dict(extrinsics=w2c, intrinsics=np.repeat(K_new[None], len(und), 0),
                                    align_to_input_ext_scale=True)
    pred = model.inference(und, process_res=a.res, process_res_method="upper_bound_resize", **kw)
    torch.cuda.synchronize()
    print(f"inference {'pose-free (ablation)' if a.no_poses else 'with the calibrated poses'}, "
          f"peak {torch.cuda.max_memory_allocated() / 1e9:.1f} GB")

    depth = pred.depth.astype(np.float32)
    conf = pred.conf.astype(np.float32)
    Kp = np.asarray(pred.intrinsics, np.float64)
    ex = np.stack([np.vstack([e[:3], [0, 0, 0, 1]]) for e in pred.extrinsics])
    c2w = np.linalg.inv(ex)
    N, h, w = depth.shape
    print(f"depth {depth.shape}; camera centres z {c2w[:, 2, 3].min():.2f}..{c2w[:, 2, 3].max():.2f} m")
    if not a.no_poses:
        err = np.linalg.norm(c2w[:, :3, 3] - np.linalg.inv(w2c)[:, :3, 3], axis=1)
        print(f"returned camera centres match the ones given to {err.max() * 1000:.1f} mm")

    yy, xx = np.mgrid[0:h, 0:w]
    thr = np.percentile(conf, a.conf_pct)
    pts, cols, src = [], [], []
    for i in range(N):
        m = (conf[i] > thr) & (depth[i] > 0)
        z = depth[i][m]
        x = (xx[m] - Kp[i, 0, 2]) / Kp[i, 0, 0] * z
        y = (yy[m] - Kp[i, 1, 2]) / Kp[i, 1, 1] * z
        pts.append(np.stack([x, y, z], -1) @ c2w[i, :3, :3].T + c2w[i, :3, 3])
        cols.append(np.asarray(Image.open(und[i]).resize((w, h), Image.BILINEAR))[..., :3][m])
        src.append(np.full(int(m.sum()), i))
    pts = np.concatenate(pts); cols = np.concatenate(cols).astype(np.uint8); src = np.concatenate(src)
    print(f"{len(pts):,} points above the {a.conf_pct:.0f}th confidence percentile")

    # --- world -> the object frame the LiDAR and radar clouds live in
    if a.no_poses:
        print("pose-free: the output is not in the world frame, so no object frame is applied")
        obj_pts = pts
    else:
        obj_pts = np.column_stack([pts[:, 0] - c_ref[0], pts[:, 1] - c_ref[1],
                                   pts[:, 2] - ground_frame[src]])

    os.makedirs(a.out_dir, exist_ok=True)
    np.savez_compressed(os.path.join(a.out_dir, "da3_pred.npz"), depth=depth, conf=conf,
                        w2c_out=ex, K=Kp, K_new=K_new, cams=c2w[:, :3, 3], frames=np.array(und),
                        c_ref=c_ref, ground_frame=ground_frame, posed=not a.no_poses)

    rho = np.hypot(*obj_pts[:, :2].T)
    near = rho < 15.0
    p_s, c_s = vox(obj_pts[near], cols[near], max(a.voxel * 3, 0.03))
    write_ply(os.path.join(a.out_dir, "scene.ply"), p_s, c_s)
    print(f"scene.ply {len(p_s):,} points")

    obj = (rho < a.radius) & (obj_pts[:, 2] > 0.03) & (obj_pts[:, 2] < 3.0)
    p_o, c_o = vox(obj_pts[obj], cols[obj], a.voxel)
    write_ply(os.path.join(a.out_dir, "object.ply"), p_o, c_o)
    print(f"object.ply {len(p_o):,} points, height {p_o[:, 2].min():.2f}..{p_o[:, 2].max():.2f} m, "
          f"width {p_o[:, 0].ptp():.2f} x {p_o[:, 1].ptp():.2f} m")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16.6, 4.6), constrained_layout=True)
    gs = fig.add_gridspec(1, 4)
    ax = fig.add_subplot(gs[0]); ax.imshow(np.asarray(Image.open(und[N // 2]))); ax.axis("off")
    ax.set_title(f"a · frame {N // 2} of {N}, undistorted", fontsize=10)
    ax = fig.add_subplot(gs[1]); im = ax.imshow(depth[N // 2], cmap="turbo"); ax.axis("off")
    ax.set_title("b · DA3 depth, same frame", fontsize=10); fig.colorbar(im, ax=ax, shrink=.8)
    ax = fig.add_subplot(gs[2])
    st = max(1, len(p_s) // 200_000)
    ax.scatter(p_s[::st, 0], p_s[::st, 1], s=.3, c=c_s[::st] / 255, linewidths=0)
    cam_o = c2w[:, :3, 3] - np.r_[c_ref, 0] if not a.no_poses else c2w[:, :3, 3]
    ax.plot(cam_o[:, 0], cam_o[:, 1], "-o", c="#E07B39", ms=3, lw=1, label="cameras (given)")
    ax.add_patch(plt.Circle((0, 0), a.radius, fill=False, color="#15202B", ls="--"))
    ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_title("c · top view, metric by construction", fontsize=10)
    ax = fig.add_subplot(gs[3], projection="3d")
    sb = max(1, len(p_o) // 150_000)
    ax.scatter(p_o[::sb, 0], p_o[::sb, 1], p_o[::sb, 2], s=.6, c=c_o[::sb] / 255,
               linewidths=0, depthshade=False)
    ax.set_xlim(-a.radius, a.radius); ax.set_ylim(-a.radius, a.radius); ax.set_zlim(0, 2.4)
    ax.set_box_aspect((1, 1, 1.05)); ax.view_init(elev=14, azim=35)
    ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("height (m)", fontsize=8)
    ax.set_title(f"d · the object, {len(p_o):,} points, to {p_o[:, 2].max():.2f} m", fontsize=10)
    fig.suptitle(f"DA3 given the calibrated camera poses · {N} frames · {a.model}", fontsize=12)
    out = os.path.join(a.out_dir, "da3_posed.png"); fig.savefig(out, dpi=120)
    print("saved", out)


if __name__ == "__main__":
    sys.exit(main())
