"""Render depth from a trained splat at its own training views and unproject it into a point cloud.

    ~/miniconda3/envs/kplanes/bin/python src/ext/obj_rgb/gs_render_cloud.py \
        /data/christineh/gs_scotty/runs/scotty/splatfacto/posed96/config.yml \
        --out /data/christineh/gs_scotty/exports/gs_cloud.ply

Runs in the `kplanes` env (nerfstudio 1.1.5 + gsplat), not the repo's .venv.

`ns-export pointcloud` cannot do this for a splat in 1.1.5 — it asserts a pixel sampler that the
full-image datamanager a splat uses does not have — and `ns-export gaussian-splat` gives the Gaussians'
centres, which are anisotropic ellipsoids placed wherever they reduce the image loss, not samples of a
surface: on this object there are 8k of them against the LiDAR's 76k, so a Chamfer between the two
would be measuring the representation rather than the geometry.

Rendering the splat's depth at each training camera and unprojecting it puts the splat through the
same step the DA3 clouds go through, which is what makes the two comparable. Points are written in the
world frame of transforms.json; `src.ext.obj_rgb.gs_to_object` moves them to the object frame.

nerfstudio's `camera_to_worlds` is OpenGL (x right, y up, z back) and the rendered depth is the
distance along the camera's -z, so the unprojection carries the sign flips.
"""
import argparse, os
from pathlib import Path

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--out", required=True)
    ap.add_argument("--accum", type=float, default=0.95, help="drop pixels the splat barely covers")
    ap.add_argument("--max-depth", type=float, default=40.0)
    ap.add_argument("--voxel", type=float, default=0.01)
    ap.add_argument("--stride", type=int, default=1, help="use every n-th training view")
    ap.add_argument("--reduce-every", type=int, default=8,
                    help="voxelise the running cloud every n views; 96 full frames at once is 200M points")
    a = ap.parse_args()

    import torch
    from nerfstudio.utils.eval_utils import eval_setup

    _, pipeline, ckpt, _ = eval_setup(Path(a.config), test_mode="inference")
    print("loaded", ckpt)
    cams = pipeline.datamanager.train_dataset.cameras
    print(f"{len(cams)} training views, {int(cams.width[0])}x{int(cams.height[0])}")

    def reduce(p, c, v):
        """One point per voxel, with the mean colour."""
        if v <= 0 or not len(p):
            return p, c
        ij = np.floor(p / v).astype(np.int64)
        _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
        q = np.stack([np.bincount(inv, p[:, d]) / cnt for d in range(3)], -1)
        g = np.stack([np.bincount(inv, c[:, d].astype(float)) / cnt for d in range(3)], -1)
        return q, g.astype(np.uint8)

    pts, cols = [], []
    acc_p = np.zeros((0, 3)); acc_c = np.zeros((0, 3), np.uint8)
    with torch.no_grad():
        for i in range(0, len(cams), a.stride):
            cam = cams[i: i + 1].to(pipeline.device)
            out = pipeline.model.get_outputs_for_camera(cam)
            depth = out["depth"][..., 0].cpu().numpy()
            acc = out["accumulation"][..., 0].cpu().numpy()
            rgb = np.clip(out["rgb"].cpu().numpy(), 0, 1)
            H, W = depth.shape
            fx, fy = float(cams.fx[i]), float(cams.fy[i])
            cx, cy = float(cams.cx[i]), float(cams.cy[i])
            c2w = cams.camera_to_worlds[i].cpu().numpy()          # 3x4, OpenGL axes

            v, u = np.mgrid[0:H, 0:W]
            m = (acc > a.accum) & (depth > 0.1) & (depth < a.max_depth)
            if not m.any():
                continue
            z = depth[m]
            cam_xyz = np.stack([(u[m] - cx) / fx * z, -(v[m] - cy) / fy * z, -z], -1)
            pts.append(cam_xyz @ c2w[:3, :3].T + c2w[:3, 3])
            cols.append((rgb[m] * 255).astype(np.uint8))
            if len(pts) >= a.reduce_every:
                acc_p, acc_c = reduce(np.concatenate([acc_p] + pts),
                                      np.concatenate([acc_c] + cols), a.voxel)
                pts, cols = [], []
                print(f"  view {i + 1}/{len(cams)}: {len(acc_p):,} points so far", flush=True)
    pts, cols = reduce(np.concatenate([acc_p] + pts), np.concatenate([acc_c] + cols), a.voxel)
    print(f"{len(pts):,} points above accumulation {a.accum}, after {a.voxel * 100:.0f} cm voxels")

    write_ply(a.out, pts, cols)
    print(f"saved {a.out} ({os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
