"""Write a nerfstudio dataset from the calibrated poses, so a splat can be trained without COLMAP.

    .venv/bin/python -m src.ext.obj_rgb.export_nerfstudio out/da3_scotty/poses96.npz \
        --out /data/christineh/gs_scotty/data --seed-ply out/da3_posed_n96_r504/scene.ply

We already know where every frame was taken from (KISS-ICP odometry composed with the rig solve), so
the structure-from-motion step a splat normally starts with is not needed: `transforms.json` carries
the poses and the training runs with them fixed, in metres, in the same world frame as everything else.

Two conversions matter. nerfstudio's `transform_matrix` is camera-to-world in the OpenGL convention
(x right, y **up**, z **back**), while ours is OpenCV (x right, y down, z forward), so the rotation is
post-multiplied by diag(1, -1, -1). And gsplat's rasteriser is pinhole only, so the frames are
undistorted here and a PERSPECTIVE camera is written; the lens is equidistant fisheye, and feeding its
K to a pinhole rasteriser would bend every ray at the edge of the frame.

--origin object puts the world origin on the statue instead of on the first LiDAR scan. It changes
nothing about the geometry — every camera and seed point moves by the same translation — but it is
what makes `ns-viewer` usable: its camera orbits the world origin and zooms toward it, so with the
origin 16 m away in the courtyard you can never get near the object, however far you scroll. It also
makes the trained world the object frame, so `gs_to_object` has nothing left to subtract.

--seed-ply gives the splat its initial points. A splat is normally seeded with COLMAP's sparse cloud;
here the DA3 reconstruction stands in for it, which keeps the whole chain camera-only. Pass nothing
for a random initialisation.
"""
import argparse, json, os, shutil

import numpy as np

# OpenCV camera axes -> OpenGL/nerfstudio camera axes
CV_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("poses", help="npz from src.ext.obj_rgb.export_poses")
    ap.add_argument("--out", required=True, help="dataset dir (put it on /data, not /)")
    ap.add_argument("--balance", type=float, default=0.0)
    ap.add_argument("--origin", choices=("world", "object"), default="object",
                    help="world = the frame of data/poses/<trace>; object = the pedestal centre and ground")
    ap.add_argument("--seed-ply", default=None)
    ap.add_argument("--seed-voxel", type=float, default=0.02)
    ap.add_argument("--seed-radius", type=float, default=12.0, help="keep seed points within this of the object")
    a = ap.parse_args()

    import cv2
    P = np.load(a.poses, allow_pickle=True)
    paths = [str(p) for p in P["frames"]]
    w2c, K, D = P["w2c"], P["K"], P["D"]
    W, H = (int(x) for x in P["wh"])
    c_ref, ground = P["c_ref"], P["ground_frame"]

    img_dir = os.path.join(a.out, "images")
    os.makedirs(img_dir, exist_ok=True)
    K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (W, H), np.eye(3), balance=a.balance)
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K_new, (W, H), cv2.CV_16SC2)
    print(f"undistort: f {K[0,0]:.0f} -> {K_new[0,0]:.0f} px, field "
          f"{2*np.degrees(np.arctan(W/2/K[0,0])):.0f} -> {2*np.degrees(np.arctan(W/2/K_new[0,0])):.0f} deg")

    shift = (np.r_[c_ref, np.median(ground)] if a.origin == "object" else np.zeros(3))
    print(f"world origin: {a.origin}" + (f", shifted by {-shift.round(2)} m" if a.origin == "object" else ""))

    frames = []
    for i, p in enumerate(paths):
        name = f"{i:04d}.png"
        q = os.path.join(img_dir, name)
        if not os.path.exists(q):
            cv2.imwrite(q, cv2.remap(cv2.imread(p), m1, m2, interpolation=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT))
        c2w = np.linalg.inv(w2c[i]) @ CV_TO_GL
        c2w[:3, 3] -= shift
        frames.append(dict(file_path=f"images/{name}", transform_matrix=c2w.tolist()))

    meta = dict(camera_model="OPENCV", w=W, h=H,
                fl_x=float(K_new[0, 0]), fl_y=float(K_new[1, 1]),
                cx=float(K_new[0, 2]), cy=float(K_new[1, 2]),
                k1=0.0, k2=0.0, p1=0.0, p2=0.0, frames=frames)

    if a.seed_ply:
        from .align_to_lidar import read_ply, voxel
        xyz, rgb = read_ply(a.seed_ply)
        # the seed cloud is in the object frame; put it back in the world the poses live in
        keep = np.hypot(*xyz[:, :2].T) < a.seed_radius
        xyz, rgb = xyz[keep], rgb[keep]
        world = np.column_stack([xyz[:, 0] + c_ref[0], xyz[:, 1] + c_ref[1],
                                 xyz[:, 2] + np.median(ground)]) - shift
        ij = np.floor(world / a.seed_voxel).astype(np.int64)
        _, idx = np.unique(ij, axis=0, return_index=True)
        world, rgb = world[idx], rgb[idx]
        from .align_to_lidar import write_ply
        sp = os.path.join(a.out, "seed.ply")
        write_ply(sp, world, rgb)
        meta["ply_file_path"] = "seed.ply"
        print(f"seed cloud {len(world):,} points from {a.seed_ply}")

    json.dump(meta, open(os.path.join(a.out, "transforms.json"), "w"), indent=1)
    c2ws = np.stack([np.array(f["transform_matrix"]) for f in frames])
    print(f"{len(frames)} frames, {W}x{H}; camera centres x {c2ws[:,0,3].min():.1f}..{c2ws[:,0,3].max():.1f}, "
          f"y {c2ws[:,1,3].min():.1f}..{c2ws[:,1,3].max():.1f}, z {c2ws[:,2,3].min():.2f}..{c2ws[:,2,3].max():.2f} m")
    print(f"object at ({c_ref[0]:.2f}, {c_ref[1]:.2f}), ground z {np.median(ground):.3f} m")
    print("wrote", os.path.join(a.out, "transforms.json"))
    print("\nnext:\n  ~/miniconda3/envs/kplanes/bin/ns-train splatfacto --data", a.out,
          "\\\n    --output-dir /data/christineh/gs_scotty/runs --vis tensorboard \\\n"
          "    nerfstudio-data --orientation-method none --center-method none --auto-scale-poses False")


if __name__ == "__main__":
    main()
