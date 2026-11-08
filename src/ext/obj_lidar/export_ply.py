"""Accumulate one object's LiDAR points over an orbit into a single point cloud and write a PLY.

    python -m src.ext.obj_lidar.export_ply data/raw/2026-11-03.17-01-12 \
        --centre -13.20 -9.87 --orbit 30 165 --pedestal-radius 0.39 --out out/scotty_lidar.ply

This is panel (d) of `src.recon.closeup` turned into a file you can open. Every scan of the window is
cut around the object and re-centred on its pedestal with the same fixed-radius circle fit, which takes
the odometry's position jitter and slow drift out (its yaw error stays). Points are written in the
object frame: x, y in the world's orientation about the refined centre, z = height above the local
ground of that scan.

Colour is the height by default, through the same turbo map the closeup figure uses, so the PLY and
the figure read the same. `--colour photo` instead takes it from the camera: each scan's points are
projected into the frame of that same instant through the fisheye model and the rig solve, exactly as
`recon.closeup` draws them on the photo, and the pixel they land on is the point's colour. The camera
sits 0.14 m from the LiDAR on the same rig, so the two see one side of the object together and little
is hidden; what is hidden is caught by a depth buffer. Points that fall outside the frame, behind the
camera, or in a scan too close for the projection to be trusted keep --no-colour grey.

That last one is the catch worth knowing: the solved lidar-to-camera translation is good to about
0.1 m and parallax grows as 1/range, so below roughly 1.5 m the projection walks off the photo.
--colour-min-range drops those scans from the colouring (not from the geometry).

--voxel downsamples on a regular grid (centroid per cell) so the file stays openable; --voxel 0 writes
every point.
"""
import argparse, json, os

import numpy as np

from ...calib.lidar import LidarTrace
from ...calib.transform import joint_path
from ...calib.board_image import project_fisheye
from ...recon.closeup import Scan

CAM = "calib/camera_fisheye_iq1m.json"


def sample_colour(xyz_sensor, img, R_lc, t_lc, cam, cell=4, tol=0.15):
    """(N,3) LiDAR points in the sensor frame -> (N,3) uint8 colour and (N,) bool 'was coloured'.

    A depth buffer on a `cell`-pixel grid keeps, per cell, only points within `tol` of the nearest one,
    so a surface behind another is not painted with the front one's pixels."""
    H, W = img.shape[:2]
    n = len(xyz_sensor)
    rgb = np.zeros((n, 3), np.uint8)
    got = np.zeros(n, bool)
    cam_xyz = xyz_sensor @ R_lc.T + t_lc
    front = cam_xyz[:, 2] > 0.3
    if not front.any():
        return rgb, got
    uv = project_fisheye(cam_xyz[front], cam)
    u, v = uv[:, 0], uv[:, 1]
    inside = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1)
    if not inside.any():
        return rgb, got
    idx = np.flatnonzero(front)[inside]
    u, v = u[inside], v[inside]
    d = np.linalg.norm(cam_xyz[idx], axis=1)

    key = (v // cell).astype(np.int64) * (W // cell + 2) + (u // cell).astype(np.int64)
    order = np.argsort(key, kind="stable")
    ks, ds = key[order], d[order]
    start = np.r_[0, np.flatnonzero(np.diff(ks)) + 1]
    nearest = np.minimum.reduceat(ds, start)
    per_point = np.repeat(nearest, np.diff(np.r_[start, len(ks)]))
    vis = np.zeros(len(idx), bool)
    vis[order] = ds <= per_point + tol          # back to the pre-sort order

    iu, iv = np.round(u[vis]).astype(np.int32), np.round(v[vis]).astype(np.int32)
    rgb[idx[vis]] = img[iv, iu, :3]
    got[idx[vis]] = True
    return rgb, got


def voxel_mean(xyz, rgb, voxel):
    """One point per occupied voxel: the centroid, with the mean colour."""
    ij = np.floor(xyz / voxel).astype(np.int64)
    _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    out_xyz = np.zeros((len(cnt), 3)); out_rgb = np.zeros((len(cnt), 3))
    for d in range(3):
        out_xyz[:, d] = np.bincount(inv, xyz[:, d]) / cnt
        out_rgb[:, d] = np.bincount(inv, rgb[:, d]) / cnt
    return out_xyz, out_rgb.astype(np.uint8)


def write_ply(path, xyz, rgb):
    """Binary little-endian PLY with per-point colour."""
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = rgb.T
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(xyz)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n").encode()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        f.write(header); f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--centre", type=float, nargs=2, required=True, help="object's world xy, good to 0.5 m")
    ap.add_argument("--orbit", type=float, nargs=2, required=True, help="start, end (s) of the orbit")
    ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius (m)")
    ap.add_argument("--slice", type=float, nargs=2, default=(0.45, 1.0), help="pedestal slice the centre is fitted on")
    ap.add_argument("--pedestal-radius", type=float, default=None)
    ap.add_argument("--z-min", type=float, default=0.05, help="drop points below this height above ground")
    ap.add_argument("--z-max", type=float, default=None,
                    help="drop points above this height above ground; a cylinder set against a building"
                         " catches its overhang, which is not the object")
    ap.add_argument("--stride", type=int, default=1, help="use every n-th scan")
    ap.add_argument("--voxel", type=float, default=0.005, help="downsample grid (m); 0 keeps every point")
    ap.add_argument("--colour", choices=("height", "photo"), default="height")
    ap.add_argument("--frames-dir", default=None,
                    help="--colour photo: dir of camera frames named <index>.jpg, index into camera/ts")
    ap.add_argument("--colour-min-range", type=float, default=1.8,
                    help="--colour photo: skip scans whose object is nearer than this (projection drifts)")
    ap.add_argument("--no-colour", type=int, nargs=3, default=(110, 110, 115),
                    help="--colour photo: grey for points the camera could not give a colour")
    ap.add_argument("--poses-dir", default="data/poses")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    name = os.path.basename(a.trace.rstrip("/"))
    lt = LidarTrace(a.trace)
    L = np.load(os.path.join(a.poses_dir, name, "lidar.npz"))
    t0 = np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8")[0]
    c_guess = np.array(a.centre)

    radius = a.pedestal_radius
    if radius is None:
        radius = Scan(lt, L, t0 + a.orbit[0], c_guess, None, a.slice).radius
        print(f"pedestal radius {radius:.2f} m (free fit on the first scan)")

    photo = a.colour == "photo"
    if photo:
        if not a.frames_dir:
            raise SystemExit("--colour photo needs --frames-dir")
        import matplotlib.image as mpimg
        cam = json.load(open(CAM))
        lc = json.load(open(joint_path(a.trace)))["lidar_to_camera"]
        R_lc, t_lc = np.array(lc["R"]), np.array(lc["t_m"])
        cam_ts = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8")
        print(f"photo colour: {len(cam_ts)} camera frames, lidar->camera sigma "
              f"{lc['sigma']['rotvec_deg'][0]:.2f} deg / {lc['sigma']['t_m'][0] * 100:.1f} cm")

    sel = np.flatnonzero((L["t"] >= t0 + a.orbit[0]) & (L["t"] < t0 + a.orbit[1]))[::a.stride]
    pts, hts, fits, centres, cols = [], [], [], [], []
    n_close, n_got, n_tot = 0, 0, 0
    for j in sel:
        s = Scan(lt, L, L["t"][j], c_guess, radius, a.slice)
        if s.fit < 0.3 or s.n_slice < 200:
            continue
        cut = s.cut(a.radius) & (s.h > a.z_min)
        if a.z_max is not None:
            cut &= s.h < a.z_max
        if not cut.any():
            continue
        pts.append((s.xyz[cut, :2] - s.c) @ s.T[:2, :2].T)      # re-centred, world orientation
        hts.append(s.h[cut])
        fits.append(s.fit); centres.append(s.world_xy(s.c))
        if photo:
            c = np.tile(np.array(a.no_colour, np.uint8), (int(cut.sum()), 1))
            if s.dist() >= a.colour_min_range:
                k = int(np.argmin(np.abs(cam_ts - L["t"][j])))
                f = os.path.join(a.frames_dir, f"{k:06d}.jpg")
                if os.path.exists(f):
                    img = mpimg.imread(f)
                    cc, got = sample_colour(s.xyz[cut], img, R_lc, t_lc, cam)
                    c[got] = cc[got]
                    n_got += int(got.sum())
            else:
                n_close += 1
            n_tot += int(cut.sum())
            cols.append(c)
    if not pts:
        raise SystemExit("no scan passed the pedestal fit; check --centre / --pedestal-radius")

    xy = np.concatenate(pts); h = np.concatenate(hts); centres = np.array(centres)
    xyz = np.column_stack([xy, h])

    if photo:
        rgb = np.concatenate(cols)
        print(f"photo colour: {n_got:,} of {n_tot:,} points got a pixel ({100 * n_got / max(n_tot, 1):.0f}%); "
              f"{n_close} scans skipped as nearer than {a.colour_min_range} m")
    else:
        import matplotlib
        rgb = (matplotlib.colormaps["turbo"](np.clip(h / 2.2, 0, 1))[:, :3] * 255).astype(np.uint8)

    n_raw = len(xyz)
    if a.voxel > 0:
        xyz, rgb = voxel_mean(xyz, rgb.astype(float), a.voxel)

    write_ply(a.out, xyz, rgb)
    jitter = float(np.hypot(*centres.std(0)))
    print(f"{name} {a.orbit[0]:.0f}-{a.orbit[1]:.0f} s: {len(fits)} of {len(sel)} scans used, "
          f"fit inlier share median {np.median(fits):.2f}; refined centre (world) {centres.mean(0).round(2)}, "
          f"scan centres scatter {jitter:.2f} m")
    print(f"{n_raw:,} points -> {len(xyz):,} after {a.voxel * 100:.1f} cm voxels; "
          f"extent x {xyz[:, 0].min():+.2f}..{xyz[:, 0].max():+.2f}, y {xyz[:, 1].min():+.2f}..{xyz[:, 1].max():+.2f}, "
          f"height {xyz[:, 2].min():.2f}..{xyz[:, 2].max():.2f} m")
    print("saved", a.out, f"({os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
