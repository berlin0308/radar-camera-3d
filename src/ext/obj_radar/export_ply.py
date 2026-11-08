"""Accumulate the radar model's reconstruction of one object over an orbit and write a PLY.

    python -m src.ext.obj_radar.export_ply data/raw/2026-11-03.17-01-12 \
        --dump /data/berlin/grt/2026-11-03.17-01-12 --centre -13.20 -9.87 --orbit 30 165 \
        --pedestal-radius 0.39 --out out/scotty_radar.ply

The radar's own 3D of the object, in the frame `src.ext.obj_lidar.export_ply` writes: x, y in the
world's orientation about the refined pedestal centre, z = height above that scan's local ground. The
two PLYs therefore overlay without a fit, and so does the DA3 one once `align_to_lidar` has placed it.

GRT (`--source grt`, the default) predicts a 64 x 128 x 64 (elevation, azimuth, range) occupancy grid
per radar frame, already in the LiDAR sensor frame, so no extrinsic enters here. Cells above
--threshold are taken as points. Only the **first** occupied cell of each (el, az) column is kept:
behind a surface the grid keeps firing, and accumulating those over an orbit fills the object solid.

`--source cfar` instead replays the CFAR point cloud of `data/recon/<trace>/radar_points.npz`. Those
points carry no elevation, so they are drawn at the radar's height above the ground — a flat ring, not
a shape. It is there as the baseline the model is supposed to beat, not as a reconstruction.

Per scan the object's centre is refined on its pedestal exactly as the LiDAR export does, and the local
ground comes from the LiDAR points of that instant (the GRT grid has no ground fit of its own).

--scene widens the cut-out and drops the per-scan re-centring, for the background layer of the viewer.
The re-centring is what takes the odometry's jitter out of the object, but it is a correction measured
on the pedestal: applied to something 10 m away it would move that thing by the same amount for no
reason. So the scene is accumulated in the world frame and only shifted, once, so that the refined
pedestal centre is the origin — the same thing `src.ext.obj_lidar.export_scene` does for the LiDAR.
"""
import argparse, os

import numpy as np

from ...calib.lidar import LidarTrace
from ...recon.closeup import Scan
from ...grt.occupancy import GridGeometry
from ..obj_lidar.export_ply import voxel_mean, write_ply


def first_return(occ):
    """(el, az, rng) bool -> (el, az) index of the first occupied range cell, -1 where none."""
    any_ = occ.any(axis=-1)
    k = occ.argmax(axis=-1)
    k[~any_] = -1
    return k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--dump", help="GRT dump dir (occ3d_logit.npy, ts.npy); required for --source grt")
    ap.add_argument("--source", choices=("grt", "cfar"), default="grt")
    ap.add_argument("--centre", type=float, nargs=2, required=True)
    ap.add_argument("--orbit", type=float, nargs=2, required=True)
    ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius (m)")
    ap.add_argument("--scene", action="store_true",
                    help="the surroundings instead of the object: world frame, no per-scan re-centring")
    ap.add_argument("--slice", type=float, nargs=2, default=(0.45, 1.0))
    ap.add_argument("--pedestal-radius", type=float, default=None)
    ap.add_argument("--threshold", type=float, default=0.5, help="GRT occupancy probability")
    ap.add_argument("--all-cells", action="store_true", help="keep every occupied cell, not just the first of each column")
    ap.add_argument("--all-detections", action="store_true", help="--source cfar: keep the points the static-world filter rejected too")
    ap.add_argument("--z-min", type=float, default=0.05)
    ap.add_argument("--z-max", type=float, default=2.6)
    ap.add_argument("--stride", type=int, default=2, help="use every n-th LiDAR scan")
    ap.add_argument("--voxel", type=float, default=0.02)
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

    if a.source == "grt":
        if not a.dump:
            raise SystemExit("--dump is required for --source grt")
        G = GridGeometry(a.trace)
        ts_d = np.load(os.path.join(a.dump, "ts.npy"))
        logit = np.load(os.path.join(a.dump, "occ3d_logit.npy"), mmap_mode="r")
        print(f"GRT {logit.shape} cells, {len(ts_d)} frames, {ts_d[0] - t0:.1f}..{ts_d[-1] - t0:.1f} s; "
              f"cells {G.d_rng * 100:.0f} cm in range, {np.degrees(G.az[0] - G.az[1]):.2f} deg across")
    else:
        rec = np.load(os.path.join("data/recon", name, "radar_points.npz"))
        P = np.load(os.path.join(a.poses_dir, name, "radar.npz"))
        keep = rec["static"] if not a.all_detections else np.ones(len(rec["xyz"]), bool)
        cf_xyz, cf_t = rec["xyz"][keep], P["t"][rec["frame"][keep]]
        print(f"CFAR {keep.sum():,} of {len(keep):,} points" + ("" if a.all_detections else " pass the static filter"))

    sel = np.flatnonzero((L["t"] >= t0 + a.orbit[0]) & (L["t"] < t0 + a.orbit[1]))[::a.stride]
    pts, hts, used, n_frames = [], [], 0, 0
    scans = []                                   # (t, refined centre in world xy, ground z in world) per fitted scan
    for j in sel:
        t = L["t"][j]
        s = Scan(lt, L, t, c_guess, radius, a.slice)
        if s.fit < 0.3 or s.n_slice < 200:
            continue
        used += 1
        # local ground of this scan, in the sensor frame: z of the LiDAR points minus their height
        near = s.cut(a.radius + 1.0)
        if near.sum() < 200:
            continue
        ground_z = float(np.median(s.xyz[near, 2] - s.h[near]))
        scans.append((t, s.world_xy(s.c), float((s.T[:3, :3] @ np.r_[s.c, ground_z] + s.T[:3, 3])[2])))

        if a.source == "cfar":
            continue
        if a.source == "grt":
            k = int(np.argmin(np.abs(ts_d - t)))
            if abs(ts_d[k] - t) > 0.15:
                continue
            n_frames += 1
            p = 1.0 / (1.0 + np.exp(-np.asarray(logit[k], dtype=np.float32)))
            occ = p > a.threshold
            if a.all_cells:
                cells = np.argwhere(occ)
            else:
                kk = first_return(occ)
                el, az = np.nonzero(kk >= 0)
                cells = np.stack([el, az, kk[el, az]], axis=1)
            if not len(cells):
                continue
            xyz = G.xyz(cells)
        h = xyz[:, 2] - ground_z
        if a.scene:
            w = s.world_xy(xyz[:, :2])                        # no re-centring: it is a pedestal fit
            m = (np.hypot(*(w - s.world_xy(s.c)).T) < a.radius) & (h > a.z_min) & (h < a.z_max)
            if not m.any():
                continue
            pts.append(w[m])
        else:
            m = (np.hypot(*(xyz[:, :2] - s.c).T) < a.radius) & (h > a.z_min) & (h < a.z_max)
            if not m.any():
                continue
            pts.append((xyz[m, :2] - s.c) @ s.T[:2, :2].T)
        hts.append(h[m])

    if a.source == "cfar":
        # every detection goes to the scan nearest it in time: that scan's refined centre and ground
        if not scans:
            raise SystemExit("no scan passed the pedestal fit")
        s_t = np.array([r[0] for r in scans]); s_c = np.array([r[1] for r in scans]); s_g = np.array([r[2] for r in scans])
        j = np.clip(np.searchsorted(s_t, cf_t), 0, len(s_t) - 1)
        j = np.where((j > 0) & (np.abs(s_t[np.maximum(j - 1, 0)] - cf_t) < np.abs(s_t[j] - cf_t)), j - 1, j)
        h = cf_xyz[:, 2] - s_g[j]
        d = cf_xyz[:, :2] - s_c[j]              # world frame already: the object frame's orientation is the world's
        m = (np.hypot(*d.T) < a.radius) & (h > a.z_min) & (h < a.z_max)
        n_frames = len(np.unique(rec["frame"]))
        pts, hts = [d[m]], [h[m]]

    if not pts:
        raise SystemExit("nothing landed on the object; check --centre, --threshold and --orbit")

    xy = np.concatenate(pts); h = np.concatenate(hts)
    if a.scene:
        c_ref = np.array([r[1] for r in scans]).mean(0)       # one shift, so the pedestal is the origin
        xy = xy - c_ref
        print(f"scene: world frame, shifted so the refined centre {c_ref.round(2)} is the origin")
    xyz = np.column_stack([xy, h])

    import matplotlib
    rgb = (matplotlib.colormaps["turbo"](np.clip(h / (2.5 if a.scene else 2.2), 0, 1))[:, :3] * 255).astype(np.uint8)

    n_raw = len(xyz)
    if a.voxel > 0:
        xyz, rgb = voxel_mean(xyz, rgb.astype(float), a.voxel)

    write_ply(a.out, xyz, rgb)
    print(f"{name} {a.orbit[0]:.0f}-{a.orbit[1]:.0f} s, source {a.source}: {used} of {len(sel)} scans fitted, "
          f"{n_frames} radar frames contributed")
    print(f"{n_raw:,} points -> {len(xyz):,} after {a.voxel * 100:.0f} cm voxels; "
          f"extent x {xyz[:, 0].min():+.2f}..{xyz[:, 0].max():+.2f}, y {xyz[:, 1].min():+.2f}..{xyz[:, 1].max():+.2f}, "
          f"height {xyz[:, 2].min():.2f}..{xyz[:, 2].max():.2f} m")
    print("saved", a.out, f"({os.path.getsize(a.out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
