"""Accumulate radar points in the world frame with the KISS-ICP poses, next to the LiDAR built the same way.

    python -m src.recon.accumulate data/raw/2026-10-29.15-49-40 --start 660 --seconds 360 --out figures/accum_jeep.png

Radar points: CFAR per frame -> static-world filter -> radar frame -> LiDAR frame (joint calibration)
-> world (radar pose interpolated from the LiDAR odometry). z is set to 0 in the radar frame (unusable).
The static filter uses the sensor velocity from the odometry (finite difference of the radar-time poses,
median-filtered over 1 s against ICP jitter, rotated into the radar frame): a static target at azimuth az
must show Doppler -v_sensor . u(az) (approaching = negative on our Doppler axis, see radar.ego).
The LiDAR reference is built with the same poses: per scan, the operator and the rig (within 1.5 m of
the sensor) are dropped and only the layer 0.3-2.5 m above the local ground, within 15 m, is kept (the
radar, z unusable, sees roughly that layer). Ground per scan, in the sensor frame: a plane fitted to the
low points within 10 m (the handheld rig is tilted, so the ground is a slope in the sensor frame), then
per 2 m x 2 m cell the 5th percentile of the residual to that plane (a raised lawn next to a plaza gets
its own offset). Per scan because the odometry's z and pitch jitter smears the ground into a 0.5 m slab
in the world frame, where a height cut no longer separates it. Everything is judged within 15 m of the path. Two numbers per run: distance from each radar point (within 15 m of the path) to the
nearest LiDAR point of the layer (xy), and coverage = share of 0.5 m LiDAR cells that have a radar point within 0.5 m.
Writes data/recon/<trace>/radar_points.npz (world xyz, snr, frame, kept-by-filter flag) and lidar_points.npz.
--dense also writes dense_power.npz: the range-azimuth power map of every frame (dB, float16, [N, 64 az, 256 range],
with frame index, time and the axes) for a reconstruction that works from the full map rather than CFAR points;
the pose of frame i is radar.npz T at that index, and RANGE_OFFSET still applies to the range axis.
"""
import argparse, os

import numpy as np
from scipy.ndimage import median_filter

from ..radar import RadarTrace, detect, to_xyz
from ..radar.ego import static_mask_from_velocity, estimate_ego_dense
from ..radar.angle import AZIMUTHS
from ..calib.lidar import LidarTrace
from ..calib.transform import RadarToLidar


def height_above_ground(xyz, z_max=-0.8, rho_max=10.0, tol=0.15, cell=2.0, min_points=20):
    """Height of every point of one scan above the local ground (sensor frame, z up, sensor ~1.7 m up).
    A plane z = a x + b y + c is fitted by least squares to the low points within rho_max (inliers
    re-selected twice), then each cell x cell column with at least min_points points subtracts the
    5th percentile of its residual, so a terrace or raised lawn is ground too."""
    rho = np.linalg.norm(xyz[:, :2], axis=1); cand = (xyz[:, 2] < z_max) & (rho < rho_max)
    A = np.c_[xyz[:, :2], np.ones(len(xyz))]
    for _ in range(3):
        coef = np.linalg.lstsq(A[cand], xyz[cand, 2], rcond=None)[0]
        resid = xyz[:, 2] - A @ coef; cand = (np.abs(resid) < tol) & (rho < rho_max)
    ij = np.floor(xyz[:, :2] / cell).astype(np.int64); key = ij[:, 0] * 1_000_003 + ij[:, 1]
    order = np.argsort(key); ks, r = key[order], resid[order]
    cut = np.flatnonzero(np.diff(ks)) + 1; ground = np.zeros(len(ks))
    for s, e in zip(np.r_[0, cut], np.r_[cut, len(ks)]):
        if e - s >= min_points: ground[s:e] = max(np.percentile(r[s:e], 5), 0.0)      # never below the plane
    h = np.empty(len(xyz)); h[order] = r - ground
    return h


def coverage(lid_xy, radar_xy, path_xy, cell=0.5, reach=15.0, tol=0.5):
    """Share of occupied LiDAR cells within reach of the path that have a radar point within tol."""
    from scipy.spatial import cKDTree
    cells = np.unique(np.floor(lid_xy / cell).astype(np.int64), axis=0) * cell + cell / 2
    cells = cells[cKDTree(path_xy).query(cells)[0] < reach]
    hit = cKDTree(radar_xy).query(cells)[0] < tol
    return hit.mean(), len(cells)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--start", type=float, default=0.0); ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--poses-dir", default="data/poses"); ap.add_argument("--out", required=True); ap.add_argument("--snr", type=float, default=10)
    ap.add_argument("--lidar-stride", type=int, default=5); ap.add_argument("--dense", action="store_true")
    a = ap.parse_args()
    name = os.path.basename(a.trace.rstrip("/")); P = np.load(os.path.join(a.poses_dir, name, "radar.npz")); L = np.load(os.path.join(a.poses_dir, name, "lidar.npz"))
    tr = RadarTrace(a.trace); lt = LidarTrace(a.trace); T_rl = RadarToLidar.load_joint()
    t0 = tr.ts[0] + a.start; t1 = t0 + a.seconds if a.seconds else np.inf
    sel = (P["t"] >= t0) & (P["t"] < t1); idx, T_w, t_w = P["index"][sel], P["T"][sel], P["t"][sel]
    # sensor velocity in the world frame from the poses: finite difference, median over 21 frames (1 s) to kill
    # ICP jitter spikes, then a 5-frame mean; walking speed changes slower than that
    pos = T_w[:, :3, 3]; vel_w = np.gradient(pos, t_w, axis=0, edge_order=1)
    vel_w = median_filter(vel_w, size=(21, 1), mode="nearest")
    vel_w = np.stack([np.convolve(vel_w[:, i], np.ones(5) / 5, mode="same") for i in range(3)], axis=-1)
    R_rl = T_rl.R                                                              # radar -> lidar rotation
    world, snr, frame, kept, ego, dense = [], [], [], [], [], []
    for n, (k, T) in enumerate(zip(idx, T_w)):
        pts, info = detect(tr.frame(k), tr.params); pts = pts[pts["snr"] > a.snr]
        if a.dense: dense.append((10 * np.log10(info["power"] + 1e-12)).astype(np.float16))
        if len(pts) == 0: continue
        v_lidar = T[:3, :3].T @ vel_w[n]; v_radar = R_rl.T @ v_lidar                  # world -> lidar -> radar frame
        m = static_mask_from_velocity(pts, v_radar[:2], v_res=tr.params.velocity_resolution)
        s_dop, phi_dop, sup = estimate_ego_dense(info["spec_az"], tr.params.velocities, AZIMUTHS)   # Doppler-only cross-check
        xyz = to_xyz(pts); xyz[:, 2] = 0.0
        xyz_w = T_rl(xyz) @ T[:3, :3].T + T[:3, 3]
        world.append(xyz_w); snr.append(pts["snr"]); frame.append(np.full(len(pts), k)); kept.append(m)
        ego.append((t_w[n], np.hypot(*v_radar[:2]), np.arctan2(-v_radar[0], v_radar[1]), s_dop, phi_dop, sup, len(pts)))
    world, snr, frame, kept = map(np.concatenate, (world, snr, frame, kept)); ego = np.array(ego)
    lid, lid_h = [], []
    for j in range(0, len(L["t"]), a.lidar_stride):
        if not (t0 <= L["t"][j] < t1): continue
        kl = lt.index_at(L["t"][j]); xyz = lt.xyz(kl); rho = np.linalg.norm(xyz[:, :2], axis=1); xyz = xyz[(rho > 1.5) & (rho < 15)]   # no operator, no rig, near field
        h = height_above_ground(xyz); keep = (h > 0.3) & (h < 2.5); xyz = xyz[keep][::5]; lid_h.append(h[keep][::5])
        lid.append(xyz @ L["T"][j, :3, :3].T + L["T"][j, :3, 3])
    lid = np.concatenate(lid); lid_h = np.concatenate(lid_h)
    d = os.path.join("data/recon", name); os.makedirs(d, exist_ok=True)
    np.savez(os.path.join(d, "radar_points.npz"), xyz=world, snr=snr, frame=frame, static=kept, ego=ego)
    np.savez(os.path.join(d, "lidar_points.npz"), xyz=lid, height=lid_h)
    if a.dense:
        from ..radar.pointcloud import RANGE_OFFSET
        np.savez(os.path.join(d, "dense_power.npz"), db=np.stack(dense), frame=idx, t=t_w, azimuths=AZIMUTHS, ranges=tr.params.ranges - RANGE_OFFSET)
        print(f"dense range-azimuth power of {len(dense)} frames -> {d}/dense_power.npz")
    moving = ego[:, 1] > 0.15
    print(f"{name}: {len(idx)} radar frames, {len(world)} CFAR points, {kept.mean()*100:.0f}% pass the static filter; "
          f"odometry speed median {np.median(ego[:, 1]):.2f} m/s; Doppler-only estimate agrees within 0.1 m/s on "
          f"{(np.abs(ego[moving, 3] - ego[moving, 1]) < 0.1).mean()*100:.0f}% of moving frames; LiDAR reference {len(lid)} points -> {d}")

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from scipy.spatial import cKDTree
    near = cKDTree(T_w[:, :2, 3]).query(world[:, :2])[0] < 15.0                      # radar points with a LiDAR reference
    tree = cKDTree(lid[:, :2])
    d_all = tree.query(world[near, :2])[0]; d_keep = tree.query(world[near & kept, :2])[0]
    cov_all, n_cells = coverage(lid[:, :2], world[:, :2], T_w[:, :2, 3]); cov_keep, _ = coverage(lid[:, :2], world[kept, :2], T_w[:, :2, 3])
    print(f"nearest LiDAR point (xy, 0.3-2.5 m layer, {near.mean()*100:.0f}% of radar points within 15 m of the path): all CFAR median {np.median(d_all):.2f} m, within 0.5 m {(d_all < .5).mean()*100:.0f}%; "
          f"static-filtered median {np.median(d_keep):.2f} m, within 0.5 m {(d_keep < .5).mean()*100:.0f}% | "
          f"coverage of the {n_cells} LiDAR cells (0.5 m) within 15 m of the path: all {cov_all*100:.0f}%, static-filtered {cov_keep*100:.0f}%")
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 5.8), constrained_layout=True)
    lim = [world[:, 0].min() - 5, world[:, 0].max() + 5, world[:, 1].min() - 5, world[:, 1].max() + 5]
    for axx, m, title in [(ax[0], np.ones(len(world), bool), f"a · all CFAR points, {len(world)}"), (ax[1], kept, f"b · after the static-world filter, {kept.sum()}")]:
        axx.scatter(lid[:, 0], lid[:, 1], s=.2, c="#C9D1DA"); axx.scatter(world[m, 0], world[m, 1], s=3, c=snr[m], cmap="magma", vmin=5, vmax=30)
        axx.set_aspect("equal"); axx.set_xlim(lim[:2]); axx.set_ylim(lim[2:]); axx.set_title(title, fontsize=12)
    ax[2].scatter(lid[:, 0], lid[:, 1], s=.2, c=lid_h, cmap="viridis", vmin=0.3, vmax=2.5); ax[2].set_aspect("equal"); ax[2].set_xlim(lim[:2]); ax[2].set_ylim(lim[2:]); ax[2].set_title(f"c · LiDAR, same poses, 0.3–2.5 m above ground (colour = height), {len(lid)} points", fontsize=12)
    fig.suptitle(f"{name} {a.start:.0f}–{a.start + (a.seconds or 0):.0f} s · world frame from KISS-ICP · radar via joint calibration, z = 0 · nearest LiDAR median {np.median(d_all):.2f} -> {np.median(d_keep):.2f} m · coverage {cov_all*100:.0f} -> {cov_keep*100:.0f}%", fontsize=12)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=130); print("saved", a.out)


if __name__ == "__main__":
    main()
