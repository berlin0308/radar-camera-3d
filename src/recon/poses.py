"""LiDAR odometry with KISS-ICP over a time window, then poses at radar timestamps.

Writes data/poses/<trace>/lidar.npz  (t [N], T [N,4,4]: lidar frame -> world, world = first scan's frame)
       data/poses/<trace>/radar.npz  (t [M], T [M,4,4]: same transform sampled at radar frame times in the window)
and a figure with the trajectory, the start-to-end gap, and the LiDAR map built with these poses.

    python -m src.recon.poses data/raw/2026-10-29.15-49-40 --start 660 --seconds 360 --out figures/poses_jeep.png

KISS-ICP is odometry only (no loop closure), so the start-to-end gap on a closed orbit is the drift.

Deskew (--deskew gyro, the default): each scan is un-skewed with the IMU's z gyro before registration.
Column j of the range image was measured at ts + j/2048 * 0.1 s (the Ouster timestamp is the scan start);
its points are rotated about z by -(psi(t_col) - psi(t_mid)), psi = integrated gyro yaw, so the scan looks
as if taken at mid-scan. Sign and timestamp convention were picked on the 700-730 s swing of the Jeep trace
(98 deg/s): yaw error vs the gyro 4 deg at the end, 24 deg peak; every other combination, KISS-ICP's own
constant-velocity deskew (--deskew kiss, +98 deg, path doubled by jitter) and no deskew (--deskew none,
-208 deg) fail there. Translation within a scan (~10 cm at walking speed) is not compensated.

Map: max_range 30 m, voxel 0.3 m. With KISS-ICP's defaults (50 m, 0.5 m) the same swing, run from 660 s,
still ends 267 deg off the gyro: the sparse far returns of the open lot steer the registration. 30 m /
0.3 m brings it to 15 deg with no jumps (660-740 s test); the correspondence threshold settings do not help.
"""
import argparse, os, time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ..calib.lidar import LidarTrace


SCAN_PERIOD = 0.1       # s, one revolution


def gyro_yaw(trace):
    """(t, psi): integrated z angular rate of the rig IMU, radians, same clock as the LiDAR."""
    d = os.path.join(trace, "imu"); t = np.fromfile(os.path.join(d, "ts"), "<f8"); av = np.fromfile(os.path.join(d, "avel"), "<f4").reshape(-1, 3)
    return t, np.cumsum(av[:, 2] * np.gradient(t))


def deskew_gyro(xyz, t_col, t_mid, gyro):
    """Rotate each point about z so the scan looks as if taken at t_mid (rotation only)."""
    t_g, psi = gyro
    dpsi = -(np.interp(t_col, t_g, psi) - np.interp(t_mid, t_g, psi))
    c, s = np.cos(dpsi), np.sin(dpsi); x, y = xyz[:, 0].copy(), xyz[:, 1].copy()
    xyz[:, 0] = c * x - s * y; xyz[:, 1] = s * x + c * y
    return xyz


def run_kiss(lt, scans, max_range=30.0, min_range=0.5, voxel=0.3, deskew="gyro", gyro=None):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    cfg = KISSConfig(); cfg.data.max_range = max_range; cfg.data.min_range = min_range; cfg.data.deskew = deskew == "kiss"
    cfg.mapping.voxel_size = voxel if voxel is not None else max_range / 100.0     # the pipeline's default rule; the class needs it explicit
    odom = KissICP(cfg)
    col_frac = (np.arange(lt.shape[1]) / lt.shape[1])[None, :]
    poses = []
    for k in scans:
        r = lt.range_image(k); ok = r > min_range
        xyz = lt.xyz(k, r_min=min_range)                                   # same geometry as everything else (encoder - beam offset)
        frac = np.broadcast_to(col_frac, r.shape)[ok]
        if deskew == "gyro":
            xyz = deskew_gyro(xyz, lt.ts[k] + frac * SCAN_PERIOD, lt.ts[k] + 0.5 * SCAN_PERIOD, gyro)
        odom.register_frame(xyz.astype(np.float64), frac.astype(np.float64))
        poses.append(odom.last_pose.copy())
    return np.array(poses)


def interpolate(t_src, T_src, t_dst):
    """Poses at t_dst from poses at t_src: linear position, slerp rotation. Outside the range: clamped."""
    t_dst = np.clip(t_dst, t_src[0], t_src[-1])
    pos = np.stack([np.interp(t_dst, t_src, T_src[:, i, 3]) for i in range(3)], axis=-1)
    rot = Slerp(t_src, Rotation.from_matrix(T_src[:, :3, :3]))(t_dst).as_matrix()
    T = np.tile(np.eye(4), (len(t_dst), 1, 1)); T[:, :3, :3] = rot; T[:, :3, 3] = pos
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--start", type=float, default=0.0); ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--out", required=True); ap.add_argument("--poses-dir", default="data/poses"); ap.add_argument("--deskew", choices=["gyro", "kiss", "none"], default="gyro")
    a = ap.parse_args()
    lt = LidarTrace(a.trace); t0 = lt.ts[0] + a.start; t1 = t0 + a.seconds if a.seconds else lt.ts[-1]
    scans = np.nonzero((lt.ts >= t0) & (lt.ts <= t1))[0]
    tic = time.perf_counter(); T = run_kiss(lt, scans, deskew=a.deskew, gyro=gyro_yaw(a.trace) if a.deskew == "gyro" else None); dt = time.perf_counter() - tic
    t_l = lt.ts[scans]
    name = os.path.basename(a.trace.rstrip("/")); d = os.path.join(a.poses_dir, name); os.makedirs(d, exist_ok=True)
    np.savez(os.path.join(d, "lidar.npz"), t=t_l, T=T, start=a.start, seconds=a.seconds or 0.0, deskew=a.deskew)
    ts_r = np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8"); m = (ts_r >= t_l[0]) & (ts_r <= t_l[-1])
    T_r = interpolate(t_l, T, ts_r[m]); np.savez(os.path.join(d, "radar.npz"), t=ts_r[m], T=T_r, index=np.nonzero(m)[0])
    pos = T[:, :3, 3]; path = np.linalg.norm(np.diff(pos, axis=0), axis=1).sum(); gap = np.linalg.norm(pos[-1] - pos[0])
    yaw = np.degrees(np.unwrap(Rotation.from_matrix(T[:, :3, :3]).as_euler("zyx")[:, 0]))
    # IMU check: the IMU's own orientation estimate, yaw change over the same window
    imu_ts = np.fromfile(os.path.join(a.trace, "imu", "ts"), "<f8"); imu_rot = np.fromfile(os.path.join(a.trace, "imu", "rot"), "<f4").reshape(-1, 3)
    i0, i1 = np.searchsorted(imu_ts, [t_l[0], t_l[-1]]); imu_yaw = np.degrees(np.unwrap(np.radians(imu_rot[i0:i1, 2]) if np.abs(imu_rot).max() > 7 else imu_rot[i0:i1, 2]))
    print(f"{name}: {len(scans)} scans over {t_l[-1]-t_l[0]:.0f} s in {dt:.0f} s ({len(scans)/dt:.1f} scans/s); path {path:.1f} m, "
          f"start-to-end gap {gap:.2f} m ({gap/max(path,1e-6)*100:.1f}% of path), yaw turned {yaw[-1]-yaw[0]:+.0f} deg (IMU says {imu_yaw[-1]-imu_yaw[0]:+.0f} deg); "
          f"{m.sum()} radar frames with poses -> {d}")

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 5.4), constrained_layout=True, gridspec_kw=dict(width_ratios=[1, 1.2, 1]))
    sc = ax[0].scatter(pos[:, 0], pos[:, 1], c=t_l - t_l[0], cmap="viridis", s=4); ax[0].scatter(*pos[0, :2], marker="^", s=120, color="#2C43B0", label="start"); ax[0].scatter(*pos[-1, :2], marker="s", s=90, color="#E07B39", label="end")
    ax[0].set_aspect("equal"); ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)"); ax[0].legend(fontsize=10); ax[0].set_title(f"a · KISS-ICP trajectory, {len(scans)} scans, gap {gap:.2f} m", fontsize=12); fig.colorbar(sc, ax=ax[0], shrink=.7, label="s")
    # map: every 10th scan, above-ground points, downsampled, in the world frame
    pts = []
    for j in range(0, len(scans), 10):
        xyz = lt.xyz(scans[j]); xyz = xyz[(xyz[:, 2] > -0.9) & (np.linalg.norm(xyz[:, :2], axis=1) < 25)][::7]
        pts.append(xyz @ T[j, :3, :3].T + T[j, :3, 3])
    pts = np.concatenate(pts)
    ax[1].scatter(pts[:, 0], pts[:, 1], s=.2, c=np.clip(pts[:, 2], -1, 3), cmap="viridis"); ax[1].plot(pos[:, 0], pos[:, 1], color="#E07B39", lw=1)
    ax[1].set_aspect("equal"); ax[1].set_xlim(pos[:, 0].min() - 20, pos[:, 0].max() + 20); ax[1].set_ylim(pos[:, 1].min() - 20, pos[:, 1].max() + 20); ax[1].set_title("b · LiDAR map with these poses (above ground, every 10th scan)", fontsize=12)
    ax[2].plot(t_l - t_l[0], yaw - yaw[0], color="#2C43B0", label="KISS-ICP yaw"); ax[2].plot(imu_ts[i0:i1] - t_l[0], imu_yaw - imu_yaw[0], color="#E07B39", lw=1, label="IMU yaw")
    ax[2].set_xlabel("time (s)"); ax[2].set_ylabel("yaw change (deg)"); ax[2].legend(fontsize=10); ax[2].set_title("c · heading: odometry vs IMU", fontsize=12)
    fig.suptitle(f"{name} · {a.start:.0f}–{a.start + (t_l[-1]-t_l[0]):.0f} s · path {path:.0f} m", fontsize=12)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=130); print("saved", a.out)


if __name__ == "__main__":
    main()
