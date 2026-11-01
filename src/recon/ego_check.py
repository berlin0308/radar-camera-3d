"""Does the Doppler of the static world agree with the LiDAR odometry? One figure per window.

    python -m src.recon.ego_check data/raw/2026-10-29.15-49-40 --start 660 --seconds 360 --frames 750 965 --out figures/ego_check_jeep.png

a, b: the Doppler-azimuth power of one frame (summed over range beyond 1.5 m) with the static-world curve
      v = -s cos(az - phi) from the radar's own ridge fit (radar.ego.estimate_ego_dense) and from the odometry
      velocity rotated into the radar frame, plus the CFAR detections of that frame.
c:    speed over the window, radar ridge vs odometry; d: heading, same.
"""
import argparse, os

import numpy as np
from scipy.ndimage import median_filter

from ..radar import RadarTrace, detect
from ..radar.ego import estimate_ego_dense, ego_curve
from ..radar.angle import AZIMUTHS
from ..calib.transform import RadarToLidar


def pose_velocity_radar(P, R_rl, smooth=21):
    """Sensor velocity in the radar frame for every radar-time pose: (N, 3)."""
    pos = P["T"][:, :3, 3]; vel_w = np.gradient(pos, P["t"], axis=0, edge_order=1)
    vel_w = median_filter(vel_w, size=(smooth, 1), mode="nearest")
    vel_w = np.stack([np.convolve(vel_w[:, i], np.ones(5) / 5, mode="same") for i in range(3)], axis=-1)
    return np.einsum("ij,nj->ni", R_rl.T, np.einsum("nji,nj->ni", P["T"][:, :3, :3], vel_w))   # world -> lidar -> radar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--start", type=float, default=0.0); ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--frames", type=float, nargs=2, default=(750, 965)); ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--poses-dir", default="data/poses"); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    name = os.path.basename(a.trace.rstrip("/")); P = np.load(os.path.join(a.poses_dir, name, "radar.npz")); tr = RadarTrace(a.trace); R_rl = RadarToLidar.load_joint().R
    t0 = tr.ts[0] + a.start; t1 = t0 + a.seconds if a.seconds else np.inf
    sel = np.nonzero((P["t"] >= t0) & (P["t"] < t1))[0]; v_r = pose_velocity_radar(P, R_rl)
    vb = tr.params.velocities; v_res = tr.params.velocity_resolution

    rows = []
    for n in sel[::a.stride]:
        _, info = detect(tr.frame(P["index"][n]), tr.params)
        s, phi, sup = estimate_ego_dense(info["spec_az"], vb, AZIMUTHS)
        rows.append((P["t"][n] - tr.ts[0], s, np.degrees(phi), sup, np.hypot(*v_r[n, :2]), np.degrees(np.arctan2(-v_r[n, 0], v_r[n, 1]))))
    R = np.array(rows); moving = R[:, 4] > 0.3
    agree = np.abs(R[moving, 1] - R[moving, 4]) < 0.15
    print(f"{name} {a.start:.0f}-{a.start + (a.seconds or 0):.0f} s: {len(R)} frames checked, {moving.sum()} moving (> 0.3 m/s); "
          f"ridge speed within 0.15 m/s of the odometry on {agree.mean()*100:.0f}% of them, median |diff| {np.median(np.abs(R[moving, 1] - R[moving, 4])):.2f} m/s; "
          f"heading median |diff| {np.median(np.abs((R[moving, 2] - R[moving, 5] + 180) % 360 - 180)):.0f} deg (frames with ridge support > 0.5: "
          f"{np.median(np.abs((R[moving & (R[:, 3] > .5), 2] - R[moving & (R[:, 3] > .5), 5] + 180) % 360 - 180)):.0f} deg)")

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(16.64, 9.6), constrained_layout=True)
    az_deg = np.degrees(AZIMUTHS)
    for axx, t_rel, lab in zip(ax[0], a.frames, "ab"):
        n = sel[np.argmin(np.abs(P["t"][sel] - (tr.ts[0] + t_rel)))]; k = P["index"][n]
        pts, info = detect(tr.frame(k), tr.params); pts = pts[pts["snr"] > 10]
        Pw = (np.abs(info["spec_az"][..., 17:235]) ** 2).sum(-1); Pw = 10 * np.log10(Pw / Pw.max() + 1e-9)
        s, phi, sup = estimate_ego_dense(info["spec_az"], vb, AZIMUTHS)
        axx.imshow(Pw, aspect="auto", origin="lower", extent=[az_deg[0], az_deg[-1], vb[0], vb[-1]], cmap="magma", vmin=-35, vmax=0)
        axx.plot(az_deg, ego_curve(AZIMUTHS, s, phi), color="#5BC0EB", lw=2, label=f"radar ridge fit: {s:.2f} m/s, heading {np.degrees(phi):+.0f}°, support {sup:.2f}")
        sp, hd = np.hypot(*v_r[n, :2]), np.arctan2(-v_r[n, 0], v_r[n, 1])
        axx.plot(az_deg, ego_curve(AZIMUTHS, sp, hd), "--", color="#F5F5F5", lw=1.5, label=f"from LiDAR odometry: {sp:.2f} m/s, heading {np.degrees(hd):+.0f}°")
        axx.scatter(np.degrees(pts["az"]), pts["v"], s=28, facecolors="none", edgecolors="#9BE564", lw=1.2, label=f"CFAR detections, snr > 10 ({len(pts)})")
        axx.set_xlabel("azimuth (deg, + left)"); axx.set_ylabel("Doppler velocity (m/s, approaching < 0)"); axx.legend(fontsize=9, loc="upper right")
        axx.set_title(f"{lab} · t = {t_rel:.0f} s · Doppler-azimuth power (sum over range > 1.5 m) and the static-world curve", fontsize=11)
    ax[1, 0].plot(R[:, 0], R[:, 4], color="#2C43B0", lw=1.2, label="LiDAR odometry (KISS-ICP, 1 s median)"); ax[1, 0].plot(R[:, 0], R[:, 1], ".", ms=3, color="#E07B39", label="radar ridge fit")
    ax[1, 0].set_ylabel("speed (m/s)"); ax[1, 0].set_xlabel("time in trace (s)"); ax[1, 0].legend(fontsize=9); ax[1, 0].set_title(f"c · speed, {agree.mean()*100:.0f}% of moving frames within 0.15 m/s", fontsize=11)
    ax[1, 1].plot(R[:, 0], R[:, 5], color="#2C43B0", lw=1.2, label="LiDAR odometry"); ax[1, 1].plot(R[moving, 0], R[moving, 2], ".", ms=3, color="#E07B39", label="radar ridge fit (moving frames)")
    ax[1, 1].set_ylabel("heading in the radar frame (deg, + left)"); ax[1, 1].set_xlabel("time in trace (s)"); ax[1, 1].set_ylim(-180, 180); ax[1, 1].legend(fontsize=9); ax[1, 1].set_title("d · heading of the motion in the radar frame", fontsize=11)
    fig.suptitle(f"{name} · {a.start:.0f}–{a.start + (a.seconds or 0):.0f} s · static-world Doppler vs LiDAR odometry", fontsize=12)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=130); print("saved", a.out)


if __name__ == "__main__":
    main()
