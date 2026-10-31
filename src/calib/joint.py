"""Joint extrinsic calibration with the LiDAR frame as the world.

One least-squares problem over the radar pose and the camera pose relative to the LiDAR, fed by
every measurement of the seven board placements, each weighted by its own noise:

    radar range      (r_meas - |p_radar|)            / 0.02 m
    radar azimuth    (az_meas - atan2(-x, y))         / 2 deg        (+ = left)
    camera pixel     (uv_meas - fisheye(p_camera))    / 20 px
    radar elevation  not used (weight 0)

Unknowns: radar yaw, tx, ty, tz (pitch and roll fixed at 0: same mounting plate as the LiDAR,
and unobservable without elevation); camera rotation vector and translation (6). 10 unknowns,
28 equations. Radar -> camera then follows by composition, consistent by construction.
Parameter 1-sigma from the Jacobian at the solution.

    python -m src.calib.joint
"""
import json, os

import cv2
import numpy as np
from scipy.optimize import least_squares

from .board_image import project_fisheye
from .radar_camera import load_cam, CAM
from .lidar import LidarTrace
from .radar_lidar import SHOTS
from .transform import RadarToLidar

SIG_R, SIG_AZ, SIG_PX = 0.02, np.radians(2.0), 20.0


def gather(pix_json="calib/board_pixels.json", skip=("calib_06",)):
    """Per shot: LiDAR board point (lidar frame), radar (r, az), image (u, v)."""
    pix = json.load(open(pix_json)); T_rl = RadarToLidar.load()
    rows = []
    for name in SHOTS:
        if name in skip or name not in pix: continue
        p = pix[name]; r, az = p["radar"]["r"], p["radar"]["az"]
        d = os.path.join("data/raw", SHOTS[name]); lt = LidarTrace(d); xyz = lt.xyz(lt.index_at(p["t"]))
        pred = T_rl(np.array([[-r * np.sin(az), r * np.cos(az), 0.0]]))[0]
        near = (np.hypot(xyz[:, 0] - pred[0], xyz[:, 1] - pred[1]) < 0.5) & (xyz[:, 2] > -1.0) & (xyz[:, 2] < 0.6)
        if near.sum() < 10: continue
        rows.append(dict(name=name, P=xyz[near].mean(axis=0), r=r, az=az, uv=np.array(p["centre"])))
    return rows


def unpack(x):
    yaw, tx, ty, tz = x[:4]; rvec, tc = x[4:7], x[7:10]
    c, s = np.cos(yaw), np.sin(yaw)
    R_lr = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])        # lidar -> radar rotation (yaw only)
    R_lc, _ = cv2.Rodrigues(rvec)
    return R_lr, np.array([tx, ty, tz]), R_lc, tc


def residuals(x, rows, cam):
    R_lr, t_lr, R_lc, t_lc = unpack(x)
    out = []
    for s in rows:
        p = R_lr @ s["P"] + t_lr                                  # board in the radar frame
        out.append((s["r"] - np.linalg.norm(p)) / SIG_R)
        out.append(np.angle(np.exp(1j * (s["az"] - np.arctan2(-p[0], p[1])))) / SIG_AZ)
        q = R_lc @ s["P"] + t_lc                                  # board in the camera frame
        out.extend((s["uv"] - project_fisheye(q[None], cam)[0]) / SIG_PX)
    return np.array(out)


def run(out_json="calib/extrinsics_joint.json", fig_path="figures/calib_joint_residuals.png"):
    rows = gather(); cam, K, D = load_cam()
    # initial guess from the pairwise solves: radar<-lidar is the inverse of radar->lidar; camera<-lidar from its PnP
    T_rl = RadarToLidar.load(); lc = json.load(open("calib/extrinsics_lidar_camera.json"))
    R_lr0 = T_rl.R.T; t_lr0 = -R_lr0 @ T_rl.t
    x0 = np.r_[np.arctan2(R_lr0[1, 0], R_lr0[0, 0]), t_lr0, cv2.Rodrigues(np.array(lc["R"]))[0].ravel(), lc["t_m"]]
    res0 = residuals(x0, rows, cam)
    sol = least_squares(residuals, x0, args=(rows, cam), method="lm")
    x = sol.x; res = sol.fun
    # 1-sigma from the Jacobian (residuals are already in sigma units)
    JtJ = sol.jac.T @ sol.jac; cov = np.linalg.pinv(JtJ) * max(1.0, (res @ res) / max(len(res) - len(x), 1))
    sig = np.sqrt(np.diag(cov))
    R_lr, t_lr, R_lc, t_lc = unpack(x)
    n = len(rows); rr = res.reshape(n, 4)
    rng_cm, az_deg, px = rr[:, 0] * SIG_R * 100, np.degrees(rr[:, 1] * SIG_AZ), np.hypot(rr[:, 2], rr[:, 3]) * SIG_PX
    rr0 = res0.reshape(n, 4); rng0, az0, px0 = rr0[:, 0] * SIG_R * 100, np.degrees(rr0[:, 1] * SIG_AZ), np.hypot(rr0[:, 2], rr0[:, 3]) * SIG_PX
    # radar -> lidar and radar -> camera for use
    R_rl, t_rl = R_lr.T, -R_lr.T @ t_lr
    R_rc, t_rc = R_lc @ R_rl, R_lc @ t_rl + t_lc
    yaw_rl = np.degrees(np.arctan2(R_rl[1, 0], R_rl[0, 0]))
    print(f"joint solve: cost {0.5 * (res0 @ res0):.1f} -> {sol.cost:.1f} (sigma units), {len(res)} residuals, {len(x)} unknowns")
    print(f"radar->lidar: yaw {yaw_rl:+.2f}° ± {np.degrees(sig[0]):.2f}, t ({t_rl[0]:+.3f}, {t_rl[1]:+.3f}, {t_rl[2]:+.3f}) m ± ({sig[1]:.3f}, {sig[2]:.3f}, {sig[3]:.3f})")
    print(f"lidar->camera: rotvec {np.degrees(x[4:7]).round(2)}° ± {np.degrees(sig[4:7]).round(2)}, t {t_lc.round(3)} m ± {sig[7:10].round(3)}")
    print(f"radar->camera (composed): rotvec {np.degrees(cv2.Rodrigues(R_rc)[0].ravel()).round(2)}°, t {t_rc.round(3)} m")
    print(f"residuals: radar range {np.abs(rng_cm).mean():.1f} cm (pairwise start {np.abs(rng0).mean():.1f}), "
          f"radar azimuth {np.abs(az_deg).mean():.2f}° ({np.abs(az0).mean():.2f}), camera {px.mean():.1f} px ({px0.mean():.1f})")
    for s, a, b, c_ in zip(rows, rng_cm, az_deg, px): print(f"  {s['name']}: range {a:+.1f} cm, azimuth {b:+.2f}°, pixel {c_:.1f} px")
    json.dump(dict(world="lidar (sensor frame)", shots=[s["name"] for s in rows],
                   sigmas=dict(range_m=SIG_R, azimuth_deg=2.0, pixel_px=SIG_PX),
                   radar_to_lidar=dict(yaw_deg=float(yaw_rl), t_m=t_rl.tolist(), sigma=dict(yaw_deg=float(np.degrees(sig[0])), t_m=sig[1:4].tolist()),
                                       note="pitch, roll fixed at 0 (unobservable without radar elevation)"),
                   lidar_to_camera=dict(R=R_lc.tolist(), t_m=t_lc.tolist(), sigma=dict(rotvec_deg=np.degrees(sig[4:7]).tolist(), t_m=sig[7:10].tolist())),
                   radar_to_camera=dict(R=R_rc.tolist(), t_m=t_rc.tolist(), intrinsics=CAM, note="composed; consistent with the two above by construction"),
                   residuals=dict(range_cm=rng_cm.tolist(), azimuth_deg=az_deg.tolist(), pixel_px=px.tolist()),
                   residuals_pairwise_start=dict(range_cm=rng0.tolist(), azimuth_deg=az0.tolist(), pixel_px=px0.tolist())),
              open(out_json, "w"), indent=1)

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 5.4), constrained_layout=True)
    names = [s["name"][6:] for s in rows]; xs = np.arange(n); w = 0.38
    for a, before, after, title, unit, lim in [(ax[0], rng0, rng_cm, "a · radar range residual", "cm", 2), (ax[1], az0, az_deg, "b · radar azimuth residual", "deg", 2), (ax[2], px0, px, "c · camera pixel residual", "px", 20)]:
        a.bar(xs - w / 2, before, w, color="#B9C2CC", label="pairwise solves"); a.bar(xs + w / 2, after, w, color="#2C43B0", label="joint solve")
        a.axhline(0, color="#566373", lw=.8); a.axhspan(-lim, lim, color="#2C43B0", alpha=.07)
        a.set_xticks(xs); a.set_xticklabels(names); a.set_ylabel(unit); a.set_title(title); a.legend(fontsize=10)
    ax[0].text(.02, .95, "band = 1σ assumed", transform=ax[0].transAxes, va="top", fontsize=10, color="#566373")
    fig.suptitle(f"Joint calibration in the LiDAR frame: 10 unknowns, 28 weighted residuals · radar->lidar yaw {yaw_rl:+.1f}° ± {np.degrees(sig[0]):.1f}, "
                 f"t_z {t_rl[2]:+.2f} ± {sig[3]:.2f} m · camera {px.mean():.0f} px", fontsize=12)
    fig.savefig(fig_path, dpi=150); print("saved", out_json, fig_path)


if __name__ == "__main__":
    run()
