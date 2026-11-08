"""DA3 estimates its own cameras. We also know where the camera really was — compare the two.

    .venv/bin/python -m src.ext.obj_rgb.camera_track data/raw/2026-11-03.17-01-12 \
        --pred out/da3_scotty/da3_pred.npz --centre -13.07 -9.86 --out out/da3_scotty/camera_track.png

The real track needs no photogrammetry: the KISS-ICP LiDAR odometry of `data/poses/<trace>/lidar.npz`
composed with `lidar_to_camera` from the joint calibration (sigma 0.3 deg, 1.2 cm) puts the camera
centre in the world frame for any instant. DA3's track is only defined up to a similarity, so it is
fitted onto the real one for (scale, yaw, translation) in the horizontal plane and the residual is
what is left. That scale is the metre scale of the whole DA3 reconstruction, measured off the
trajectory rather than off the object — an estimate independent of `align_to_lidar`, which measures
the same number off the point cloud.

Both are worth having: if they agree, DA3's shape and its cameras are consistent, and the scale is
real rather than an artefact of whichever reference it was fitted to.
"""
import argparse, json, os, re

import numpy as np
from scipy.optimize import minimize

from ...calib.transform import joint_path


def camera_centres(trace, t_unix, poses_dir="data/poses"):
    """(N,) Unix times -> (N, 3) camera centres in the world frame of data/poses/<trace>."""
    name = os.path.basename(trace.rstrip("/"))
    L = np.load(os.path.join(poses_dir, name, "lidar.npz"))
    lc = json.load(open(joint_path(trace)))["lidar_to_camera"]
    R_lc, t_lc = np.array(lc["R"]), np.array(lc["t_m"])
    c_lid = -R_lc.T @ t_lc                                   # camera centre in the LiDAR frame
    j = np.array([int(np.argmin(np.abs(L["t"] - t))) for t in t_unix])
    T = L["T"][j]
    return np.einsum("nij,j->ni", T[:, :3, :3], c_lid) + T[:, :3, 3], lc["sigma"], c_lid


def fit_sim2(src, dst):
    """(scale, yaw, t) taking 2D src onto 2D dst, least squares (Umeyama)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    H = a.T @ b
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    s = S @ np.array([1, np.sign(np.linalg.det(Vt.T @ U.T))]) / (a ** 2).sum()
    return float(s), float(np.arctan2(R[1, 0], R[0, 0])), mu_d - s * (R @ mu_s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--pred", required=True, help="da3_pred.npz written by run_da3.py")
    ap.add_argument("--frames", default=None, help="frame dir; default: the one recorded in the npz")
    ap.add_argument("--centre", type=float, nargs=2, required=True, help="object's refined world xy")
    ap.add_argument("--poses-dir", default="data/poses")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    P = np.load(a.pred, allow_pickle=True)
    paths = [str(p) for p in P["frames"]]
    tsec = np.array([float(re.search(r"_t(\d+\.\d+)", os.path.basename(p)).group(1)) for p in paths])
    t0 = np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8")[0]

    true, sigma, c_lid = camera_centres(a.trace, t0 + tsec, a.poses_dir)
    c_ref = np.array(a.centre)
    true_xy = true[:, :2] - c_ref
    da3 = P["cams"]                                           # already levelled and object-centred
    print(f"{len(paths)} frames, {tsec[0]:.0f}..{tsec[-1]:.0f} s; lidar->camera sigma "
          f"{sigma['rotvec_deg'][0]:.2f} deg, {sigma['t_m'][0] * 100:.1f} cm; camera centre in the "
          f"LiDAR frame {np.round(c_lid, 3)}")

    r_true = np.hypot(*true_xy.T)
    ang = np.degrees(np.arctan2(true_xy[:, 1], true_xy[:, 0])) % 360
    turns = np.diff(np.unwrap(np.radians(ang)))
    print(f"\nreal track: distance to the object {r_true.min():.2f}..{r_true.max():.2f} m (mean {r_true.mean():.2f}), "
          f"world z spread {np.ptp(true[:, 2]):.2f} m")
    print(f"            {np.abs(turns).sum() * 180 / np.pi / 360:.1f} laps in all, net "
          f"{np.degrees(np.unwrap(np.radians(ang))[-1] - ang[0]):+.0f} deg "
          f"({(turns > 0).sum()} steps one way, {(turns < 0).sum()} the other)")

    s, yaw, t = fit_sim2(da3[:, :2], true_xy)
    c, z = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -z], [z, c]])
    q = s * da3[:, :2] @ R.T + t
    err = np.hypot(*(q - true_xy).T)
    print(f"\nDA3 track onto the real one: scale x{s:.3f}, yaw {np.degrees(yaw) % 360:.1f} deg")
    print(f"  per-camera error: median {np.median(err) * 100:.0f} cm, max {err.max() * 100:.0f} cm, "
          f"against a track {r_true.mean():.1f} m from the object")
    print(f"  DA3's height spread {np.ptp(da3[:, 2]):.2f} m against the real {np.ptp(true[:, 2]):.2f} m")
    print(f"\n  -> the reconstruction's metres are off by x{s:.3f} according to the cameras")

    out = dict(scale_from_cameras=s, yaw_deg=float(np.degrees(yaw) % 360),
               err_median_cm=float(np.median(err) * 100), err_max_cm=float(err.max() * 100),
               true_radius_m=[float(r_true.min()), float(r_true.mean()), float(r_true.max())],
               true_z_spread_m=float(np.ptp(true[:, 2])), da3_z_spread_m=float(np.ptp(da3[:, 2])),
               laps=float(np.abs(turns).sum() * 180 / np.pi / 360), n=len(paths))
    json.dump(out, open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15.5, 5.0), constrained_layout=True)

    for axx, (xy, lab, col) in zip([ax[0], ax[0]], [(true_xy, "calibrated (KISS-ICP + rig solve)", "#2C43B0"),
                                                    (q, "DA3, fitted onto it", "#E07B39")]):
        axx.plot(xy[:, 0], xy[:, 1], "-o", c=col, ms=4, lw=1.2, label=lab, alpha=.9)
    for i in range(len(q)):
        ax[0].plot([true_xy[i, 0], q[i, 0]], [true_xy[i, 1], q[i, 1]], c="#99A1AC", lw=.8, zorder=0)
    ax[0].add_patch(plt.Circle((0, 0), 0.39, color="#15202B", fill=False, ls="--"))
    ax[0].scatter([0], [0], marker="*", s=160, c="#15202B", zorder=5, label="the object")
    ax[0].set_aspect("equal"); ax[0].legend(fontsize=8.5); ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("y (m)")
    ax[0].set_title(f"a · where the camera was, top view\nDA3 off by {np.median(err) * 100:.0f} cm median "
                    f"after a x{s:.3f} scale", fontsize=10)

    ax[1].plot(tsec, r_true, "-o", c="#2C43B0", ms=4, lw=1.2, label="calibrated")
    ax[1].plot(tsec, np.hypot(*q.T), "-o", c="#E07B39", ms=4, lw=1.2, label="DA3, fitted")
    ax[1].axhline(2.6, color="#C23B22", ls=":", lw=1.4, label="2.6 m, the radius I guessed")
    ax[1].axhline(r_true.mean(), color="#15202B", ls="--", lw=1, label=f"true mean {r_true.mean():.2f} m")
    ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("distance to the object (m)"); ax[1].legend(fontsize=8)
    ax[1].set_title("b · it is not an orbit: two laps, opposite ways,\nthe radius wandering 1.9–4.8 m", fontsize=10)

    un = np.degrees(np.unwrap(np.radians(ang)))            # unwrapped: a lap is a full rise or fall
    ax[2].plot(tsec, un, "-o", c="#2C43B0", ms=4, lw=1.2)
    ax[2].axhline(un[0], color="#99A1AC", lw=.8)
    ax[2].axhline(un[0] - 360, color="#99A1AC", lw=.8, ls=":")
    ax[2].annotate("one lap clockwise", (tsec[8], un[8]), fontsize=8.5, xytext=(6, 10),
                   textcoords="offset points", color="#2C43B0")
    ax[2].annotate("then one back", (tsec[25], un[25]), fontsize=8.5, xytext=(-80, -14),
                   textcoords="offset points", color="#2C43B0")
    ax[2].set_xlabel("t (s)"); ax[2].set_ylabel("azimuth about the object, unwrapped (deg)")
    ax[2].set_title(f"c · azimuth against time: {np.abs(turns).sum() * 180 / np.pi / 360:.1f} laps, "
                    f"net {np.degrees(np.unwrap(np.radians(ang))[-1] - ang[0]):+.0f} deg", fontsize=10)

    fig.suptitle("DA3's own cameras against the calibrated ones — the scale the reconstruction should have had",
                 fontsize=12)
    fig.savefig(a.out, dpi=120)
    print("saved", a.out)


if __name__ == "__main__":
    main()
