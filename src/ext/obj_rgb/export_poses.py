"""Write the calibrated camera poses for a set of frames, for a model that can be told where it was.

    .venv/bin/python -m src.ext.obj_rgb.export_poses data/raw/2026-11-03.17-01-12 \
        --frames out/da3_scotty/frames --centre -13.20 -9.87 --orbit 30 165 \
        --pedestal-radius 0.39 --out out/da3_scotty/poses.npz

DA3's `inference()` takes `extrinsics` (N, 4, 4 world-to-camera) and `intrinsics` (N, 3, 3, for the
image as given — it rescales them itself), so pose-free is a default, not the only mode. Handing it
the poses we already have makes the reconstruction metric by construction: no Sim3 afterwards, and
the LiDAR is free to be an independent reference instead of also being the ruler.

The pose of a frame is the KISS-ICP LiDAR odometry at its timestamp composed with `lidar_to_camera`
from the joint calibration (sigma 0.29 deg, 1.2 cm). Both use the camera convention x right, y down,
z forward, so nothing needs flipping.

Also written, so the result can be put in the same object frame as the LiDAR and radar clouds:
the refined pedestal centre in world xy, and the ground's world z near the object, both from the same
fixed-radius circle fit `src.ext.obj_lidar.export_ply` uses. And the fisheye parameters, because the
lens is nowhere near a pinhole: k1..k4 are small but the model is equidistant, so a point at the
corner lands 273 px (16 px at DA3's working size) from where a pinhole would put it. The frames must
be undistorted before intrinsics mean anything.
"""
import argparse, json, os, re

import numpy as np

from ...calib.lidar import LidarTrace
from ...calib.transform import joint_path
from ...recon.closeup import Scan

CAM = "calib/camera_fisheye_iq1m.json"


def frame_times(frames_dir, t0):
    """Frame files named <i>_t<seconds>.png -> (paths, Unix times)."""
    names = sorted(f for f in os.listdir(frames_dir) if f.lower().endswith((".png", ".jpg", ".jpeg")))
    sec = []
    for f in names:
        m = re.search(r"_t(\d+(?:\.\d+)?)", f)
        if not m:
            raise SystemExit(f"{f}: expected a name like 000_t0030.0.png")
        sec.append(float(m.group(1)))
    return [os.path.join(frames_dir, f) for f in names], t0 + np.array(sec), np.array(sec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--frames", required=True)
    ap.add_argument("--centre", type=float, nargs=2, required=True)
    ap.add_argument("--orbit", type=float, nargs=2, required=True)
    ap.add_argument("--slice", type=float, nargs=2, default=(0.45, 1.0))
    ap.add_argument("--pedestal-radius", type=float, default=None)
    ap.add_argument("--stride", type=int, default=8, help="LiDAR scans used for the centre and ground")
    ap.add_argument("--poses-dir", default="data/poses")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    name = os.path.basename(a.trace.rstrip("/"))
    L = np.load(os.path.join(a.poses_dir, name, "lidar.npz"))
    t0 = np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8")[0]
    paths, t_unix, tsec = frame_times(a.frames, t0)

    # --- camera poses: world -> camera
    lc = json.load(open(joint_path(a.trace)))["lidar_to_camera"]
    R_lc, t_lc = np.array(lc["R"]), np.array(lc["t_m"])
    T_lc = np.eye(4); T_lc[:3, :3], T_lc[:3, 3] = R_lc, t_lc
    j = np.array([int(np.argmin(np.abs(L["t"] - t))) for t in t_unix])
    dt = np.abs(L["t"][j] - t_unix)
    w2c = np.stack([T_lc @ np.linalg.inv(L["T"][k]) for k in j])
    print(f"{len(paths)} frames, {tsec[0]:.0f}..{tsec[-1]:.0f} s; nearest LiDAR scan within "
          f"{dt.max() * 1000:.0f} ms; lidar->camera sigma {lc['sigma']['rotvec_deg'][0]:.2f} deg, "
          f"{lc['sigma']['t_m'][0] * 100:.1f} cm")
    c2w = np.linalg.inv(w2c)
    print(f"camera centres span x {c2w[:, 0, 3].min():.1f}..{c2w[:, 0, 3].max():.1f}, "
          f"y {c2w[:, 1, 3].min():.1f}..{c2w[:, 1, 3].max():.1f}, z {c2w[:, 2, 3].min():.2f}..{c2w[:, 2, 3].max():.2f} m")

    # --- the object frame: refined centre and the ground beside it, the same fit the LiDAR export uses
    lt = LidarTrace(a.trace)
    c_guess = np.array(a.centre)
    radius = a.pedestal_radius or Scan(lt, L, t0 + a.orbit[0], c_guess, None, a.slice).radius
    sel = np.flatnonzero((L["t"] >= t0 + a.orbit[0]) & (L["t"] < t0 + a.orbit[1]))[::a.stride]
    centres, grounds, fits, g_t = [], [], [], []
    for k in sel:
        s = Scan(lt, L, L["t"][k], c_guess, radius, a.slice)
        if s.fit < 0.3 or s.n_slice < 200:
            continue
        near = s.cut(1.2 + 1.0)
        if near.sum() < 200:
            continue
        g = float(np.median(s.xyz[near, 2] - s.h[near]))
        centres.append(s.world_xy(s.c))
        grounds.append(float((s.T[:3, :3] @ np.r_[s.c, g] + s.T[:3, 3])[2]))
        g_t.append(L["t"][k]); fits.append(s.fit)
    centres = np.array(centres); grounds = np.array(grounds); g_t = np.array(g_t)
    c_ref, ground_z = centres.mean(0), float(np.median(grounds))
    # The ground's world z drifts over the orbit (that is the odometry, not the ground), so a single
    # number would smear heights by its scatter. Give each frame the ground of its own moment.
    ground_frame = np.interp(t_unix, g_t, grounds)
    print(f"object frame from {len(fits)} scans: centre {c_ref.round(2)} (scatter {np.hypot(*centres.std(0)):.2f} m), "
          f"pedestal radius {radius:.2f} m")
    print(f"ground world z {grounds.min():.3f}..{grounds.max():.3f} m over the orbit (median {ground_z:.3f}, "
          f"scatter {grounds.std():.3f} m) -> interpolated per frame, {ground_frame.min():.3f}..{ground_frame.max():.3f}")

    cam = json.load(open(CAM))
    K = np.array([[cam["fl_x"], 0, cam["cx"]], [0, cam["fl_y"], cam["cy"]], [0, 0, 1]])
    D = np.array([cam["k1"], cam["k2"], cam["k3"], cam["k4"]])

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    np.savez(a.out, w2c=w2c, K=K, D=D, wh=np.array([cam["w"], cam["h"]]),
             frames=np.array(paths), t=t_unix, tsec=tsec,
             c_ref=c_ref, ground_z=ground_z, ground_frame=ground_frame,
             ground_t=g_t, ground_scan=grounds, pedestal_radius=radius, model=cam["camera_model"])
    print("saved", a.out)


if __name__ == "__main__":
    main()
