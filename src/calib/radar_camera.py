"""Radar -> camera extrinsics by PnP on the calibration board.

3D points: board position from the radar (x, y from range and azimuth, z from the LiDAR board height,
since radar elevation is unusable; a constant z offset is absorbed by the translation).
2D points: board centre in the camera frame (calib/board_pixels.json). Fisheye intrinsics from
red-rover. Solves with cv2.solvePnP after undistorting the pixels, reports the reprojection error,
and writes calib/extrinsics_radar_camera.json plus a figure.

    python -m src.calib.radar_camera
"""
import json, os

import cv2
import numpy as np

from .board_image import project_fisheye, CAM
from .radar_lidar import SHOTS
from .lidar import LidarTrace
from .transform import RadarToLidar


def load_cam(path=CAM):
    c = json.load(open(path))
    K = np.array([[c["fl_x"], 0, c["cx"]], [0, c["fl_y"], c["cy"]], [0, 0, 1]])
    D = np.array([c["k1"], c["k2"], c["k3"], c["k4"]])
    return c, K, D


def run(pix_json="calib/board_pixels.json", out_json="calib/extrinsics_radar_camera.json", fig_path="figures/calib_radar_camera.png",
        skip=("calib_06",), cam_path=CAM):
    pix = json.load(open(pix_json)); cam, K, D = load_cam(cam_path); T_rl = RadarToLidar.load()
    names, obj, img = [], [], []
    for name in SHOTS:
        if name in skip or name not in pix: continue
        p = pix[name]; r, az = p["radar"]["r"], p["radar"]["az"]
        # board height: from the LiDAR scan of that shot, relative to the lidar origin
        d = os.path.join("data/raw", SHOTS[name]); lt = LidarTrace(d); xyz = lt.xyz(lt.index_at(p["t"]))
        pred = T_rl(np.array([[-r * np.sin(az), r * np.cos(az), 0.0]]))[0]
        near = (np.hypot(xyz[:, 0] - pred[0], xyz[:, 1] - pred[1]) < 0.5) & (xyz[:, 2] > -1.0) & (xyz[:, 2] < 0.6)
        z = float(np.median(xyz[near, 2])) if near.sum() > 10 else 0.0
        names.append(name); obj.append([-r * np.sin(az), r * np.cos(az), z]); img.append(p["centre"])
    obj, img = np.array(obj, np.float64), np.array(img, np.float64)

    # radar (x right, y forward, z up) -> camera axes (x right, y down, z forward), then PnP refines
    A = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], np.float64)
    obj_cam_axes = obj @ A.T
    und = cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2), K, D).reshape(-1, 2)      # normalised coordinates
    ok, rvec, tvec = cv2.solvePnP(obj_cam_axes, und, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE,
                                  useExtrinsicGuess=True, rvec=np.zeros(3), tvec=np.zeros(3))
    R, _ = cv2.Rodrigues(rvec); t = tvec.ravel()
    proj = project_fisheye(obj_cam_axes @ R.T + t, cam)
    proj0 = project_fisheye(obj_cam_axes, cam)
    err, err0 = np.hypot(*(proj - img).T), np.hypot(*(proj0 - img).T)
    R_full = R @ A                                                 # radar frame -> camera frame
    rot_deg = np.degrees(cv2.Rodrigues(R)[0].ravel())
    print(f"radar->camera: rotation (Rodrigues, deg, in camera axes) {rot_deg.round(2)}, t {t.round(3)} m")
    print(f"reprojection error: before {err0.mean():.1f} px, after {err.mean():.1f} px (max {err.max():.1f}) over {len(names)} shots")
    for n, e0, e in zip(names, err0, err): print(f"  {n}: {e0:.1f} -> {e:.1f} px")
    json.dump(dict(frame_from="radar (x right, y forward, z up)", frame_to="camera (x right, y down, z forward)",
                   R=R_full.tolist(), t_m=t.tolist(), rotation_deg_rodrigues=rot_deg.tolist(), intrinsics=cam_path,
                   shots=names, reprojection_px=err.tolist(), reprojection_px_before=err0.tolist(),
                   object_points_radar_m=obj.tolist(), image_points_px=img.tolist(),
                   projected_px_before=proj0.tolist(), projected_px_after=proj.tolist(),
                   note="board z taken from the LiDAR board height (lidar frame); a constant z offset is absorbed by t"),
              open(out_json, "w"), indent=1)

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 4, figsize=(16.64, 5.6), constrained_layout=True)
    for ax, n, pi, p0, p1 in zip(axes.flat, names, img, proj0, proj):
        im = plt.imread(os.path.join("data/sessions/calib_frames", f"{n}.png")); ax.imshow(im)
        ax.scatter(*pi, marker="+", s=140, color="#E07B39", lw=2, label="board centre")
        ax.scatter(*p0, marker="x", s=80, color="#2C43B0", lw=2, label="radar, no extrinsics")
        ax.scatter(*p1, marker="o", s=80, facecolors="none", edgecolors="#2E8B57", lw=2, label="radar, calibrated")
        ax.plot([pi[0], p1[0]], [pi[1], p1[1]], color="#2E8B57", lw=1)
        ax.set_xlim(max(0, pi[0] - 400), min(1920, pi[0] + 400)); ax.set_ylim(min(1080, pi[1] + 225), max(0, pi[1] - 225))
        ax.set_title(f"{n}: {err0[names.index(n)]:.0f} -> {err[names.index(n)]:.0f} px", fontsize=11); ax.axis("off")
    axes.flat[0].legend(fontsize=8, loc="lower left"); axes.flat[-1].axis("off")
    fig.suptitle(f"Radar detections projected into the camera, before (blue x) and after (green o) PnP; mean error {err0.mean():.0f} -> {err.mean():.0f} px", fontsize=12)
    fig.savefig(fig_path, dpi=120); print("saved", out_json, fig_path)


def run_lidar_camera(pix_json="calib/board_pixels.json", out_json="calib/extrinsics_lidar_camera.json", skip=("calib_06",), cam_path=CAM):
    """LiDAR -> camera by PnP on the LiDAR board centroids (cm-level 3D points), then compose with
    radar -> LiDAR to get a second radar -> camera estimate; the two are compared as a loop check."""
    pix = json.load(open(pix_json)); cam, K, D = load_cam(cam_path); T_rl = RadarToLidar.load()
    names, obj, img = [], [], []
    for name in SHOTS:
        if name in skip or name not in pix: continue
        p = pix[name]; r, az = p["radar"]["r"], p["radar"]["az"]
        d = os.path.join("data/raw", SHOTS[name]); lt = LidarTrace(d); xyz = lt.xyz(lt.index_at(p["t"]))
        pred = T_rl(np.array([[-r * np.sin(az), r * np.cos(az), 0.0]]))[0]
        near = (np.hypot(xyz[:, 0] - pred[0], xyz[:, 1] - pred[1]) < 0.5) & (xyz[:, 2] > -1.0) & (xyz[:, 2] < 0.6)
        if near.sum() < 10: continue
        names.append(name); obj.append(xyz[near].mean(axis=0)); img.append(p["centre"])
    obj, img = np.array(obj, np.float64), np.array(img, np.float64)
    und = cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    # guess: lidar -x forward -> camera z; lidar z up -> camera -y; lidar -y right? derive from the radar->lidar yaw
    A = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], np.float64)                 # radar axes -> camera axes
    R0 = A @ T_rl.R.T                                                            # lidar -> radar -> camera axes
    rvec0, _ = cv2.Rodrigues(R0)
    ok, rvec, tvec = cv2.solvePnP(obj, und, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess=True,
                                  rvec=rvec0.copy(), tvec=np.zeros(3))
    R, _ = cv2.Rodrigues(rvec); t = tvec.ravel()
    err = np.hypot(*(project_fisheye(obj @ R.T + t, cam) - img).T)
    print(f"lidar->camera: reprojection {err.mean():.1f} px (max {err.max():.1f}) over {len(names)} shots")
    # compose: radar -> lidar -> camera
    Rc, tc = R @ T_rl.R, R @ T_rl.t + t
    direct = json.load(open("calib/extrinsics_radar_camera.json"))
    Rd, td = np.array(direct["R"]), np.array(direct["t_m"])
    dR = np.degrees(np.linalg.norm(cv2.Rodrigues(Rd.T @ Rc)[0]))
    print(f"composed radar->camera vs direct: rotation differs by {dR:.1f}°, translation by {np.linalg.norm(tc - td):.2f} m "
          f"(composed t {tc.round(2)}, direct t {td.round(2)}; the radar->lidar z offset is unknown, so z is not comparable)")
    json.dump(dict(frame_from="lidar (sensor frame)", frame_to="camera (x right, y down, z forward)", R=R.tolist(), t_m=t.tolist(),
                   intrinsics=cam_path, shots=names, reprojection_px=err.tolist(),
                   composed_radar_to_camera=dict(R=Rc.tolist(), t_m=tc.tolist(), note="z of t carries the unknown radar->lidar z offset"),
                   loop_check=dict(rotation_diff_deg=float(dR), translation_diff_m=float(np.linalg.norm(tc - td)))),
              open(out_json, "w"), indent=1)
    print("saved", out_json)


if __name__ == "__main__":
    run()
    run_lidar_camera()
