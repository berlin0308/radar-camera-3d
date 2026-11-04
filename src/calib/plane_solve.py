"""Extrinsics of the current rig from the board's plane, with the board modelled as a mirror.

At 4 mm wavelength a flat board is a mirror: the strongest return comes from the specular point, the foot
of the perpendicular from the radar onto the board's plane, not from the board's centre. The LiDAR measures
that plane directly (every scan of a static shot, a few mm rms), so the radar can be predicted properly:

    radar range      (r_meas - |p_spec|)                  / 0.02 m
    radar azimuth    (az_meas - atan2(-x, y) of p_spec)   / 2 deg     (+ = left)

Unknowns: radar yaw, tx, ty. Height is held at the 10/21 value: range changes by about 1 cm for 15 cm of
height at these distances, so these shots do not constrain it; a tape measure will. Radar pitch and roll
are 0, as in src.calib.joint.

LiDAR -> camera (6 unknowns) comes from the board's four corners: in the LiDAR a fixed-size square fitted
on the plane, in the image the edges snapped to the strongest gradient in undistorted coordinates (a fisheye
bends long edges) from corners read by hand off the median frame. Robust loss, 8 px scale.

    python -m src.calib.plane_solve               # writes calib/extrinsics_joint.json, figures/calib_specular.png
    python -m src.calib.plane_solve --snap        # redo calib/board_corners_2026-11-03.json from HAND_CORNERS
    python -m src.calib.plane_solve --check       # leave-one-out, 10/21 angle shots, image edges on other traces
    python -m src.calib.plane_solve --overlay     # figures/calib_lidar_on_board.png, the 10/21 solve against this one
    python -m src.calib.plane_solve --shots       # figures/calib_shots_1103.png, a top view of each board shot

Writes nothing but the json and the figure. The 10/21 solve stays in calib/extrinsics_joint_2026-10-21.json.
"""
import argparse, json, os

import cv2
import numpy as np
from scipy.optimize import least_squares

from ..radar import RadarTrace, detect
from .board_image import project_fisheye
from .lidar import LidarTrace
from .radar_camera import load_cam, CAM
from .transform import RadarToLidar, joint_path

SHOTS = {"calib2_01": "2026-11-03.16-03-39", "calib2_02": "2026-11-03.16-07-14", "calib2_03": "2026-11-03.16-10-01", "calib2_04": "2026-11-03.16-12-15",
         "calib2_05": "2026-11-03.16-14-17", "calib2_06": "2026-11-03.16-16-48", "calib2_07": "2026-11-03.16-23-01", "calib2_08": "2026-11-03.16-27-20"}
SHOTS_1021 = {"calib_01": "2026-10-21.15-43-34", "calib_02": "2026-10-21.15-44-38", "calib_03b": "2026-10-21.15-47-59", "calib_04": "2026-10-21.15-49-11",
              "calib_05b": "2026-10-21.15-53-58", "calib_07": "2026-10-21.15-56-29", "calib_08": "2026-10-21.15-57-27"}
ANGLE_1021 = {"angle_0": "2026-10-21.15-28-00", "angle_L20": "2026-10-21.15-30-22", "angle_R20": "2026-10-21.15-40-49", "angle_R40": "2026-10-21.15-36-20"}
LAST_30S = {"angle_R40"}      # only the last 30 s of that trace hold the shot (notes/shots.csv)

SIDE = 0.925                  # board side (m): the shots standing on the floor put its bottom edge at the floor, within 2 cm
SIG_R, SIG_AZ = 0.02, np.radians(2.0)
CORNERS = "calib/board_corners_2026-11-03.json"
OUT = "calib/extrinsics_joint.json"

# Corners read off the median frame of each shot: top left, top right, bottom right, bottom left. None = hidden.
HAND_CORNERS = {"calib2_01": [(808, 462), (1098, 462), (1097, 747), (812, 747)], "calib2_02": [None, (601, 460), (602, 755), None],
                "calib2_03": [(1312, 489), (1618, 451), (1607, 770), (1322, 752)], "calib2_04": [(751, 254), (1138, 256), (1141, 640), (739, 640)],
                "calib2_05": [(1113, 432), (1523, 416), (1500, 850), (1112, 820)], "calib2_06": [(442, 240), (820, 262), (802, 642), (417, 640)],
                "calib2_07": [(868, 405), (1368, 385), (1358, 930), (838, 898)], "calib2_08": [(452, 160), (942, 172), (938, 668), (418, 662)]}
REJECT = {("calib2_03", 0)}   # the top-left corner of this shot sits against a white object and the edge snap goes astray


def radar_board(trace_dir, r_max=9.0, last_seconds=None):
    """Median range, azimuth and SNR of the strongest detection, over the shot."""
    tr = RadarTrace(trace_dir); n = len(tr)
    k0 = 0 if last_seconds is None else max(0, n - int(last_seconds * 20)); rows = []
    for k in range(k0, n, 2):
        p, _ = detect(tr.frame(k), tr.params, r_min=0.35, r_max=r_max)
        if len(p):
            q = p[np.argmax(p["snr"])]; rows.append((q["r"], q["az"], q["snr"]))
    r, az, snr = np.median(np.array(rows), axis=0)
    return dict(r=float(r), az=float(az), snr=float(snr), t0=float(tr.ts[k0]), t1=float(tr.ts[-1]))


def board_plane(trace_dir, seed_xy, t0, t1, gate=0.9, height=1.9, seed=0):
    """Plane of the board from every scan in [t0, t1]: RANSAC near seed_xy above the floor, then a least-squares refit."""
    lt = LidarTrace(trace_dir); ks = np.nonzero((lt.ts >= t0 - 0.5) & (lt.ts <= t1 + 0.5))[0][:-1]; ks = ks[:: max(1, len(ks) // 40)]
    P = np.concatenate([lt.xyz(k) for k in ks]); floor = float(np.percentile(lt.xyz(ks[0])[:, 2], 2))
    Q = P[(np.hypot(P[:, 0] - seed_xy[0], P[:, 1] - seed_xy[1]) < gate) & (P[:, 2] > floor + 0.12) & (P[:, 2] < floor + height)]
    rng = np.random.default_rng(seed); best, normal, anchor = 0, None, None
    for _ in range(3000):
        a, b, c = Q[rng.choice(len(Q), 3, replace=False)]; n = np.cross(b - a, c - a); L = np.linalg.norm(n)
        if L < 1e-6: continue
        n = n / L
        if abs(n[2]) > 0.35: continue                                 # an upright board, not the floor or a table
        k = int((np.abs((Q - a) @ n) < 0.015).sum())
        if k > best: best, normal, anchor = k, n, a
    X = Q[np.abs((Q - anchor) @ normal) < 0.02]; centroid = X.mean(0)
    n = np.linalg.svd(X - centroid, full_matrices=False)[2][2]; n = n if n @ centroid < 0 else -n     # normal faces the sensor
    return dict(centroid=centroid, normal=n, floor=floor, rms=float(np.sqrt(np.mean(((X - centroid) @ n) ** 2))), P=P, n_points=len(X))


def board_square(pl, side=SIDE, cell=0.02, band=0.08):
    """Centre and four corners of a square of known side on the plane: most occupied cells inside, fewest in a band around it."""
    n, c0 = pl["normal"], pl["centroid"]; P = pl["P"]
    m = (np.linalg.norm(P[:, :2] - c0[:2], axis=1) < 1.3) & (P[:, 2] > pl["floor"] + 0.06) & (P[:, 2] < pl["floor"] + 2.0) & (np.abs((P - c0) @ n) < 0.025)
    X = P[m]; e_h = np.cross([0, 0, 1], n); e_h /= np.linalg.norm(e_h); e_v = np.cross(n, e_h)
    uv = np.c_[(X - c0) @ e_h, (X - c0) @ e_v]; o = uv.min(0) - 0.25; ij = np.floor((uv - o) / cell).astype(int)
    G = np.zeros(ij.max(0) + int(0.5 / cell), np.float32); G[ij[:, 0], ij[:, 1]] = 1
    k, b = int(round(side / 2 / cell)), int(band / cell); best = (-1e9, 0, 0, 0)
    for ang in np.arange(-10, 10.1, 1.0):
        M = cv2.getRotationMatrix2D((G.shape[1] / 2, G.shape[0] / 2), ang, 1.0); Gr = cv2.warpAffine(G, M, G.shape[::-1], flags=cv2.INTER_LINEAR)
        inner = cv2.boxFilter(Gr, -1, (2 * k + 1, 2 * k + 1), normalize=False, borderType=cv2.BORDER_CONSTANT)
        outer = cv2.boxFilter(Gr, -1, (2 * (k + b) + 1, 2 * (k + b) + 1), normalize=False, borderType=cv2.BORDER_CONSTANT)
        score = inner - 1.5 * (outer - inner); i, j = np.unravel_index(np.argmax(score), score.shape)
        if score[i, j] > best[0]: best = (float(score[i, j]), float(ang), int(i), int(j))
    _, ang, i, j = best
    a = cv2.getRotationMatrix2D((G.shape[1] / 2, G.shape[0] / 2), -ang, 1.0) @ np.array([j, i, 1.0])
    cu, cv_ = np.array([a[1], a[0]]) * cell + o; th = np.radians(-ang); ca, sa = np.cos(th), np.sin(th); d = side / 2
    centre = c0 + cu * e_h + cv_ * e_v
    corners = [centre + (ca * sx * d - sa * sy * d) * e_h + (sa * sx * d + ca * sy * d) * e_v for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
    return centre, np.array(corners), ang


def gather(shots, seed_transform, last=()):
    """One row per shot: the radar's reading and the board's plane in the LiDAR frame."""
    rows = []
    for name, trace in shots.items():
        d = os.path.join("data/raw", trace); rb = radar_board(d, r_max=5.5 if "2026-10-21" in trace else 9.0, last_seconds=30 if name in last else None)
        seed = seed_transform(np.array([[-rb["r"] * np.sin(rb["az"]), rb["r"] * np.cos(rb["az"]), 0.0]]))[0]
        pl = board_plane(d, seed[:2], rb["t0"], rb["t1"]); rows.append(dict(name=name, trace=trace, **rb, **{k: pl[k] for k in ("centroid", "normal", "floor", "rms", "n_points")}, plane=pl))
    return rows


def _unpack(x, tz):
    c, s = np.cos(x[0]), np.sin(x[0])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.r_[x[1], x[2], tz]            # lidar -> radar


def predict(x, tz, rows, model="specular"):
    R_lr, t = _unpack(x, tz); q = np.array([s["centre"] if "centre" in s else s["centroid"] for s in rows])
    if model == "specular":
        nrm = np.array([s["normal"] for s in rows]); o = -R_lr.T @ t; q = o + ((q - o) * nrm).sum(1, keepdims=True) * nrm
    p = q @ R_lr.T + t
    return np.linalg.norm(p, axis=1), np.arctan2(-p[:, 0], p[:, 1]), q


def residuals(x, tz, rows, model):
    r, az, _ = predict(x, tz, rows, model)
    return np.r_[(np.array([s["r"] for s in rows]) - r) * 100, np.degrees(np.angle(np.exp(1j * (np.array([s["az"] for s in rows]) - az))))]   # cm, deg


def solve_radar(rows, x0, tz, model="specular"):
    def f(x):
        e = residuals(x, tz, rows, model); return np.r_[e[: len(rows)] / (SIG_R * 100), e[len(rows):] / np.degrees(SIG_AZ)]
    s = least_squares(f, x0, method="lm"); sig = np.sqrt(np.diag(np.linalg.pinv(s.jac.T @ s.jac) * max(1.0, (s.fun @ s.fun) / (len(s.fun) - len(x0)))))
    R_lr, t = _unpack(s.x, tz); R_rl = R_lr.T; t_rl = -R_rl @ t
    e = residuals(s.x, tz, rows, model); n = len(rows)
    return dict(x=s.x, sig=sig, yaw_deg=float(np.degrees(np.arctan2(R_rl[1, 0], R_rl[0, 0]))), t_m=t_rl, range_cm=e[:n], azimuth_deg=e[n:])


def corner_pairs(rows, img_corners, R0, t0, cam):
    """LiDAR corners paired with image corners: order the LiDAR square by its projection under a rough solve."""
    obj, img, tags = [], [], []
    for s in rows:
        C = s["corners"]; uv = project_fisheye(C @ R0.T + t0, cam); a, b = uv.sum(1), uv[:, 0] - uv[:, 1]
        order = [np.argmin(a), np.argmax(b), np.argmax(a), np.argmin(b)]
        for i, q in enumerate(img_corners[s["name"]]):
            if q is not None: obj.append(C[order[i]]); img.append(q); tags.append((s["name"], i))
    return np.array(obj), np.array(img, float), tags


def solve_camera(obj, img, x0, cam):
    def f(x):
        R, _ = cv2.Rodrigues(x[:3]); return (project_fisheye(obj @ R.T + x[3:], cam) - img).ravel()
    s = least_squares(f, x0, loss="soft_l1", f_scale=8.0); e = f(s.x); sig = np.sqrt(np.diag(np.linalg.pinv(s.jac.T @ s.jac) * (e @ e) / max(len(e) - 6, 1)))
    return dict(x=s.x, sig=sig, R=cv2.Rodrigues(s.x[:3])[0], t=s.x[3:], px=np.hypot(*e.reshape(-1, 2).T))


def median_frame(trace_dir, n=15):
    c = cv2.VideoCapture(os.path.join(trace_dir, "camera", "video.avi")); N = int(c.get(cv2.CAP_PROP_FRAME_COUNT)); fs = []
    for i in np.linspace(30, N - 5, n).astype(int):
        c.set(cv2.CAP_PROP_POS_FRAMES, int(i)); ok, f = c.read()
        if ok: fs.append(f)
    return np.median(np.stack(fs), axis=0).astype(np.uint8)


def snap_corners(out=CORNERS):
    """Edges from the hand-read corners, snapped to the strongest gradient, straight in undistorted coordinates; corners = intersections."""
    cam, K, D = load_cam(); res = {}
    und = lambda p: cv2.fisheye.undistortPoints(np.asarray(p, np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)

    def edge_points(gx, gy, a, b, half=16, trim=0.12, n=60):
        a, b = np.array(a, float), np.array(b, float); u = (b - a) / np.linalg.norm(b - a); nrm = np.array([-u[1], u[0]]); pts = []
        for s in np.linspace(trim, 1 - trim, n):
            q = (a + s * (b - a))[None] + np.arange(-half, half + 1)[:, None] * nrm; mx, my = q[:, 0].astype(np.float32)[:, None], q[:, 1].astype(np.float32)[:, None]
            v = np.abs(cv2.remap(gx, mx, my, cv2.INTER_LINEAR).ravel() * nrm[0] + cv2.remap(gy, mx, my, cv2.INTER_LINEAR).ravel() * nrm[1])
            if v.max() > 6: pts.append(q[np.argmax(v)])
        return np.array(pts)

    def fit_line(p):
        for _ in range(3):
            c = p.mean(0); u = np.linalg.svd(p - c)[2][0]; r = np.abs((p - c) @ np.array([-u[1], u[0]])); p = p[r < max(np.percentile(r, 75), 2.0 / cam["fl_x"])]
        return p.mean(0), np.linalg.svd(p - p.mean(0))[2][0]

    for name, hand in HAND_CORNERS.items():
        g = cv2.GaussianBlur(cv2.cvtColor(median_frame(os.path.join("data/raw", SHOTS[name])), cv2.COLOR_BGR2GRAY), (0, 0), 1.5).astype(np.float32)
        gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=5), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=5); lines = {}
        for k, (i, j) in enumerate([(0, 1), (1, 2), (2, 3), (3, 0)]):
            if hand[i] is None or hand[j] is None: continue
            p = edge_points(gx, gy, hand[i], hand[j])
            if len(p) >= 8: lines[k] = fit_line(und(p))
        out_c = [None] * 4
        for i, (k1, k2) in enumerate([(3, 0), (0, 1), (1, 2), (2, 3)]):
            if (name, i) in REJECT: continue
            if k1 in lines and k2 in lines:
                (c1, u1), (c2, u2) = lines[k1], lines[k2]; t = np.linalg.solve(np.c_[u1, -u2], c2 - c1); q = c1 + t[0] * u1
                out_c[i] = project_fisheye(np.array([[q[0], q[1], 1.0]]), cam)[0].tolist()
            elif hand[i] is not None:
                out_c[i] = [float(hand[i][0]), float(hand[i][1])]
        res[name] = out_c; print(name, np.round([c if c else [np.nan, np.nan] for c in out_c]).tolist())
    json.dump(dict(note="top left, top right, bottom right, bottom left, in pixels; null = hidden or rejected. Hand-read from the median frame of each shot, edges snapped by src.calib.plane_solve --snap", corners=res), open(out, "w"), indent=1)


def load_rows():
    old = RadarToLidar.load_joint(trace=SHOTS_1021["calib_01"]); rows = gather(SHOTS, old)
    for s in rows:
        s["centre"], s["corners"], s["rotation"] = board_square(s["plane"])
        print(f"{s['name']} {s['trace'][-8:]}: radar {s['r']:.2f} m {np.degrees(s['az']):+.1f} deg {s['snr']:.0f} dB; board centre {np.round(s['centre'], 3)}, "
              f"plane rms {s['rms'] * 1000:.1f} mm, normal vs line of sight {np.degrees(np.arccos(abs(s['normal'] @ s['centre'] / np.linalg.norm(s['centre'])))):.1f} deg, bottom edge {s['corners'][:, 2].min():.2f} (floor {s['floor']:.2f})")
    return rows


def seed_x(trace=SHOTS_1021["calib_01"]):
    """Start and held height from the 10/21 solve."""
    j = json.load(open(joint_path(trace)))["radar_to_lidar"]; y = np.radians(j["yaw_deg"]); c, s = np.cos(y), np.sin(y)
    t = -np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T @ np.array(j["t_m"]); return np.r_[-y, t[:2]], float(t[2])


def rows_1021(tz_only=False):
    old = RadarToLidar.load_joint(trace=SHOTS_1021["calib_01"])
    return gather(SHOTS_1021, old), gather(ANGLE_1021, old, last=LAST_30S)


def run(out=OUT, fig="figures/calib_specular.png"):
    cam, _, _ = load_cam(); rows = load_rows(); x0, tz = seed_x()
    centre = solve_radar(rows, x0, tz, "centre"); spec = solve_radar(rows, x0, tz, "specular")
    for tag, s in (("board centre", centre), ("specular point", spec)):
        print(f"radar -> lidar, {tag}: yaw {s['yaw_deg']:+.2f} +- {np.degrees(s['sig'][0]):.2f} deg, t {np.round(s['t_m'], 3)}; |range| {np.abs(s['range_cm']).mean():.1f} cm, |azimuth| {np.abs(s['azimuth_deg']).mean():.2f} deg")
    lc_old = json.load(open(joint_path(SHOTS_1021["calib_01"])))["lidar_to_camera"]; R0, t0 = np.array(lc_old["R"]), np.array(lc_old["t_m"])
    obj, img, tags = corner_pairs(rows, json.load(open(CORNERS))["corners"], R0, t0, cam)
    c = solve_camera(obj, img, np.r_[cv2.Rodrigues(R0)[0].ravel(), t0], cam); before = np.hypot(*(project_fisheye(obj @ R0.T + t0, cam) - img).T)
    print(f"lidar -> camera: {before.mean():.1f} px under the 10/21 solve -> {c['px'].mean():.1f} px mean, {np.median(c['px']):.1f} median, {c['px'].max():.1f} max, {len(obj)} corners; t {c['t'].round(3)} m +- {c['sig'][3:].round(3)}, rotation +- {np.degrees(c['sig'][:3]).round(2)} deg")
    R_rl, _ = _unpack(spec["x"], tz); R_rl = R_rl.T; t_rl = spec["t_m"]; R_rc, t_rc = c["R"] @ R_rl, c["R"] @ t_rl + c["t"]
    json.dump(dict(world="lidar (sensor frame)", rig="OS-0-128, from 2026-10-29 on", solved="2026-11-03 board shots, src.calib.plane_solve", model="board plane; radar on the specular point", shots=[s["name"] for s in rows],
                   sigmas=dict(range_m=SIG_R, azimuth_deg=2.0, pixel_px="soft_l1, 8 px"),
                   radar_to_lidar=dict(yaw_deg=spec["yaw_deg"], t_m=t_rl.tolist(), sigma=dict(yaw_deg=float(np.degrees(spec["sig"][0])), t_xy_m=spec["sig"][1:].tolist()), note="pitch, roll fixed at 0; height held at the 10/21 value (not observable here); to be replaced by a tape measure"),
                   lidar_to_camera=dict(R=c["R"].tolist(), t_m=c["t"].tolist(), sigma=dict(rotvec_deg=np.degrees(c["sig"][:3]).tolist(), t_m=c["sig"][3:].tolist())),
                   radar_to_camera=dict(R=R_rc.tolist(), t_m=t_rc.tolist(), intrinsics=CAM, note="composed; consistent with the two above by construction"),
                   residuals=dict(range_cm=spec["range_cm"].tolist(), azimuth_deg=spec["azimuth_deg"].tolist(), corner_px=c["px"].tolist(), range_cm_board_centre=centre["range_cm"].tolist(), azimuth_deg_board_centre=centre["azimuth_deg"].tolist())),
              open(out, "w"), indent=1)
    print("saved", out)
    figure(rows, x0, tz, spec, centre, fig)


def figure(rows, x0, tz, spec, centre, path):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    BLUE, ORANGE, INK, MUTED = "#2C43B0", "#E07B39", "#15202B", "#566373"
    cal, angle = rows_1021(); x10, _ = solve_radar(cal, x0, tz)["x"], None
    ang_c = residuals(solve_radar(cal, x0, tz, "centre")["x"], tz, angle, "centre")[len(angle):]; ang_s = residuals(x10, tz, angle, "specular")[len(angle):]
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 5.4), constrained_layout=True, gridspec_kw=dict(width_ratios=[1.25, 1, 0.7]))
    R_lr, t = _unpack(spec["x"], tz); a = ax[0]
    a.scatter(0, 0, marker="^", s=110, color=INK, zorder=5, label="radar")
    for k, s in enumerate(rows):
        C = s["corners"][[0, 1]] @ R_lr.T + t; p = (s["centre"] @ R_lr.T + t); _, _, q = predict(spec["x"], tz, [s]); q = q[0] @ R_lr.T + t
        a.plot(C[:, 0], C[:, 1], color=MUTED, lw=2, solid_capstyle="round", label="board, from the LiDAR" if k == 0 else None)
        a.plot([0, q[0]], [0, q[1]], color=BLUE, lw=0.8, alpha=.6)
        a.scatter(*p[:2], s=44, facecolor="white", edgecolor=ORANGE, lw=1.6, zorder=4, label="board centre" if k == 0 else None)
        a.scatter(*q[:2], s=40, color=BLUE, zorder=4, label="specular point" if k == 0 else None)
        a.scatter(-s["r"] * np.sin(s["az"]), s["r"] * np.cos(s["az"]), marker="x", s=46, color=INK, lw=1.4, zorder=6, label="radar reading" if k == 0 else None)
    a.set_aspect("equal"); a.set_xlim(-2.6, 2.6); a.set_ylim(-0.2, 3.3); a.set_xlabel("x, right (m)"); a.set_ylabel("y, forward (m)"); a.legend(fontsize=10, loc="lower center", ncol=2, frameon=False)
    a.set_title("a · a flat board returns from its specular point (radar frame, 11/3)", fontsize=12, loc="left")
    w = .38; n = len(rows); names = [s["name"][-2:] for s in rows] + [s["name"][6:] for s in angle]; xs = np.r_[np.arange(n), n + 1 + np.arange(len(angle))]
    az_c, az_s = np.r_[centre["azimuth_deg"], ang_c], np.r_[spec["azimuth_deg"], ang_s]
    b = ax[1]; b.bar(xs - w / 2, az_c, w * .92, color=ORANGE, label="aimed at the board centre"); b.bar(xs + w / 2, az_s, w * .92, color=BLUE, label="aimed at the specular point"); b.axhline(0, color=MUTED, lw=.8)
    b.axvline(n, color=MUTED, lw=.8, ls=":"); b.set_xticks(xs); b.set_xticklabels(names, fontsize=9); b.set_ylabel("azimuth residual (deg)"); b.legend(fontsize=10, frameon=False, loc="upper left"); b.set_ylim(-8, 11)
    b.text(n / 2 - .5, -7.4, "11/3 shots, in the solve", ha="center", fontsize=10, color=MUTED); b.text(n + 2.5, -7.4, "10/21, held out", ha="center", fontsize=10, color=MUTED)
    b.set_title("b · azimuth residual per shot", fontsize=12, loc="left")
    c = ax[2]; means = [np.abs(centre["azimuth_deg"]).mean(), np.abs(spec["azimuth_deg"]).mean(), np.abs(ang_c).mean(), np.abs(ang_s).mean()]; pos = [0, .9, 2.2, 3.1]
    bars = c.bar(pos, means, .8, color=[ORANGE, BLUE, ORANGE, BLUE]); c.set_xticks([.45, 2.65]); c.set_xticklabels(["11/3 shots", "10/21 held out"]); c.set_ylabel("mean |azimuth residual| (deg)")
    for x, v in zip(pos, means): c.text(x, v + .05, f"{v:.1f}°", ha="center", fontsize=11, color=INK)
    c.set_title("c · mean error", fontsize=12, loc="left")
    for a_ in ax[1:]: a_.spines[["top", "right"]].set_visible(False); a_.grid(axis="y", color="#E6EBF2", lw=.8); a_.set_axisbelow(True)
    fig.savefig(path, dpi=150); print("saved", path)


def shots_figure(path="figures/calib_shots_1103.png"):
    """Top view of each board shot in the radar frame: the LiDAR scan, the fitted board, every CFAR point of the shot, the radar's reading."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    x0, tz = seed_x(); rows = load_rows(); sol = solve_radar(rows, x0, tz); R_lr, t = _unpack(sol["x"], tz)
    fig, axes = plt.subplots(2, 4, figsize=(16.64, 8.6), constrained_layout=True)
    for a, s in zip(axes.flat, rows):
        lt = LidarTrace(os.path.join("data/raw", s["trace"])); xyz = lt.xyz(lt.index_at(s["t0"] + 3)); xyz = xyz[xyz[:, 2] > s["floor"] + 0.3]; p = xyz @ R_lr.T + t
        tr = RadarTrace(os.path.join("data/raw", s["trace"])); pts = np.concatenate([detect(tr.frame(k), tr.params, r_min=0.35, r_max=9.0)[0] for k in range(0, len(tr), 4)])
        a.scatter(p[:, 0], p[:, 1], s=.4, color="#B9C2CC", label="LiDAR scan, above the floor")
        a.scatter(-pts["r"] * np.sin(pts["az"]), pts["r"] * np.cos(pts["az"]), s=9, c=pts["snr"], cmap="plasma", vmin=10, vmax=45, label="CFAR points of the shot, colour = SNR")
        C = s["corners"][[0, 1]] @ R_lr.T + t; a.plot(C[:, 0], C[:, 1], color="#566373", lw=2.4, label="board, plane fit")
        a.scatter(-s["r"] * np.sin(s["az"]), s["r"] * np.cos(s["az"]), marker="x", s=70, color="#15202B", lw=1.8, zorder=5, label="radar reading")
        a.scatter(0, 0, marker="^", s=70, color="#15202B"); a.set_aspect("equal"); a.set_xlim(-5, 5); a.set_ylim(-0.5, 7.5); a.spines[["top", "right"]].set_visible(False); a.tick_params(labelsize=9)
        a.set_title(f"{s['name']} · {s['trace'][-8:]} · {s['r']:.2f} m, {np.degrees(s['az']):+.0f}°, {s['snr']:.0f} dB", fontsize=11, loc="left")
    axes.flat[0].legend(fontsize=8, loc="upper left", frameon=False, markerscale=2); fig.supxlabel("x, right (m)", fontsize=11); fig.supylabel("y, forward (m)", fontsize=11)
    fig.savefig(path, dpi=120); print("saved", path)


def overlay_figure(path="figures/calib_lidar_on_board.png", picks=("calib2_01", "calib2_03", "calib2_05", "calib2_07")):
    """The LiDAR points on the board, projected on the image through the 10/21 solve and this one. Crops hold the board only, no people."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    cam, _, _ = load_cam(); old = json.load(open(joint_path(SHOTS_1021["calib_01"])))["lidar_to_camera"]; new = json.load(open(OUT))["lidar_to_camera"]; corners = json.load(open(CORNERS))["corners"]
    T = RadarToLidar.load_joint(trace=SHOTS_1021["calib_01"]); fig, ax = plt.subplots(2, len(picks), figsize=(16.64, 7.2), constrained_layout=True)
    for j, name in enumerate(picks):
        d = os.path.join("data/raw", SHOTS[name]); rb = radar_board(d); seed = T(np.array([[-rb["r"] * np.sin(rb["az"]), rb["r"] * np.cos(rb["az"]), 0.0]]))[0]; pl = board_plane(d, seed[:2], rb["t0"], rb["t1"])
        P = pl["P"]; m = (np.abs((P - pl["centroid"]) @ pl["normal"]) < 0.02) & (np.linalg.norm(P[:, :2] - pl["centroid"][:2], axis=1) < 0.75) & (P[:, 2] > pl["floor"] + 0.1); Q = P[m][::9]
        img = cv2.cvtColor(median_frame(d), cv2.COLOR_BGR2RGB); c = np.array([q for q in corners[name] if q is not None]); x0, x1, y0, y1 = int(c[:, 0].min() - 90), int(c[:, 0].max() + 90), int(c[:, 1].min() - 70), int(c[:, 1].max() + 70)
        x0, y0 = max(x0, 0), max(y0, 0); x1, y1 = min(x1, 1919), min(y1, 1079)
        for i, (lc, col, tag) in enumerate([(old, "#E07B39", "10/21 solve"), (new, "#2C43B0", "11/3 solve")]):
            uv = project_fisheye(Q @ np.array(lc["R"]).T + np.array(lc["t_m"]), cam); a = ax[i, j]; a.imshow(img); a.scatter(uv[:, 0], uv[:, 1], s=1.2, color=col, alpha=.8, linewidths=0)
            a.scatter(c[:, 0], c[:, 1], marker="+", s=130, color="white", linewidths=1.6); a.set_xlim(x0, x1); a.set_ylim(y1, y0); a.axis("off"); a.set_title(f"{name}, {tag}", fontsize=12, loc="left")
    fig.suptitle("LiDAR points on the board (dots), projected on the image; white crosses are the board corners found in the image", fontsize=13)
    fig.savefig(path, dpi=110); print("saved", path)


def check():
    """Leave-one-out on the 11/3 shots; 10/21 angle shots held out of a 10/21 solve; image edges on traces outside the solve."""
    cam, _, _ = load_cam(); rows = load_rows(); x0, tz = seed_x(); corners = json.load(open(CORNERS))["corners"]
    for model in ("centre", "specular"):
        r_, a_ = [], []
        for i in range(len(rows)):
            s = solve_radar(rows[:i] + rows[i + 1:], x0, tz, model); e = residuals(s["x"], tz, [rows[i]], model); r_.append(e[0]); a_.append(e[1])
        print(f"leave-one-out, {model}: |range| {np.abs(r_).mean():.1f} cm, |azimuth| {np.abs(a_).mean():.2f} deg")
    lc = json.load(open(OUT))["lidar_to_camera"]; R, t = np.array(lc["R"]), np.array(lc["t_m"]); px = []
    for i, s in enumerate(rows):
        o, m, _ = corner_pairs(rows[:i] + rows[i + 1:], corners, R, t, cam); c = solve_camera(o, m, np.r_[cv2.Rodrigues(R)[0].ravel(), t], cam)
        o2, m2, _ = corner_pairs([s], corners, R, t, cam); px.extend(np.hypot(*(project_fisheye(o2 @ c["R"].T + c["t"], cam) - m2).T))
    print(f"leave-one-out, corners: {np.mean(px):.1f} px mean, {np.median(px):.1f} median")
    cal, angle = rows_1021()
    for model in ("centre", "specular"):
        s = solve_radar(cal, x0, tz, model); e = residuals(s["x"], tz, angle, model); n = len(angle)
        print(f"10/21 rig, {model}: solved on calib_*, yaw {s['yaw_deg']:+.2f} +- {np.degrees(s['sig'][0]):.2f}, t {np.round(s['t_m'], 3)}; on calib_* |azimuth| {np.abs(s['azimuth_deg']).mean():.2f}, on angle shots held out |azimuth| {np.abs(e[n:]).mean():.2f} deg, per shot "
              + ", ".join(f"{q['name']} {v:+.1f}" for q, v in zip(angle, e[n:])))
    edges(cam)


def _edge_points(lt, k, th=0.4):
    r = lt.range_image(k); ang = lt.enc - lt.az; X, Y, Z = r * np.cos(lt.alt) * np.cos(ang), r * np.cos(lt.alt) * np.sin(ang), r * np.sin(lt.alt)
    d = np.diff(r, axis=1); a = (np.abs(d) > th) & (r[:, :-1] > 0.5) & (r[:, 1:] > 0.5)
    i1, j1 = np.nonzero(a & (d > 0)); i2, j2 = np.nonzero(a & (d < 0))                   # the nearer side of each jump
    return np.r_[np.c_[X[i1, j1], Y[i1, j1], Z[i1, j1]], np.c_[X[i2, j2 + 1], Y[i2, j2 + 1], Z[i2, j2 + 1]]]


def edges(cam, per_trace=14):
    """Do LiDAR depth edges land on image edges? Mean image gradient at the projected points, frames where the rig turns slowly."""
    old = json.load(open(joint_path(SHOTS_1021["calib_01"])))["lidar_to_camera"]; new = json.load(open(OUT))["lidar_to_camera"]
    Ro, to, Rn, tn = np.array(old["R"]), np.array(old["t_m"]), np.array(new["R"]), np.array(new["t_m"])
    frames = []
    for tr in ["2026-10-29.15-49-40", "2026-10-29.16-12-38", "2026-10-29.16-22-36", "2026-10-29.16-27-55", "2026-11-03.17-01-12", "2026-11-03.17-07-11", "2026-11-03.17-11-37"]:
        d = os.path.join("data/raw", tr); lt = LidarTrace(d); ts = np.fromfile(d + "/camera/ts", "<f8"); im = np.fromfile(d + "/imu/ts", "<f8"); w = np.linalg.norm(np.fromfile(d + "/imu/avel", "<f4").reshape(-1, 3), axis=1)
        c = cv2.VideoCapture(d + "/camera/video.avi"); got = 0
        for k in np.linspace(5, len(lt) - 5, 200).astype(int):
            i0, i1 = np.searchsorted(im, [lt.ts[k] - 0.2, lt.ts[k] + 0.3])
            if i1 <= i0 or np.degrees(w[i0:i1].max()) > 8: continue
            P = _edge_points(lt, k); c.set(cv2.CAP_PROP_POS_FRAMES, int(np.searchsorted(ts, lt.ts[k] + 0.05))); ok, f = c.read()
            if not ok or len(P) < 200: continue
            g = cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (0, 0), 3).astype(np.float32); frames.append((tr, P, np.hypot(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1)))); got += 1
            if got >= per_trace: break

    def score(R, t):
        out = []
        for _, P, G in frames:
            q = P @ R.T + t; q = q[(q[:, 2] > 0.5) & (np.linalg.norm(q, axis=1) < 25)]; uv = project_fisheye(q, cam).astype(int); m = (uv[:, 0] >= 0) & (uv[:, 0] < 1920) & (uv[:, 1] >= 0) & (uv[:, 1] < 1080)
            out.append(G[uv[m, 1], uv[m, 0]].mean() if m.sum() > 30 else np.nan)
        return np.array(out)
    so, sn = score(Ro, to), score(Rn, tn); ok = np.isfinite(so) & np.isfinite(sn)
    print(f"image edges, {ok.sum()} frames of {len(frames)}: gradient at LiDAR edges, 10/21 solve {so[ok].mean():.2f}, 11/3 solve {sn[ok].mean():.2f}; 11/3 better in {(sn[ok] > so[ok]).sum()}")
    for tr in sorted({f[0] for f in frames}):
        m = np.array([f[0] == tr for f in frames]) & ok; print(f"   {tr}: {so[m].mean():.2f} -> {sn[m].mean():.2f} over {m.sum()} frames, better in {(sn[m] > so[m]).sum()}")
    base = np.nanmean(sn)
    for axis, name in enumerate(["about camera x (pitch)", "about camera y (yaw)", "about camera z (roll)"]):
        line = []
        for deg in (-3, -1.5, 1.5, 3):
            v = np.zeros(3); v[axis] = np.radians(deg); line.append(f"{deg:+.1f}: {np.nanmean(score(cv2.Rodrigues(v)[0] @ Rn, tn)) - base:+.2f}")
        print(f"   perturb rotation {name}: " + ", ".join(line))
    for axis, name in enumerate("xyz"):
        line = []
        for cm in (-20, -10, 10, 20):
            v = np.zeros(3); v[axis] = cm / 100; line.append(f"{cm:+d} cm: {np.nanmean(score(Rn, tn + v)) - base:+.2f}")
        print(f"   perturb t{name}: " + ", ".join(line))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--snap", action="store_true"); ap.add_argument("--check", action="store_true"); ap.add_argument("--overlay", action="store_true"); ap.add_argument("--shots", action="store_true"); a = ap.parse_args()
    snap_corners() if a.snap else check() if a.check else overlay_figure() if a.overlay else shots_figure() if a.shots else run()
