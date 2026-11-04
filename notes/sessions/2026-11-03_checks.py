"""The numbers behind the 2026-11-03 session log (notes/sessions/2026-11-03.md). Writes nothing into calib/.

    python notes/sessions/2026-11-03_checks.py streams                  # per-stream counts, gaps, invalid frames
    python notes/sessions/2026-11-03_checks.py board                    # the 8 board shots: 10/21 transform vs a fresh radar-only solve
    python notes/sessions/2026-11-03_checks.py light                    # ground brightness and edge strength, against 10/29 daylight
    python notes/sessions/2026-11-03_checks.py orbit --poses-dir <dir>  # Scotty orbit: range to the pedestal, degrees swept, closure

`orbit` needs the odometry first:
    python -m src.recon.poses data/raw/2026-11-03.17-01-12 --out x.png
"""
import argparse, glob, json, os, sys

import numpy as np

sys.path.insert(0, ".")
SESSION = "2026-11-03"
BOARD = ["16-03-39", "16-07-14", "16-10-01", "16-12-15", "16-14-17", "16-16-48", "16-23-01", "16-27-20"]


def streams():
    print("trace | camera s | radar n, invalid, start after camera | LiDAR n, % missing, longest gap | camera n, longest gap | IMU n")
    for t in sorted(glob.glob(f"data/raw/{SESSION}.*")):
        ts = {s: np.fromfile(f"{t}/{s}/ts", "<f8") for s in ("radar", "lidar", "camera", "imu")}
        v = np.fromfile(f"{t}/radar/valid", "u1"); dl = np.diff(ts["lidar"]); miss = int((np.round(dl / 0.1).astype(int) - 1).sum())
        print(f"{os.path.basename(t)[-8:]} | {ts['camera'][-1] - ts['camera'][0]:6.1f} | {len(v)}, {int((v == 0).sum())}, +{ts['radar'][0] - ts['camera'][0]:.2f} s | "
              f"{len(ts['lidar'])}, {100 * miss / (len(ts['lidar']) + miss):.1f} %, {dl.max():.2f} s | {len(ts['camera'])}, {np.diff(ts['camera']).max() * 1e3:.0f} ms | {len(ts['imu'])}")


def board():
    from scipy.optimize import least_squares
    from src.calib.lidar import LidarTrace
    from src.calib.radar_lidar import local_cluster
    from src.calib.transform import RadarToLidar
    from src.radar import RadarTrace, detect
    T = RadarToLidar.load_joint(); P, R, A = [], [], []
    for n in BOARD:
        d = f"data/raw/{SESSION}.{n}"; tr = RadarTrace(d); k0 = max(0, len(tr) // 2 - 30); rows = []
        for k in range(k0, min(len(tr), k0 + 60)):
            p, _ = detect(tr.frame(k), tr.params, r_min=0.35, r_max=9.0)
            if len(p):
                q = p[np.argmax(p["snr"])]; rows.append((q["r"], q["az"], q["snr"]))
        r, az, snr = np.median(np.array(rows), axis=0)
        lt = LidarTrace(d); xyz = lt.xyz(lt.index_at(tr.ts[len(tr) // 2]))
        pred = T(np.array([[-r * np.sin(az), r * np.cos(az), 0.0]]))[0]; b = local_cluster(xyz, pred[:2], radius=1.0)
        print(f"{n}: radar {r:.2f} m, {np.degrees(az):+.1f} deg, {snr:.0f} dB; LiDAR board {np.round(b, 2)} (xy range {np.hypot(*b[:2]):.2f} m); floor z {np.percentile(xyz[:, 2], 2):.2f}")
        P.append(b); R.append(r); A.append(az)
    P, R, A = np.array(P), np.array(R), np.array(A)

    # Radar part of src.calib.joint: range sigma 2 cm, azimuth sigma 2 deg; same conventions (x is lidar -> radar).
    def unpack(x):
        c, s = np.cos(x[0]), np.sin(x[0]); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.asarray(x[1:4])

    def res(x):
        Rl, t = unpack(x); p = P @ Rl.T + t
        return np.r_[(R - np.linalg.norm(p, axis=1)) / 0.02, np.angle(np.exp(1j * (A - np.arctan2(-p[:, 0], p[:, 1])))) / np.radians(2)]

    def show(tag, x):
        Rl, t = unpack(x); p = P @ Rl.T + t; trl = -Rl.T @ t; yaw = np.degrees(np.arctan2(Rl.T[1, 0], Rl.T[0, 0]))
        rr = (R - np.linalg.norm(p, axis=1)) * 100; aa = np.degrees(np.angle(np.exp(1j * (A - np.arctan2(-p[:, 0], p[:, 1])))))
        print(f"{tag}: radar->lidar yaw {yaw:+.1f} deg, t {np.round(trl, 2)} m; range residual mean {rr.mean():+.1f}, |mean| {np.abs(rr).mean():.1f} cm; |azimuth| {np.abs(aa).mean():.1f} deg")

    j = json.load(open("calib/extrinsics_joint.json"))["radar_to_lidar"]; y0 = np.radians(j["yaw_deg"]); c, s = np.cos(y0), np.sin(y0)
    t0 = -np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T @ np.array(j["t_m"]); x0 = np.r_[-y0, t0]
    show("10/21 joint on these shots", x0)
    s1 = least_squares(res, x0, method="lm"); show("fresh solve, z free (z is not observable here)", s1.x)
    s2 = least_squares(lambda v: res(np.r_[v, t0[2]]), x0[:3], method="lm"); show("fresh solve, z held at 10/21", np.r_[s2.x, t0[2]])
    sig = np.sqrt(np.diag(np.linalg.pinv(s2.jac.T @ s2.jac) * max(1.0, (s2.fun @ s2.fun) / (len(s2.fun) - 3))))
    print(f"   1 sigma of that last solve: yaw {np.degrees(sig[0]):.1f} deg, xy {np.round(sig[1:], 3)} m")


def light():
    import cv2
    for n, a, b, tag in [("2026-10-29.15-49-40", 660, 1020, "10/29 Jeep lot, day"), ("2026-10-29.16-22-36", 170, 225, "10/29 Scotty, day"), ("2026-10-29.16-27-55", 20, 240, "10/29 paths, day"),
                         ("2026-11-03.17-01-12", 30, 165, "11/03 17:01 Scotty"), ("2026-11-03.17-07-11", 20, 200, "11/03 17:07 the Cut"), ("2026-11-03.17-11-37", 10, 180, "11/03 17:11 paths"),
                         ("2026-11-03.17-11-37", 200, 400, "11/03 17:15 indoors"), ("2026-11-03.17-18-46", 0, 80, "11/03 17:19 atrium")]:
        ts = np.fromfile(f"data/raw/{n}/camera/ts", "<f8"); c = cv2.VideoCapture(f"data/raw/{n}/camera/video.avi"); L, G = [], []
        for s in np.linspace(a, b, 12):
            c.set(cv2.CAP_PROP_POS_FRAMES, int(np.searchsorted(ts, ts[0] + s))); ok, f = c.read()
            if not ok: continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)[int(1080 * 0.55):, :].astype(np.float32)   # lower part only: no sky
            L.append(np.median(g)); G.append(np.mean(np.hypot(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1))))
        print(f"{tag:22s} ground luma {np.median(L):5.1f}   edge strength {np.median(G):5.1f}")


def orbit(poses_dir, t_start=30.0, t_turn=105.0, t_end=165.0, guess=(-12.0, -9.5)):
    from src.calib.lidar import LidarTrace
    n = f"{SESSION}.17-01-12"; d = np.load(f"{poses_dir}/{n}/lidar.npz"); T, tl = d["T"], d["t"]; t = tl - tl[0]
    lt = LidarTrace(f"data/raw/{n}"); idx = np.searchsorted(lt.ts, tl)

    def slice_points(a, b):
        out = []
        for i in range(0, len(tl), 5):
            if not a <= t[i] <= b: continue
            x = lt.xyz(idx[i]); g = np.percentile(x[:, 2], 3); m = (x[:, 2] > g + 0.5) & (x[:, 2] < g + 0.95) & (np.hypot(x[:, 0], x[:, 1]) < 6)
            out.append(x[m] @ T[i, :3, :3].T + T[i, :3, 3])
        return np.concatenate(out)[:, :2]

    def pedestal(p, c0, half, step):
        """Centre of the 0.39 m pedestal circle (radius from src.recon.closeup): most points on the ring, few inside."""
        p = p[np.hypot(p[:, 0] - c0[0], p[:, 1] - c0[1]) < half + 1.0]; xs = np.arange(c0[0] - half, c0[0] + half, step); ys = np.arange(c0[1] - half, c0[1] + half, step)
        gx, gy = np.meshgrid(xs, ys, indexing="ij"); best, at = -1, None
        for i in range(0, len(xs)):
            r = np.hypot(p[:, 0, None] - gx[i][None, :], p[:, 1, None] - gy[i][None, :]); score = ((r > 0.31) & (r < 0.47)).sum(0) - 3 * (r < 0.25).sum(0); j = score.argmax()
            if score[j] > best: best, at = score[j], (xs[i], ys[j])
        return np.array(at)

    c = pedestal(slice_points(t_start, t_end)[::3], guess, 3.5, 0.05); print(f"pedestal centre ({c[0]:.2f}, {c[1]:.2f}) in the first scan's frame")
    P = T[:, :3, 3]; R = np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]); th = np.unwrap(np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0]))
    for a, b in [(t_start, t_turn), (t_turn, t_end), (t_start, t_end)]:
        m = (t >= a) & (t <= b)
        print(f"{a:.0f}-{b:.0f} s: range to pedestal median {np.median(R[m]):.2f}, 5-95 % {np.percentile(R[m], 5):.2f}-{np.percentile(R[m], 95):.2f} m; swept {np.degrees(th[m][-1] - th[m][0]):+.0f} deg; "
              f"speed {np.linalg.norm(np.diff(P[m, :2], axis=0), axis=1).sum() / (b - a):.2f} m/s; start-to-end {np.hypot(*(P[m, :2][-1] - P[m, :2][0])):.2f} m")
    for a, b in [(t_start, t_turn), (t_turn, t_end)]:
        q = pedestal(slice_points(a, b), c, 0.6, 0.02); print(f"pedestal from {a:.0f}-{b:.0f} s alone: ({q[0]:.2f}, {q[1]:.2f})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("what", choices=["streams", "board", "light", "orbit"]); ap.add_argument("--poses-dir", default="data/poses"); a = ap.parse_args()
    {"streams": streams, "board": board, "light": light, "orbit": lambda: orbit(a.poses_dir)}[a.what]()
