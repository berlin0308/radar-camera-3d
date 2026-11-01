"""Radar -> LiDAR extrinsics from the 10/21 calibration board placements.

The board is the strongest radar return in each 10 s shot. In the LiDAR scan it is a compact
cluster at the same range (radar range is good to 1 cm after a +0.183 m offset), so for every shot
the candidate clusters in that range shell are found, and the yaw on which most shots agree picks
the right cluster. A 2D rigid transform (yaw, tx, ty) is then solved by least squares; radar
elevation is too noisy for pitch, so z is left to a tape measure.

    python -m src.calib.radar_lidar                 # writes calib/extrinsics_radar_lidar.json and figures
"""
import argparse, json, os

import numpy as np

from ..radar import RadarTrace, detect, AZIMUTHS
from .lidar import LidarTrace

from ..radar.pointcloud import RANGE_OFFSET   # 0.183 m, applied inside detect(); kept here for the json record
SHOTS = {"calib_01": "2026-10-21.15-43-34", "calib_02": "2026-10-21.15-44-38", "calib_03b": "2026-10-21.15-47-59",
         "calib_04": "2026-10-21.15-49-11", "calib_05b": "2026-10-21.15-53-58", "calib_06": "2026-10-21.15-55-19",
         "calib_07": "2026-10-21.15-56-29", "calib_08": "2026-10-21.15-57-27"}


R_MAX = {"calib_06": 1.0}     # the board fills the camera at ~0.5 m; a stronger echo sits behind it


def radar_board(trace_dir, n_frames=40, r_max=5.0):
    """Median (r, az, el, snr) of the strongest detection under r_max over the middle frames."""
    tr = RadarTrace(trace_dir)
    k0 = len(tr) // 2 - n_frames // 2
    rows = []
    for k in range(k0, k0 + n_frames):
        pts, _ = detect(tr.frame(k), tr.params, r_min=0.35, r_max=r_max)
        if len(pts):
            p = pts[np.argmax(pts["snr"])]; rows.append((p["r"], p["az"], p["el"], p["snr"]))
    m = np.median(np.array(rows), axis=0)
    return dict(r=float(m[0]), az=float(m[1]), el=float(m[2]), snr=float(m[3]), t=float(tr.ts[len(tr) // 2]))   # detect() already removes the range offset


def lidar_candidates(xyz, r, dr=0.5, z_lo=-1.0, z_hi=0.6, cell=0.15, min_pts=20, max_extent=1.0, min_height=0.25):
    """Compact upright clusters in the range shell r ± dr, as (azimuth deg, centroid xyz, n_points).

    Floor returns (z below z_lo in the lidar frame) and the ceiling are dropped first; the rest is
    binned on a cell-sized xy grid and 8-connected cells are merged. A board is a cluster that is
    small in xy (under max_extent) but tall (over min_height).
    """
    rho = np.hypot(xyz[:, 0], xyz[:, 1])
    sel = (np.abs(rho - r) < dr) & (xyz[:, 2] > z_lo) & (xyz[:, 2] < z_hi)
    p = xyz[sel]
    if len(p) < min_pts:
        return []
    ij = np.floor(p[:, :2] / cell).astype(int); ij -= ij.min(axis=0)
    grid = np.zeros(ij.max(axis=0) + 3, dtype=int); grid[ij[:, 0] + 1, ij[:, 1] + 1] = 1
    label = np.zeros_like(grid); n = 0
    for i, j in zip(*np.nonzero(grid)):                         # flood fill, 8-connected
        if label[i, j]: continue
        n += 1; stack = [(i, j)]; label[i, j] = n
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    if grid[a + da, b + db] and not label[a + da, b + db]:
                        label[a + da, b + db] = n; stack.append((a + da, b + db))
    lab = label[ij[:, 0] + 1, ij[:, 1] + 1]
    out = []
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() < min_pts: continue
        q = p[m]; ext = np.ptp(q[:, :2], axis=0).max(); height = np.ptp(q[:, 2])
        if ext > max_extent or height < min_height: continue
        c = q.mean(axis=0); out.append((float(np.degrees(np.arctan2(c[1], c[0]))), c, int(m.sum())))
    return out


def local_cluster(xyz, pred, radius=1.0, z_lo=-1.0, z_hi=0.6, cell=0.1, min_pts=15, min_height=0.25):
    """Centroid of the largest upright cluster within radius of the predicted xy, or None."""
    sel = (np.hypot(xyz[:, 0] - pred[0], xyz[:, 1] - pred[1]) < radius) & (xyz[:, 2] > z_lo) & (xyz[:, 2] < z_hi)
    p = xyz[sel]
    if len(p) < min_pts:
        return None
    ij = np.floor(p[:, :2] / cell).astype(int); ij -= ij.min(axis=0)
    grid = np.zeros(ij.max(axis=0) + 3, dtype=int); grid[ij[:, 0] + 1, ij[:, 1] + 1] = 1
    label = np.zeros_like(grid); n = 0
    for i, j in zip(*np.nonzero(grid)):
        if label[i, j]: continue
        n += 1; stack = [(i, j)]; label[i, j] = n
        while stack:
            a, b = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    if grid[a + da, b + db] and not label[a + da, b + db]:
                        label[a + da, b + db] = n; stack.append((a + da, b + db))
    lab = label[ij[:, 0] + 1, ij[:, 1] + 1]
    best, best_n = None, 0
    for k in range(1, n + 1):
        m = lab == k
        if m.sum() < min_pts or np.ptp(p[m, 2]) < min_height: continue
        if m.sum() > best_n:
            best, best_n = p[m].mean(axis=0), int(m.sum())
    return best


def solve_2d(src, dst):
    """Rigid 2D fit dst ≈ R(yaw) src + t by least squares (Kabsch). Returns yaw (rad), t (2,), residuals."""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = md - R @ ms
    res = dst - (src @ R.T + t)
    return float(np.arctan2(R[1, 0], R[0, 0])), t, res


def run(data_root="data/raw", out_json="calib/extrinsics_radar_lidar.json", fig_dir="figures"):
    shots = {}
    for name, trace in SHOTS.items():
        d = os.path.join(data_root, trace)
        rb = radar_board(d, r_max=R_MAX.get(name, 5.0))
        lt = LidarTrace(d); k = lt.index_at(rb["t"]); xyz = lt.xyz(k)
        cands = lidar_candidates(xyz, rb["r"])
        shots[name] = dict(radar=rb, lidar_scan=k, xyz=xyz, cands=cands)
        print(f"{name}: radar r={rb['r']:.2f} m az={np.degrees(rb['az']):+.1f}° snr={rb['snr']:.0f} dB; "
              f"{len(cands)} lidar candidates at that range: " + ", ".join(f"{c[0]:+.0f}°({c[2]})" for c in cands))

    # Start from the prior that the lidar's -x axis points forward (radar +y -> lidar -x is a +90 deg
    # rotation, no offset), pick the cluster nearest each predicted board position, solve, then
    # re-pick with the solved transform.
    yaw, t = np.pi / 2, np.zeros(2)
    for it in range(3):
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        src, dst, used = [], [], []
        for name, s in shots.items():
            rb = s["radar"]
            if rb["r"] < 0.4:
                if it == 0: print(f"  {name}: board at {rb['r']:.2f} m is inside the radar leakage zone, skipped")
                continue
            p_r = np.array([-rb["r"] * np.sin(rb["az"]), rb["r"] * np.cos(rb["az"])])   # radar frame: x right, y forward
            pred = R @ p_r + t
            gate = 1.0 if it == 0 else 0.5
            best = local_cluster(s["xyz"], pred, radius=gate)
            if best is None:
                if it == 2: print(f"  {name}: no upright cluster within {gate} m of the predicted spot, skipped")
                s.pop("board", None); continue
            src.append(p_r); dst.append(best[:2]); used.append(name); s["board"] = best
        src, dst = np.array(src), np.array(dst)
        yaw, t, res = solve_2d(src, dst)
        print(f"iteration {it}: {len(used)} shots, yaw {np.degrees(yaw):+.1f}°, t ({t[0]:+.2f}, {t[1]:+.2f}) m, "
              f"mean residual {np.hypot(res[:, 0], res[:, 1]).mean()*100:.1f} cm")
    err = np.hypot(res[:, 0], res[:, 1])
    print(f"radar->lidar: yaw {np.degrees(yaw):+.2f}°, t = ({t[0]:+.3f}, {t[1]:+.3f}) m, "
          f"residual mean {err.mean()*100:.1f} cm, max {err.max()*100:.1f} cm over {len(used)} shots")
    for n, e, s_, d_ in zip(used, err, src, dst):
        print(f"  {n}: radar ({s_[0]:+.2f}, {s_[1]:+.2f}) -> lidar ({d_[0]:+.2f}, {d_[1]:+.2f}), {e*100:.1f} cm, "
              f"board z {shots[n]['board'][2]:+.2f} m, radar el {np.degrees(shots[n]['radar']['el']):+.1f}°")

    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    json.dump(dict(frame_from="radar (x right, y forward, z up)", frame_to="lidar (sensor frame)",
                   yaw_deg=float(np.degrees(yaw)), t_xy_m=[float(t[0]), float(t[1])], t_z_m=None,
                   range_offset_m=RANGE_OFFSET, shots=used, residual_cm=[float(e * 100) for e in err],
                   note="2D solve; pitch/roll/z not estimated (radar elevation too noisy)"),
              open(out_json, "w"), indent=1)
    figures(shots, used, src, dst, yaw, t, res, fig_dir)
    return shots, yaw, t


def figures(shots, used, src, dst, yaw, t, res, fig_dir):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(fig_dir, exist_ok=True)
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

    # per shot: lidar top view with the board cluster and the transformed radar point
    fig, axes = plt.subplots(2, 4, figsize=(16.64, 7.2), constrained_layout=True)
    for ax, name in zip(axes.flat, shots):
        s = shots[name]; xyz = s["xyz"]; rb = s["radar"]
        ax.scatter(xyz[:, 0], xyz[:, 1], s=.4, c="#B9C2CC")
        ax.add_patch(plt.Circle((0, 0), rb["r"], fill=False, color="#2C43B0", ls=":", lw=.8))
        if "board" in s:
            ax.scatter(s["board"][0], s["board"][1], s=90, color="#E07B39", marker="s", label="board in LiDAR")
            p = R @ np.array([-rb["r"] * np.sin(rb["az"]), rb["r"] * np.cos(rb["az"])]) + t
            ax.scatter(p[0], p[1], s=90, color="#2C43B0", marker="x", lw=2, label="radar, transformed")
        else:
            ax.text(0, -4.3, "skipped", ha="center", color="#E07B39", fontsize=12)
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_aspect("equal")
        ax.set_title(f"{name}: r {rb['r']:.2f} m, az {np.degrees(rb['az']):+.0f}°", fontsize=12)
        ax.tick_params(labelsize=9)
    axes.flat[0].legend(fontsize=9, loc="lower left")
    fig.suptitle("LiDAR top view per calibration shot (grey, lidar frame), dotted ring = radar range, "
                 "board found in LiDAR (orange), radar detection after the solved transform (blue x)", fontsize=12)
    fig.savefig(os.path.join(fig_dir, "calib_radar_lidar_shots.png"), dpi=130); plt.close(fig)

    # the solve
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 5.4), constrained_layout=True)
    fit = src @ R.T + t
    for a, pts, title in [(ax[0], src, "a · before: radar frame vs lidar frame"), (ax[1], fit, "b · after: radar points transformed into the lidar frame")]:
        a.scatter(dst[:, 0], dst[:, 1], s=80, color="#E07B39", marker="s", label="board in LiDAR")
        a.scatter(pts[:, 0], pts[:, 1], s=80, color="#2C43B0", marker="x", lw=2, label="board from radar")
        for p, q, n in zip(pts, dst, used):
            a.plot([p[0], q[0]], [p[1], q[1]], color="#566373", lw=1); a.annotate(n[-3:], q, fontsize=9, xytext=(4, 4), textcoords="offset points")
        a.scatter(0, 0, marker="^", s=120, color="#15202B"); a.set_aspect("equal"); a.set_xlim(-4.5, 4.5); a.set_ylim(-1, 5)
        a.set_xlabel("x (m)"); a.set_ylabel("y (m)"); a.set_title(title); a.legend(fontsize=10, loc="upper left")
    err = np.hypot(res[:, 0], res[:, 1]) * 100
    ax[2].bar(range(len(used)), err, color="#2C43B0"); ax[2].set_xticks(range(len(used))); ax[2].set_xticklabels([u[6:] for u in used])
    ax[2].set_ylabel("residual (cm)"); ax[2].set_title(f"c · residual per shot, mean {err.mean():.1f} cm")
    ax[2].text(.02, .95, f"yaw {np.degrees(yaw):+.1f}°\nt = ({t[0]:+.2f}, {t[1]:+.2f}) m\n2D solve, {len(used)} shots",
               transform=ax[2].transAxes, va="top", fontsize=12, family="monospace")
    fig.savefig(os.path.join(fig_dir, "calib_radar_lidar_solve.png"), dpi=150); plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--data", default="data/raw"); a = ap.parse_args()
    run(a.data)
