"""Close-up of one object as the three sensors see it.

    python -m src.recon.closeup data/raw/2026-10-29.16-22-36 --centre -12.45 -2.60 \
        --far 170 --near 174 --orbit 175 229 --split --out figures/closeup_scotty.png

Panels: two camera frames (one a few metres out, one closer) with the LiDAR scan of that instant and the
radar detections projected on them through the joint calibration (a, b); the LiDAR cut-out of the object
from the one close scan (c) and from the whole orbit (d); the top view with the radar points of the orbit
(e); the range-azimuth power of the far frame (f). --split also writes <out>_1.png (a-c) and <out>_2.png (d-f).

--centre is the object's position in the world frame of data/poses/<trace> (first LiDAR scan), a guess
good to half a metre. Per scan the centre is then refined on the object's pedestal: a circle of fixed
radius (fitted once on the far scan, where most of the arc is visible) is fitted to the LiDAR points
between --slice heights, two parameters, Gauss-Newton with a shrinking inlier band. That takes the
odometry's jitter and slow drift out of the LiDAR cut-out (its yaw error stays); the radar points keep
theirs, they come from data/recon/<trace>/radar_points.npz. Radar points carry no elevation, so they are
drawn at the radar's height. Below ~1.5 m the LiDAR-camera projection drifts off the photo (the solved
translation is only good to 0.1 m and parallax grows as 1/range), so keep --near at 2 m or more.
"""
import argparse, json, os, subprocess

import numpy as np

from ..radar import RadarTrace, detect
from ..radar.pointcloud import to_xy, RANGE_OFFSET
from ..radar.angle import AZIMUTHS
from ..calib.lidar import LidarTrace
from ..calib.transform import RadarToLidar, RadarToCamera
from ..calib.board_image import project_fisheye
from .accumulate import height_above_ground

W, H = 1920, 1080
HCMAP, HMAX = "turbo", 2.2
TITLE = dict(fontsize=9)


def camera_frame(trace, t_unix, path):
    ts = np.fromfile(os.path.join(trace, "camera", "ts"), "<f8"); kc = int(np.searchsorted(ts, t_unix))
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{kc / 30:.3f}", "-i", os.path.join(trace, "camera", "video.avi"), "-frames:v", "1", path], check=True)
    import matplotlib.image as mpimg
    img = mpimg.imread(path); os.remove(path); return img, ts[kc]


def inside(uv):
    return (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)


def circle_free(xy):
    """Algebraic circle fit (centre, radius); fine when most of the arc is visible."""
    A = np.c_[2 * xy, np.ones(len(xy))]; sol = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)[0]
    return sol[:2], np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2)


def circle_fixed(xy, c0, radius, tols=(0.3, 0.2, 0.12, 0.08, 0.06, 0.05)):
    """Centre of a circle of known radius through the points: Gauss-Newton with a shrinking inlier band,
    started from the free fit when that is sane, else from c0. Returns (centre, share of points within 5 cm)."""
    c = np.array(c0, float)
    if len(xy) >= 100:
        cf, rf = circle_free(xy)
        if np.hypot(*(cf - c)) < 1.0 and 0.5 * radius < rf < 2 * radius: c = cf
    for tol in tols:
        d = xy - c; rho = np.linalg.norm(d, axis=1); res = rho - radius; w = np.abs(res) < tol
        if w.sum() < 30: return c, 0.0
        c = c + np.linalg.lstsq(-d[w] / rho[w, None], -res[w], rcond=None)[0]
    d = xy - c; return c, float((np.abs(np.linalg.norm(d, axis=1) - radius) < 0.05).mean())


class Scan:
    """One LiDAR scan with its pose, its ground, and the object's centre refined on the pedestal."""

    def __init__(self, lt, L, t_unix, c_world, radius=None, slice_h=(0.45, 1.0), reach=1.5):
        self.k = lt.index_at(t_unix); self.j = int(np.argmin(np.abs(L["t"] - t_unix))); self.T = L["T"][self.j]
        xyz = lt.xyz(self.k); rho = np.linalg.norm(xyz[:, :2], axis=1); self.xyz = xyz[rho > 0.5]
        self.h = height_above_ground(self.xyz)
        c0 = (self.T[:3, :3].T @ (np.r_[c_world, self.T[2, 3]] - self.T[:3, 3]))[:2]       # odometry's idea of the centre, this frame
        m = (np.hypot(*(self.xyz[:, :2] - c0).T) < reach) & (self.h > slice_h[0]) & (self.h < slice_h[1])
        if radius is None: self.c, self.radius = circle_free(self.xyz[m, :2]); self.fit = 1.0
        else: self.c, self.fit = circle_fixed(self.xyz[m, :2], c0, radius); self.radius = radius
        self.c0, self.n_slice = c0, int(m.sum())

    def dist(self):
        return float(np.hypot(*self.c))

    def cut(self, r):
        return np.hypot(*(self.xyz[:, :2] - self.c).T) < r

    def world_xy(self, xy):
        return xy @ self.T[:2, :2].T + self.T[:2, 3]


def axes3d(ax, R, look):
    ax.view_init(elev=14, azim=look); ax.set_xlim(-R, R); ax.set_ylim(-R, R); ax.set_zlim(0, 2.4); ax.set_box_aspect((1, 1, 1.05))
    ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8); ax.set_zlabel("height (m)", fontsize=8); ax.tick_params(labelsize=7)
    for p in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane): p.fill = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--centre", type=float, nargs=2, required=True); ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius (m)")
    ap.add_argument("--far", type=float, required=True); ap.add_argument("--near", type=float, required=True)
    ap.add_argument("--orbit", type=float, nargs=2, required=True, help="start, end (s) of the orbit used for the LiDAR cut-out")
    ap.add_argument("--slice", type=float, nargs=2, default=(0.45, 1.0), help="heights of the pedestal slice the centre is fitted on")
    ap.add_argument("--poses-dir", default="data/poses"); ap.add_argument("--accumulate", type=int, default=10); ap.add_argument("--snr", type=float, default=10)
    ap.add_argument("--split", action="store_true"); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    name = os.path.basename(a.trace.rstrip("/")); c_guess = np.array(a.centre)
    tr = RadarTrace(a.trace); lt = LidarTrace(a.trace); t0 = tr.ts[0]
    P = np.load(os.path.join(a.poses_dir, name, "radar.npz")); L = np.load(os.path.join(a.poses_dir, name, "lidar.npz"))
    T_rl = RadarToLidar.load_joint(); C_rc = RadarToCamera.load_joint()
    lc = json.load(open("calib/extrinsics_joint.json"))["lidar_to_camera"]; R_lc, t_lc = np.array(lc["R"]), np.array(lc["t_m"])
    rec = np.load(os.path.join("data/recon", name, "radar_points.npz"))
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

    # pedestal radius from the far scan, then every scan's centre with that radius
    radius = Scan(lt, L, t0 + a.far, c_guess, None, a.slice).radius; print(f"pedestal radius from the far scan: {radius:.2f} m")

    # a, b: camera frames with the LiDAR cut-out and the radar detections projected
    cams = []
    for tag, tsec in [("a", a.far), ("b", a.near)]:
        img, t_cam = camera_frame(a.trace, t0 + tsec, a.out + f"_{tag}.png"); s = Scan(lt, L, t_cam, c_guess, radius, a.slice); kr = tr.index_at(t_cam)
        pts = np.concatenate([detect(tr.frame(k), tr.params)[0] for k in range(kr - a.accumulate // 2, kr + a.accumulate // 2)]); pts = pts[pts["snr"] > a.snr]
        xy_r = to_xy(pts); on = np.hypot(*(T_rl(xy_r)[:, :2] - s.c).T) < radius + 0.5; uv_r = C_rc.pixels(xy_r); m_r = inside(uv_r) & (C_rc(xy_r)[:, 2] > 0.3)
        cut = s.cut(a.radius) & (s.h > 0.05); lid_cam = s.xyz[cut] @ R_lc.T + t_lc; ok = lid_cam[:, 2] > 0.3; uv_l = project_fisheye(lid_cam[ok], C_rc.cam); q = inside(uv_l)
        cams.append(dict(tag=tag, tsec=tsec, img=img, s=s, uv_l=uv_l[q], h_l=s.h[cut][ok][q], uv_r=uv_r, m_r=m_r, on=on, pts=pts, kr=kr, n_cut=int(cut.sum())))
        print(f"{tag}: t = {tsec} s, object {s.dist():.2f} m away (odometry said {np.hypot(*s.c0):.2f}), fit {s.fit:.2f}; {cut.sum()} LiDAR points on it; "
              f"radar {on.sum()} of {len(pts)} points on it, r {np.unique(pts['r'][on].round(2))}, SNR max {pts['snr'][on].max():.0f}")

    def panel_camera(ax, d):
        s, pts, on, m_r = d["s"], d["pts"], d["on"], d["m_r"]
        ax.imshow(d["img"]); ax.scatter(d["uv_l"][:, 0], d["uv_l"][:, 1], s=1.5, c=d["h_l"], cmap=HCMAP, vmin=0, vmax=HMAX, linewidths=0, alpha=.9)
        ax.scatter(d["uv_r"][m_r & ~on, 0], d["uv_r"][m_r & ~on, 1], s=70, facecolors="none", edgecolors="white", linewidths=1.2)
        ax.scatter(d["uv_r"][m_r & on, 0], d["uv_r"][m_r & on, 1], s=110, facecolors="none", edgecolors="#FF3B30", linewidths=2.2)
        ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")
        ax.set_title(f"{d['tag']} · camera t = {d['tsec']:.0f} s, object {s.dist():.1f} m away: LiDAR scan on it (dots = height)\n"
                     f"radar, {a.accumulate} frames (circles): {on.sum()} on the object at r {pts['r'][on].min():.1f}–{pts['r'][on].max():.1f} m, SNR ≤ {pts['snr'][on].max():.0f} dB", **TITLE)

    # c: the one close scan in 3D
    s_near = cams[1]["s"]; cut_c = s_near.cut(a.radius) & (s_near.h > 0.05); p_c, h_c = s_near.xyz[cut_c], s_near.h[cut_c]
    look_c = np.degrees(np.arctan2(-s_near.c[1], -s_near.c[0])) + 25                   # viewer a little to the side of the sensor

    def panel_scan3d(ax):
        ax.scatter(p_c[:, 0] - s_near.c[0], p_c[:, 1] - s_near.c[1], h_c, s=.8, c=h_c, cmap=HCMAP, vmin=0, vmax=HMAX, linewidths=0, depthshade=False)
        axes3d(ax, a.radius, look_c)
        ax.set_title(f"c · LiDAR, the one scan of b: {len(p_c):,} points on the object from {s_near.dist():.1f} m\n"
                     f"(beam spacing there ~{1000 * s_near.dist() * np.radians(360 / 2048):.0f} mm across, ~{1000 * s_near.dist() * np.radians(90 / 127):.0f} mm up)", **TITLE)

    # d: the whole orbit, every 2nd scan, each re-centred on the pedestal, world orientation about the object
    sel = np.flatnonzero((L["t"] >= t0 + a.orbit[0]) & (L["t"] < t0 + a.orbit[1]))[::2]
    pts_o, h_o, fits, centres = [], [], [], []
    for j in sel:
        s = Scan(lt, L, L["t"][j], c_guess, radius, a.slice)
        if s.fit < 0.3 or s.n_slice < 200: continue
        cut = s.cut(a.radius) & (s.h > 0.05); pts_o.append((s.xyz[cut, :2] - s.c) @ s.T[:2, :2].T); h_o.append(s.h[cut]); fits.append(s.fit); centres.append(s.world_xy(s.c))
    pts_o, h_o, centres = np.concatenate(pts_o), np.concatenate(h_o), np.array(centres); c_ref = centres.mean(0)
    odo = c_guess - c_ref; jitter = float(np.hypot(*centres.std(0)))
    pos_w = P["T"][:, :2, 3]; k_far = int(np.argmin(np.abs(P["t"] - (t0 + a.far)))); look_d = np.degrees(np.arctan2(*(pos_w[k_far] - c_ref)[::-1])) + 25
    print(f"orbit: {len(fits)} of {len(sel)} scans used, fit inlier share median {np.median(fits):.2f}; refined centre (world) {c_ref.round(2)}, "
          f"guess was {np.hypot(*odo):.2f} m off; scan centres scatter {jitter:.2f} m about the mean; {len(pts_o):,} points")

    def panel_orbit3d(ax):
        step = max(1, len(pts_o) // 400_000)
        ax.scatter(pts_o[::step, 0], pts_o[::step, 1], h_o[::step], s=.4, c=h_o[::step], cmap=HCMAP, vmin=0, vmax=HMAX, linewidths=0, depthshade=False)
        axes3d(ax, a.radius, look_d)
        ax.set_title(f"d · LiDAR, {len(fits)} orbit scans ({a.orbit[0]:.0f}–{a.orbit[1]:.0f} s), each re-centred on the pedestal\n"
                     f"(r = {radius:.2f} m circle fitted at {a.slice[0]}–{a.slice[1]} m; odometry centres scatter {jitter:.2f} m)", **TITLE)

    # e: top view with the radar points of the orbit
    R2 = 2.5; rx = rec["xyz"][:, :2] - c_ref; nr = np.hypot(*rx.T) < R2; st = rec["static"]; ps = (P["t"] >= t0 + a.orbit[0]) & (P["t"] < t0 + a.orbit[1])
    on_ped = (np.hypot(*rx[nr & st].T) < radius + 0.35).sum()

    def panel_top(ax, fig):
        m = h_o > 0.3; ax.scatter(pts_o[m][::4, 0], pts_o[m][::4, 1], s=.3, c="#AEB7C2", linewidths=0, label="LiDAR of d, > 0.3 m")
        ax.scatter(rx[nr & ~st, 0], rx[nr & ~st, 1], s=7, c="#E8C4C4", linewidths=0, label="radar, failed the static test")
        sc = ax.scatter(rx[nr & st, 0], rx[nr & st, 1], s=12, c=rec["snr"][nr & st], cmap="magma", vmin=10, vmax=30, linewidths=0, label="radar, static")
        ax.plot(pos_w[ps, 0] - c_ref[0], pos_w[ps, 1] - c_ref[1], color="#2C43B0", lw=1, label="sensor path"); ax.add_patch(plt.Circle((0, 0), radius, fill=False, color="#15202B", lw=1, ls="--"))
        ax.set_xlim(-R2, R2); ax.set_ylim(-R2, R2); ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(loc="upper left", fontsize=7.5, markerscale=2)
        ax.set_title(f"e · top view: {(nr & st).sum():,} static radar points, {on_ped:,} at the pedestal (dashed);\n"
                     f"{(nr & ~st).sum():,} rejected (pink); the ring round the path is the operator", **TITLE)
        fig.colorbar(sc, ax=ax, shrink=.75, label="radar SNR (dB)")

    # f: range-azimuth power of the far frame with the LiDAR of that instant
    s_far = cams[0]["s"]; pts_f, info = detect(tr.frame(cams[0]["kr"]), tr.params); db = 10 * np.log10(info["power"] + 1e-12)
    rng = tr.params.ranges - RANGE_OFFSET; AZ, RG = np.meshgrid(AZIMUTHS, rng, indexing="ij"); X, Y = -RG * np.sin(AZ), RG * np.cos(AZ)
    lid_r = T_rl.inverse(s_far.xyz[(s_far.h > 0.3) & (s_far.h < 2.5)]); c_r = T_rl.inverse(np.r_[s_far.c, 0.0][None])[0]; xy_d = to_xy(pts_f)
    pk = db[np.hypot(X - c_r[0], Y - c_r[1]) < radius + 0.4].max(); floor = np.median(db); r_max = RG.flat[db.argmax()]
    print(f"f: object peak {pk - floor:.0f} dB above the floor, {db.max() - pk:.0f} dB below the strongest return at {r_max:.1f} m; "
          f"CFAR detections on it: {[(round(float(p['r']), 2), round(float(np.degrees(p['az']))), round(float(p['snr']))) for p in pts_f if np.hypot(-p['r'] * np.sin(p['az']) - c_r[0], p['r'] * np.cos(p['az']) - c_r[1]) < radius + 0.4]}")

    def panel_power(ax, fig):
        R3 = 7.0
        pm = ax.pcolormesh(X, Y, db, cmap="inferno", vmin=np.percentile(db, 50), vmax=np.percentile(db, 99.8), shading="nearest", rasterized=True)
        ax.scatter(lid_r[:, 0], lid_r[:, 1], s=.3, c="#8FD3FF", linewidths=0, alpha=.5); ax.add_patch(plt.Circle(c_r[:2], radius, fill=False, color="white", lw=1.2, ls="--"))
        ax.scatter(xy_d[:, 0], xy_d[:, 1], s=14, marker="x", c="#00E5A0", linewidths=1)
        ax.set_xlim(-R3, R3); ax.set_ylim(0, 2 * R3); ax.set_aspect("equal"); ax.set_xlabel("x radar (m)"); ax.set_ylabel("y radar (m)")
        ax.set_title(f"f · radar range–azimuth power, the frame of a; LiDAR 0.3–2.5 m (blue), CFAR (x)\n"
                     f"object (dashed): peak {pk - floor:.0f} dB above the floor, {db.max() - pk:.0f} dB below the wall at {r_max:.1f} m", **TITLE)
        fig.colorbar(pm, ax=ax, shrink=.75, label="power (dB)")

    sup = f"{name}: one object up close — camera, LiDAR and radar (joint calibration, KISS-ICP poses)"
    layouts = [("", (2, 3), (16.64, 9.6), "abcdef")] + ([("_1", (1, 3), (16.64, 5.3), "abc"), ("_2", (1, 3), (16.64, 5.3), "def")] if a.split else [])
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    for suffix, shape, size, panels in layouts:
        fig = plt.figure(figsize=size, constrained_layout=True); gs = fig.add_gridspec(*shape)
        for i, p in enumerate(panels):
            cell = gs[i // shape[1], i % shape[1]]
            if p in "ab": panel_camera(fig.add_subplot(cell), cams["ab".index(p)])
            elif p == "c": panel_scan3d(fig.add_subplot(cell, projection="3d"))
            elif p == "d": panel_orbit3d(fig.add_subplot(cell, projection="3d"))
            elif p == "e": panel_top(fig.add_subplot(cell), fig)
            elif p == "f": panel_power(fig.add_subplot(cell), fig)
        fig.suptitle(sup, fontsize=12); path = a.out.replace(".png", f"{suffix}.png"); fig.savefig(path, dpi=110); plt.close(fig); print("saved", path)


if __name__ == "__main__":
    main()
