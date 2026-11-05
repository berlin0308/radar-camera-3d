"""The seven figures of notes/sessions/2026-11-05.md. Figures with camera frames go to data/sessions/2026-11-05/ only.

    python notes/sessions/2026-11-05_figures.py fig1    # 11/05 session health: every trace on the clock, the cut walk stream by stream
    python notes/sessions/2026-11-05_figures.py fig2    # 11/03 board shots: where the boards were, what the radar read
    python notes/sessions/2026-11-05_figures.py fig3    # static scenes: range-azimuth power, person minus empty room, the car scene
    python notes/sessions/2026-11-05_figures.py fig4    # rig geometry from the 11/03 solve, side and top view
    python notes/sessions/2026-11-05_figures.py fig7    # Scotty at dusk (11/03) and by day (11/05): camera brightness against the radar return
    # fig5 and fig6 are the repo's own tools:
    python -m src.recon.poses data/raw/2026-11-05.16-46-03 --seconds 616 --out figures/poses_walk_1105.png
    python -m src.recon.ego_check data/raw/2026-11-05.16-46-03 --seconds 616 --frames 74 222 --out figures/s4_fig5_ego_walk.png
    python -m src.calib.overlay data/raw/2026-11-05.16-10-35 --time 30 --accumulate 200 --rmax 20 --joint --out figures/s4_fig6_overlay_static_b.png
"""
import datetime as dt, json, os, sys

import numpy as np

sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

BLUE, ORANGE, RED, INK, MUTED, GRID = "#2C43B0", "#E07B39", "#B3261E", "#15202B", "#566373", "#E6EBF2"
LOCAL = "data/sessions/2026-11-05"
D = lambda n: f"data/raw/2026-11-05.{n}"
TRACES = [("15-43-05", "lab, empty room"), ("15-44-13", "lab, a person squatting at about 2 m"), ("16-10-35", "outdoors static: small car, building, tripods"),
          ("16-12-32", "same spot; the car leaves at the end"), ("16-25-42", "walk to the Scotty plaza"), ("16-34-29", "Scotty static, tripods either side"),
          ("16-36-41", "Scotty static, second take"), ("16-38-55", "Scotty orbit (reconstruction, Yining)"), ("16-46-03", "campus walk, indoors, elevator, back to the lab; cut off")]


def style(ax):
    ax.spines[["top", "right"]].set_visible(False); ax.grid(color=GRID, lw=.8); ax.set_axisbelow(True); ax.tick_params(labelsize=10)


def ts(t, s):
    return np.fromfile(f"{t}/{s}/ts", "<f8")


def fig1(path="figures/s4_fig1_session_1105.png"):
    fig, ax = plt.subplots(2, 1, figsize=(16.64, 9.0), constrained_layout=True, gridspec_kw=dict(height_ratios=[1.15, 1]))
    a = ax[0]; nom = dict(radar=20, lidar=10, camera=30, imu=100)
    for i, (n, what) in enumerate(TRACES):
        t = D(n); c = ts(t, "camera"); c0 = c[0]; t0 = dt.datetime.fromtimestamp(c0); x0 = t0.hour * 60 + t0.minute + t0.second / 60; y = len(TRACES) - i
        v = np.fromfile(f"{t}/radar/valid", "u1"); r = ts(t, "radar"); end = max(ts(t, s)[-1] for s in nom) - c0
        if n == "16-46-03":
            ok = r[len(v) - 1] - c0
            a.barh(y, ok / 60, left=x0, height=.55, color=BLUE)
            a.barh(y, (end - ok) / 60, left=x0 + ok / 60, height=.55, color="white", edgecolor=RED, hatch="///", lw=1)
            a.annotate(f"radar 'valid' stops at {ok:.0f} s; streams end between 781 and 823 s", (x0 + ok / 60, y + .32), fontsize=10, color=RED)
        else:
            a.barh(y, end / 60, left=x0, height=.55, color=BLUE)
        bad = r[np.nonzero(v == 0)[0]] - c0
        if len(bad): a.scatter(x0 + bad / 60, np.full(len(bad), y + .38), marker="v", s=40, color=RED, zorder=4)
        lg = np.diff(ts(t, "lidar")); a.text(x0 + end / 60 + .4, y, f"{n}  {end:.0f} s · {what}" + (f" · radar invalid {len(bad)}" if len(bad) else "") + (f" · LiDAR gaps > 0.3 s: {(lg > .3).sum()}" if (lg > .3).sum() else ""),
                                             va="center", fontsize=10.5, color=INK)
    a.set_yticks([]); a.set_xlim(15 * 60 + 40, 17 * 60 + 25)
    a.set_xticks(range(15 * 60 + 40, 17 * 60 + 26, 10)); a.set_xticklabels([f"{m // 60}:{m % 60:02d}" for m in range(15 * 60 + 40, 17 * 60 + 26, 10)])
    a.set_xlabel("clock time, 2026-11-05 (EST)"); style(a); a.grid(axis="y", visible=False)
    a.set_title("a · every trace of the session; red triangles = radar frames with dropped packets; all nine pass roverd validate", loc="left", fontsize=12)
    b = ax[1]; t = D("16-46-03"); c0 = ts(t, "camera")[0]
    for k, (s, col) in enumerate([("radar", BLUE), ("lidar", ORANGE), ("camera", "#3B8B5A"), ("imu", MUTED)]):
        x = ts(t, s) - c0; h, e = np.histogram(x, bins=np.arange(0, 830, 2)); b.plot(e[:-1] + 1, h / 2 / nom[s] + k * 0.0, color=col, lw=1.4, label=f"{s} ({nom[s]} Hz nominal)")
    v = np.fromfile(f"{t}/radar/valid", "u1"); r = ts(t, "radar")
    for x, txt, yy, ha in [(297, "camera index ends\n(rewrapped: all frames read)", 1.17, "left"), (r[len(v) - 1] - c0, "radar 'valid' ends", .35, "left"), (ts(t, "lidar")[-1] - c0, "LiDAR ends 781 s", 1.17, "right"),
                           (r[-1] - c0, "radar ts ends 796 s", .62, "right"), (ts(t, "imu")[-1] - c0, "IMU ends 804 s", .5, "right")]:
        b.axvline(x, color=RED if "valid" in txt else MUTED, ls="--", lw=1); b.text(x + (3 if ha == "left" else -3), yy, txt, fontsize=9.5, color=RED if "valid" in txt else MUTED, va="top", ha=ha)
    b.axvspan(0, 616.7, color=BLUE, alpha=.06); b.text(10, .12, "safe for everything: 0 to 616 s", fontsize=11, color=BLUE)
    b.set_ylim(0, 1.2); b.set_xlim(0, 830); b.set_xlabel("time since the camera started (s)"); b.set_ylabel("frames per second / nominal"); style(b); b.legend(fontsize=10, loc="lower left", frameon=False, ncol=4, bbox_to_anchor=(0, 0.17))
    b.set_title("b · 16-46-03 stream by stream: the recording was cut, not stopped (every file ends on a 4 KiB boundary; radar has 16,390 frames of I/Q but 15,872 timestamps and 12,288 'valid' flags)", loc="left", fontsize=12)
    fig.savefig(path, dpi=130); print("saved", path)


SHOT_NAMES = {"calib2_01": "calib_01", "calib2_02": "calib_02", "calib2_03": "calib_03", "calib2_04": "calib_04", "calib2_05": "calib_05", "calib2_06": "calib_06", "calib2_07": "calib_07c", "calib2_08": "calib_08b"}


def fig2(path="figures/s4_fig2_board_placements.png"):
    from src.calib import plane_solve as ps
    rows = ps.load_rows(); x0, tz = ps.seed_x(); sol = ps.solve_radar(rows, x0, tz); R_lr, t = ps._unpack(sol["x"], tz)
    fig, ax = plt.subplots(1, 2, figsize=(16.64, 6.6), constrained_layout=True, gridspec_kw=dict(width_ratios=[1.25, 1]))
    a = ax[0]; a.scatter(0, 0, marker="^", s=120, color=INK, zorder=5); a.text(0.08, -0.12, "radar", fontsize=10)
    nominal, centre_s, solved = [], [], []
    for s in rows:
        C = s["corners"][[0, 1]] @ R_lr.T + t; c = s["centre"] @ R_lr.T + t; _, _, q = ps.predict(sol["x"], tz, [s]); q = q[0] @ R_lr.T + t
        rx, ry = -s["r"] * np.sin(s["az"]), s["r"] * np.cos(s["az"])
        a.plot(C[:, 0], C[:, 1], color=MUTED, lw=2.5, solid_capstyle="round"); a.scatter(*c[:2], s=40, facecolor="white", edgecolor=ORANGE, lw=1.6, zorder=4)
        a.scatter(rx, ry, marker="x", s=60, color=BLUE, lw=2, zorder=5); a.annotate(SHOT_NAMES[s["name"]], (c[0], c[1]), xytext=(6, 8), textcoords="offset points", fontsize=10)
        # bearing of the board centre with the LiDAR taken as co-located and aligned (no extrinsics), and after the solve
        nominal.append(np.degrees(s["az"] - np.arctan2(-s["centre"][1], -s["centre"][0])))   # LiDAR frame: forward = -x, left = -y
        solved.append(np.degrees(s["az"] - np.arctan2(-q[0], q[1]))); centre_s.append(np.degrees(s["az"] - np.arctan2(-c[0], c[1])))
    a.scatter([], [], facecolor="white", edgecolor=ORANGE, label="board centre (LiDAR plane fit)"); a.plot([], [], color=MUTED, lw=2.5, label="board, 0.925 m"); a.scatter([], [], marker="x", color=BLUE, label="radar reading (range, azimuth)")
    a.set_aspect("equal"); a.set_xlim(-2.4, 2.4); a.set_ylim(-0.3, 3.4); a.set_xlabel("x, right (m)"); a.set_ylabel("y, forward (m)"); style(a); a.legend(fontsize=10, loc="lower right", frameon=False)
    a.set_title("a · the eight board placements of 2026-11-03 in the radar frame (11/03 solve)", loc="left", fontsize=12)
    b = ax[1]; names = [SHOT_NAMES[s["name"]] for s in rows]; xs = np.arange(len(rows)); w = .27
    b.bar(xs - w, nominal, w * .92, color="#B9C2CC", label=f"no extrinsics, board centre: mean {np.mean(nominal):+.1f}°, |mean| {np.mean(np.abs(nominal)):.1f}°")
    b.bar(xs, centre_s, w * .92, color=ORANGE, label=f"11/03 solve, board centre: mean {np.mean(centre_s):+.1f}°, |mean| {np.mean(np.abs(centre_s)):.1f}°")
    b.bar(xs + w, solved, w * .92, color=BLUE, label=f"11/03 solve, specular point: mean {np.mean(solved):+.1f}°, |mean| {np.mean(np.abs(solved)):.1f}°")
    for x_, v_ in zip(xs - w, nominal): b.text(x_, v_ - (0.4 if v_ < 0 else -0.2), f"{v_:.0f}", ha="center", va="top" if v_ < 0 else "bottom", fontsize=8.5, color=MUTED)
    b.axhline(0, color=MUTED, lw=.8); b.set_xticks(xs); b.set_xticklabels(names, rotation=30, fontsize=10); b.set_ylabel("radar azimuth minus expected (deg, + = left)"); style(b); b.legend(fontsize=10, frameon=False, loc="upper left")
    b.set_title("b · radar azimuth minus where it should point, per shot", loc="left", fontsize=12); b.set_ylim(min(-16, min(nominal) - 2), 9)
    fig.savefig(path, dpi=130); print("saved", path, "nominal", np.round(nominal, 1), "centre", np.round(centre_s, 1), "solved", np.round(solved, 1))
    return nominal, solved


def mean_power(t, n_frames=60, t0=None, t1=None):
    from src.radar import RadarTrace, detect
    tr = RadarTrace(t); k = np.arange(len(tr)); tt = tr.ts - ts(t, "camera")[0]
    if t0 is not None: k = k[(tt >= t0) & (tt <= t1)]
    ks = k[np.linspace(0, len(k) - 1, n_frames).astype(int)]; P, pts = None, []
    for j in ks:
        p, info = detect(tr.frame(j), tr.params); P = info["power"] if P is None else P + info["power"]; pts.append(p)
    return 10 * np.log10(P / len(ks)), np.concatenate(pts), tr


def polar_axes(tr):
    from src.radar import AZIMUTHS
    from src.radar.pointcloud import RANGE_OFFSET
    dres = json.load(open(os.path.join(tr.path if hasattr(tr, "path") else tr.dir, "radar", "radar.json")))["range_resolution"] if False else tr.params["range_resolution"] if isinstance(tr.params, dict) and "range_resolution" in tr.params else None
    return AZIMUTHS, RANGE_OFFSET, dres


def heat(ax, db, t, tr, rmax, title, vmin=None, vmax=None, cmap="magma", lidar=True, pts=None):
    from src.radar import AZIMUTHS
    from src.radar.pointcloud import RANGE_OFFSET
    from src.calib.lidar import LidarTrace
    from src.calib.transform import RadarToLidar
    dres = json.load(open(f"{t}/radar/radar.json"))["range_resolution"]; r = np.arange(db.shape[1]) * dres - RANGE_OFFSET
    A, Rg = np.meshgrid(AZIMUTHS, r, indexing="ij"); X, Y = -Rg * np.sin(A), Rg * np.cos(A); m = r < rmax
    im = ax.pcolormesh(X[:, m], Y[:, m], db[:, m], cmap=cmap, shading="auto", vmin=vmin, vmax=vmax, rasterized=True)
    if lidar:
        lt = LidarTrace(t); xyz = lt.xyz(len(lt) // 2); floor = np.percentile(xyz[:, 2], 3); xyz = xyz[(xyz[:, 2] > floor + 0.25) & (xyz[:, 2] < floor + 2.2)]
        p = RadarToLidar.load_joint(trace=t).inverse(xyz); ax.scatter(p[::3, 0], p[::3, 1], s=.3, color="#8FD3FF", alpha=.55, linewidths=0)
    if pts is not None and len(pts):
        ax.scatter(-pts["r"] * np.sin(pts["az"]), pts["r"] * np.cos(pts["az"]), s=10, facecolor="none", edgecolor="#7CFC9A", lw=.8)
    ax.set_aspect("equal"); ax.set_xlim(-rmax * .75, rmax * .75); ax.set_ylim(0, rmax); ax.set_title(title, loc="left", fontsize=12); ax.set_xlabel("x, right (m)"); ax.set_ylabel("y, forward (m)"); ax.tick_params(labelsize=10)
    return im, r


def fig3(path="figures/s4_fig3_static_heatmaps.png"):
    from src.radar import AZIMUTHS
    e, _, tre = mean_power(D("15-43-05")); p, pts_p, trp = mean_power(D("15-44-13")); c, pts_c, trc = mean_power(D("16-10-35"))
    fig, ax = plt.subplots(1, 3, figsize=(16.64, 6.0), constrained_layout=True, gridspec_kw=dict(width_ratios=[1, 1, 1.35]))
    im, r = heat(ax[0], p, D("15-44-13"), trp, 6, "a · lab, person squatting (60 frames)", vmin=np.percentile(p, 30), vmax=p.max())
    fig.colorbar(im, ax=ax[0], shrink=.7, label="power (dB)")
    diff = p - e; im2, _ = heat(ax[1], diff, D("15-44-13"), trp, 6, "b · minus the empty room (15-43-05)", vmin=-12, vmax=12, cmap="RdBu_r")
    fig.colorbar(im2, ax=ax[1], shrink=.7, label="dB")
    m = r < 6; i, j = np.unravel_index(np.argmax(np.where(m[None, :], diff, -99)), diff.shape)
    ax[1].scatter(-r[j] * np.sin(AZIMUTHS[i]), r[j] * np.cos(AZIMUTHS[i]), marker="o", s=160, facecolor="none", edgecolor=INK, lw=1.5)
    ax[1].annotate(f"+{diff[i, j]:.0f} dB at {r[j]:.2f} m, {np.degrees(AZIMUTHS[i]):+.0f}°", (-r[j] * np.sin(AZIMUTHS[i]), r[j] * np.cos(AZIMUTHS[i])), xytext=(12, 14), textcoords="offset points", fontsize=11)
    im3, _ = heat(ax[2], c, D("16-10-35"), trc, 16, "c · outdoors (16-10-35): LiDAR blue, CFAR green", vmin=np.percentile(c, 30), vmax=c.max(), pts=pts_c)
    fig.colorbar(im3, ax=ax[2], shrink=.7, label="power (dB)")
    fig.suptitle("Static scenes of 2026-11-05 through our own chain (src.radar): range FFT, Doppler FFT, TDM, 8-element azimuth FFT, summed over Doppler; LiDAR through the 11/03 solve", fontsize=12)
    fig.savefig(path, dpi=130); print("saved", path, f"person peak {diff[i, j]:.1f} dB at {r[j]:.2f} m {np.degrees(AZIMUTHS[i]):+.1f} deg")
    # local version with the camera frames
    import cv2
    fig2_, ax2 = plt.subplots(1, 2, figsize=(16.64, 5), constrained_layout=True)
    for a_, n in zip(ax2, ["15-44-13", "16-10-35"]):
        cap = cv2.VideoCapture(D(n) + "/camera/video.avi"); cap.set(cv2.CAP_PROP_POS_FRAMES, 900); ok, f = cap.read(); a_.imshow(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)); a_.axis("off"); a_.set_title(n, fontsize=12)
    fig2_.savefig(f"{LOCAL}/fig3_camera_LOCAL.png", dpi=90)


def fig4(path="figures/s4_fig4_rig_geometry.png"):
    j = json.load(open("calib/extrinsics_joint.json")); t_r = np.array(j["radar_to_lidar"]["t_m"]); yaw = j["radar_to_lidar"]["yaw_deg"]
    R_lc, t_lc = np.array(j["lidar_to_camera"]["R"]), np.array(j["lidar_to_camera"]["t_m"]); cam = -R_lc.T @ t_lc; axis_c = R_lc.T @ np.array([0, 0, 1.0])
    sr, sc = j["radar_to_lidar"]["sigma"]["t_xy_m"], j["lidar_to_camera"]["sigma"]["t_m"]
    fwd = lambda p: -p[0]; left = lambda p: -p[1]                       # LiDAR frame: -x is forward, -y is left (z up)
    P = {"LiDAR (optical centre)": (np.zeros(3), INK), "radar": (t_r, BLUE), "camera": (cam, ORANGE)}
    fig, ax = plt.subplots(1, 2, figsize=(16.64, 6.2), constrained_layout=True, gridspec_kw=dict(width_ratios=[1, 1.15]))
    for a, (hfun, hlab, vfun, vlab, title) in zip(ax, [(fwd, "forward (cm)", lambda p: p[2], "up (cm)", "a · side view"), (fwd, "forward (cm)", left, "left (cm)", "b · top view")]):
        for name, (p, col) in P.items():
            a.scatter(hfun(p) * 100, vfun(p) * 100, s=160, color=col, zorder=5, edgecolor="white", lw=1.5)
            off = {"camera": (-10, 14), "radar": (12, -30), "LiDAR (optical centre)": (-70, -34)}[name] if a is ax[1] else (12, 8)
            a.annotate(f"{name}\n({fwd(p) * 100:+.1f}, {left(p) * 100:+.1f}, {p[2] * 100:+.1f}) cm", (hfun(p) * 100, vfun(p) * 100), xytext=off, textcoords="offset points", fontsize=10.5)
        # boresights: radar yaw only; camera optical axis from the solve
        ang = np.radians(yaw - 90); br = np.array([-np.cos(ang), np.sin(ang) * -1, 0]) if False else np.array([-np.cos(ang), -np.sin(ang), 0.0])
        for p, d, col in [(t_r, br, BLUE), (cam, axis_c, ORANGE), (np.zeros(3), np.array([-1.0, 0, 0]), INK)]:
            a.annotate("", xy=((hfun(p) + 0.12 * hfun(d)) * 100, (vfun(p) + 0.12 * vfun(d)) * 100), xytext=(hfun(p) * 100, vfun(p) * 100), arrowprops=dict(arrowstyle="->", color=col, lw=1.6))
        a.set_xlabel(hlab); a.set_ylabel(vlab); a.set_aspect("equal"); a.set_xlim(-12, 32); a.set_ylim(*((-35, 12) if a is ax[0] else (-12, 14))); style(a); a.set_title(title, loc="left", fontsize=12)
    ax[0].text(-11, -33, f"radar height held at {t_r[2] * 100:+.0f} cm from the 10/21 solve: not measured.\n1σ: radar ±{sr[1] * 100:.0f} cm forward, ±{sr[0] * 100:.0f} cm sideways; camera ±{max(sc) * 100:.0f} cm", fontsize=10, color=RED)
    ax[1].text(-11, -11, f"arrows: boresight / optical axis. radar yaw {yaw - 90:+.1f}° from the LiDAR's forward axis", fontsize=10, color=MUTED)
    fig.suptitle("Where the sensors sit, from the 11/03 calibration (calib/extrinsics_joint.json), in the LiDAR's frame: forward, left, up. No tape measurements exist yet; these are solved, not measured", fontsize=12)
    fig.savefig(path, dpi=130); print("saved", path, {k: np.round(np.r_[fwd(v[0]), left(v[0]), v[0][2]] * 100, 1) for k, v in P.items()}, "camera axis", np.round(axis_c, 3))


def statue_power(t, centre_lidar_of, frames, tr, win_r=0.45, win_az=np.radians(8)):
    """Per radar frame: the strongest range-azimuth cell near the statue, in dB above that frame's median power."""
    from src.radar import detect, AZIMUTHS
    from src.radar.pointcloud import RANGE_OFFSET
    from src.calib.transform import RadarToLidar
    T = RadarToLidar.load_joint(trace=t); dres = json.load(open(f"{t}/radar/radar.json"))["range_resolution"]; rr = np.arange(256) * dres - RANGE_OFFSET; out = []
    for k in frames:
        c = T.inverse(centre_lidar_of(k)[None])[0]; r0, a0 = np.hypot(c[0], c[1]), np.arctan2(-c[0], c[1])
        if abs(a0) > np.radians(50): continue
        _, info = detect(tr.frame(k), tr.params); db = 10 * np.log10(info["power"] + 1e-12)
        m = (np.abs(rr - (r0 - 0.39))[None, :] < win_r) & (np.abs(AZIMUTHS - a0)[:, None] < win_az)
        if m.any(): out.append((tr.ts[k], r0, db[m].max() - np.median(db)))
    return np.array(out)


def frame_luma(t, times, video=None):
    import cv2
    cam = ts(t, "camera"); cap = cv2.VideoCapture(video or f"{t}/camera/video.avi"); out = []
    for tt in times:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(np.searchsorted(cam, tt))); ok, f = cap.read()
        if ok: out.append(float(np.median(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)[int(1080 * .55):])))
    return np.array(out)


def fig7(path="figures/s4_fig7_scotty_dusk_day.png"):
    from src.radar import RadarTrace
    from src.calib.lidar import LidarTrace
    # 11/03 dusk: poses of the orbit, pedestal at (-13.07, -9.86) in that world frame
    t3 = "data/raw/2026-11-03.17-01-12"; P = np.load(f"data/poses/2026-11-03.17-01-12/radar.npz"); tr3 = RadarTrace(t3); c3 = np.array([-13.07, -9.86, 0])
    Tk = {int(i): T for i, T in zip(P["index"], P["T"])}; cam0 = ts(t3, "camera")[0]
    f3 = [k for k in range(0, len(tr3), 3) if k in Tk and 30 <= tr3.ts[k] - cam0 <= 165]
    def c_lidar3(k):
        T = Tk[k]; p = np.r_[c3[:2], T[2, 3] - 0.3]; return T[:3, :3].T @ (p - T[:3, 3])
    s3 = statue_power(t3, c_lidar3, f3, tr3)
    # 11/05 day: static rig, pedestal found once in the LiDAR (circle of 0.39 m on the 0.45-1.0 m slice)
    t5 = D("16-34-29"); tr5 = RadarTrace(t5); lt = LidarTrace(t5); xyz = np.concatenate([lt.xyz(k) for k in range(100, 500, 20)]); g = np.percentile(xyz[:, 2], 3)
    sl = xyz[(xyz[:, 2] > g + .45) & (xyz[:, 2] < g + 1.0) & (np.hypot(xyz[:, 0], xyz[:, 1]) < 8)][:, :2]
    best = (-1, None)
    for cx in np.arange(-8, 0, .05):
        for cy in np.arange(-3, 3, .05):
            d = np.hypot(sl[:, 0] - cx, sl[:, 1] - cy); sc = ((d > .32) & (d < .46)).sum() - 3 * (d < .25).sum()
            if sc > best[0]: best = (sc, np.array([cx, cy]))
    c5 = np.r_[best[1], g + 0.7]; print("11/05 pedestal in the LiDAR frame", np.round(c5, 2), "range", round(float(np.hypot(*c5[:2])), 2))
    s5 = statue_power(t5, lambda k: c5, range(0, len(tr5), 3), tr5)
    # camera brightness of the lower image (no sky), same moments
    cam5 = ts(t5, "camera")[0]
    l3 = frame_luma(t3, s3[::15, 0]); l5 = frame_luma(t5, s5[::15, 0])
    fig, ax = plt.subplots(1, 2, figsize=(16.64, 5.8), constrained_layout=True, gridspec_kw=dict(width_ratios=[1.4, 1]))
    a = ax[0]
    a.scatter(s3[:, 1], s3[:, 2], s=10, color=ORANGE, alpha=.6, label=f"11/03 17:01-17:04, 14-11 min before sunset, walking ({len(s3)} frames)")
    a.scatter(s5[:, 1], s5[:, 2], s=10, color=BLUE, alpha=.6, label=f"11/05 16:34, about 38 min before sunset, rig static ({len(s5)} frames)")
    near = (s3[:, 1] > np.median(s5[:, 1]) - .3) & (s3[:, 1] < np.median(s5[:, 1]) + .3)
    a.axvspan(np.median(s5[:, 1]) - .3, np.median(s5[:, 1]) + .3, color=BLUE, alpha=.06)
    a.set_xlabel("range from the radar to the pedestal axis (m)"); a.set_ylabel("statue return: strongest cell near it,\ndB above the frame's median"); style(a); a.legend(fontsize=10, frameon=False, loc="lower left")
    a.set_title(f"a · radar return from the statue. At the same range ({np.median(s5[:, 1]):.1f} ± 0.3 m): dusk {np.median(s3[near, 2]):.1f} dB, day {np.median(s5[:, 2]):.1f} dB (medians)", loc="left", fontsize=11.5)
    b = ax[1]; b.boxplot([l3, l5], widths=.5, patch_artist=True, boxprops=dict(facecolor="#F4F6F9", color=MUTED), medianprops=dict(color=INK, lw=2))
    b.set_xticks([1, 2]); b.set_xticklabels(["11/03 dusk", "11/05 day"]); b.set_ylabel("camera: median luma of the lower image (0-255)"); style(b)
    b.set_title(f"b · camera brightness: {np.median(l3):.0f} vs {np.median(l5):.0f} (exposure may be automatic)", loc="left", fontsize=11.5)
    fig.suptitle("Scotty plaza at dusk (11/03) and by day (11/05): the image gets darker, the radar return from the statue stays at the same level within the spread of the walking orbit. Both before sunset", fontsize=12)
    fig.savefig(path, dpi=130)
    print("saved", path, f"dusk near-range median {np.median(s3[near, 2]):.1f} dB (n={near.sum()}), day {np.median(s5[:, 2]):.1f} dB, day range {np.median(s5[:, 1]):.2f}; luma {np.median(l3):.0f} vs {np.median(l5):.0f}")


if __name__ == "__main__":
    {"fig1": fig1, "fig2": fig2, "fig3": fig3, "fig4": fig4, "fig7": fig7}[sys.argv[1]]()
