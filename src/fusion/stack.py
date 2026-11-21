"""Multi-height stacking: radar seeds from several takes at one mark, each at its own height, in one image.

    python -m src.fusion.stack --ref data/raw/2026-11-19.17-40-44 --takes data/raw/2026-11-19.17-* \
        --time 20 --object -2.65 0.08 0.9 --out out/stack_1119.png

One take puts its seeds on one image row (the radar has no elevation, so a seed sits at the radar's
height). Several takes at different heights put them on several rows. The rig is lifted by hand
between takes and moves up to half a metre, so each take is first registered to the reference take
with its LiDAR scan (point-to-plane ICP); the LiDAR is used for that and for the score, never for
the depth itself. A seed of take k then goes radar_k -> LiDAR_k -> LiDAR_ref -> camera_ref.

The same five fills as `dense_depth` run on the stacked seeds; nearest column and the least-squares
fill are anchored on every take's own row instead of on one row.
"""
import argparse, glob, json, os

import numpy as np
from scipy.spatial import cKDTree

from ..calib.lidar import LidarTrace
from ..calib.board_image import project_fisheye
from ..calib.transform import joint_path, RadarToLidar
from ..recon.accumulate import height_above_ground
from ..eval.lidar_depth import lidar_depth_image, errors
from ..eval.silhouette import silhouette, outline, classes, edge_accuracy, close_up, around
from ..radar import RadarTrace
from ..radar.angle import AZIMUTHS
from ..radar.cfar import ca_cfar_range
from ..radar.pointcloud import spectra, RANGE_OFFSET
from ..recon.closeup import camera_frame
from .seeds import static_cells, drop_sidelobes, first_returns
from .jbu import upsample
from .wls import fill as wls_fill
from .dense_depth import guide_image, SHOWN, DISTINCT, SIMILAR


# ---------------------------------------------------------------- registration

def lidar_cloud(trace, t_unix, scans=5, rfl=False):
    """A few consecutive LiDAR scans merged (the rig is still), LiDAR frame; with rfl, also each point's reflectivity."""
    lt = LidarTrace(trace); k = lt.index_at(t_unix); ks = range(max(k - scans // 2, 0), min(k + scans // 2 + 1, len(lt)))
    if not rfl:
        return np.concatenate([lt.xyz(kk) for kk in ks])
    p = [lt.xyz_rfl(kk) for kk in ks]
    return np.concatenate([a for a, _ in p]), np.concatenate([b for _, b in p]).astype(float)


def ground_plane(xyz, reach=8.0, tol=0.04, trials=300, seed=0):
    """RANSAC ground plane among the low points: unit normal n (pointing up) and offset c with n.x + c = 0.

    The LiDAR's height above the ground is then c (its origin's distance to the plane)."""
    rng = np.random.default_rng(seed); rho = np.linalg.norm(xyz[:, :2], axis=1)
    P = xyz[(xyz[:, 2] < -0.6) & (rho < reach) & (rho > 0.8)]; best = (0, None)
    for _ in range(trials):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]; n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n); n = n if n[2] > 0 else -n
        if n[2] < 0.8:
            continue
        k = int((np.abs((P - a) @ n) < tol).sum())
        if k > best[0]:
            best = (k, (n, -a @ n))
    n, c = best[1]; inl = np.abs(P @ n + c) < tol                       # refit on the inliers
    Q = P[inl] - P[inl].mean(0); n = np.linalg.svd(Q, full_matrices=False)[2][-1]; n = n if n[2] > 0 else -n
    return n, float(-P[inl].mean(0) @ n)


def _align(a, b):
    """Rotation taking unit vector a onto b."""
    v = np.cross(a, b); c = a @ b; K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def initial_guess(src, dst):
    """Level both clouds on their ground planes and put the source at its own height above the reference ground."""
    (ns, cs), (nd, cd) = ground_plane(src), ground_plane(dst)
    T = np.eye(4); T[:3, :3] = _align(ns, nd); T[:3, 3] = nd * (cs - cd)     # origin of src sits cs above dst's ground
    return T, cs, cd


def voxel(xyz, size):
    k = np.floor(xyz / size).astype(np.int64); _, i = np.unique(k, axis=0, return_index=True)
    return xyz[i]


def normals(xyz, k=12):
    _, nb = cKDTree(xyz).query(xyz, k)
    P = xyz[nb] - xyz[nb].mean(axis=1, keepdims=True)
    _, _, vt = np.linalg.svd(P, full_matrices=False)
    return vt[:, -1]


def _se3(x):
    """Small rotation (rx, ry, rz) and translation -> 4x4."""
    th = np.linalg.norm(x[:3]); K = np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])
    R = np.eye(3) + (np.sin(th) / th * K + (1 - np.cos(th)) / th**2 * K @ K if th > 1e-12 else K)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = x[3:]; return T


def icp(src, dst, T0=np.eye(4), dists=(1.0, 0.5, 0.25, 0.12), iters=15, reach=15.0, vox=0.08):
    """Point-to-plane ICP: the 4x4 T with dst ~ T src. Returns T and the RMS point-to-plane residual of the last pass (m)."""
    src = voxel(src[np.linalg.norm(src[:, :2], axis=1) < reach], vox); dst = voxel(dst[np.linalg.norm(dst[:, :2], axis=1) < reach], vox)
    nd = normals(dst); tree = cKDTree(dst); T = T0.copy(); rms = np.nan
    for md in dists:
        for _ in range(iters):
            p = src @ T[:3, :3].T + T[:3, 3]; d, j = tree.query(p, distance_upper_bound=md); ok = np.isfinite(d)
            p, q, n = p[ok], dst[j[ok]], nd[j[ok]]
            r = ((p - q) * n).sum(1); A = np.c_[np.cross(p, n), n]
            x = np.linalg.lstsq(A, -r, rcond=None)[0]; T = _se3(x) @ T
            rms = float(np.sqrt(np.mean(r ** 2))); inl = float(ok.mean())
            if np.linalg.norm(x) < 1e-5:
                break
    return T, rms, inl


# ---------------------------------------------------------------- seeds of one take, in the reference camera

def take_seeds(trace, start, seconds):
    """Static first-return seeds of one take, radar frame (z = 0), with their attributes."""
    ts0 = np.fromfile(os.path.join(trace, "camera", "ts"), "<f8")[0]
    cells, n = static_cells(trace, ts0 + start, seconds); kept, _ = drop_sidelobes(cells); s = first_returns(kept)
    xyz = np.stack([-s["r"] * np.sin(s["az"]), s["r"] * np.cos(s["az"]), np.zeros(len(s))], axis=-1)
    return s, xyz, n


def lidar_to_camera(trace):
    path = joint_path(trace); j = json.load(open(path)); lc = j["lidar_to_camera"]
    cam = json.load(open(os.path.join(os.path.dirname(path), os.path.basename(j["radar_to_camera"]["intrinsics"]))))
    return np.array(lc["R"]), np.array(lc["t_m"]), cam


def seeds_in_ref(xyz_radar, trace_k, T_k_ref, trace_ref):
    """Radar_k points -> LiDAR_ref (via T) -> reference camera: pixels (N, 2) at full size, range from the camera, LiDAR_ref xyz, kept mask."""
    xl = RadarToLidar.load_joint(trace=trace_k)(xyz_radar)
    xr = xl @ T_k_ref[:3, :3].T + T_k_ref[:3, 3]
    R, t, cam = lidar_to_camera(trace_ref); xc = xr @ R.T + t
    uv = project_fisheye(xc, cam)
    ok = (xc[:, 2] > 0.1) & (uv[:, 0] >= 0) & (uv[:, 0] < cam["w"]) & (uv[:, 1] >= 0) & (uv[:, 1] < cam["h"])
    return uv, np.linalg.norm(xc, axis=1), xr, ok


# ---------------------------------------------------------------- fills from several rows

def take_rows(uvs, d, w, take, W):
    """Per take, its seed row across every column: row v(u), the depth of the column's nearest seed, that seed's weight."""
    u = np.arange(W, dtype=float); V, D, Wt = [], [], []
    for t in np.unique(take):
        m = take == t; o = np.argsort(uvs[m, 0]); us, vs = uvs[m][o, 0], uvs[m][o, 1]
        j = np.abs(u[:, None] - us[None, :]).argmin(axis=1)
        V.append(np.interp(u, us, vs)); D.append(d[m][o][j]); Wt.append(w[m][o][j])
    return np.array(V), np.array(D), np.array(Wt)


def fills(lab, uvs, d, w, take, sx, sy, sigma_c=0.08, ls_sigma=0.05, ls_lambda=1.0):
    """The dense_depth fills on seeds from one or several takes; with one take they are exactly dense_depth's."""
    H, W = lab.shape[:2]; V, D, Wt = take_rows(uvs, d, w, take, W)
    t = np.abs(np.arange(H, dtype=float)[:, None, None] - V[None]).argmin(axis=1)       # per pixel, the take whose row passes nearest
    out = {"nearest column": D[t, np.arange(W)[None, :]]}
    out["Gaussian"], _ = upsample(lab, uvs, d, sx, sy, None, w)
    out["joint bilateral"], _ = upsample(lab, uvs, d, sx, sy, sigma_c, w)
    cols = np.tile(np.arange(W, dtype=float), len(V))
    out["edge-aware least squares"], _ = wls_fill(lab, np.stack([cols, V.ravel()], axis=1), D.ravel(), ls_sigma, ls_lambda, radius=0, weight=Wt.ravel())
    out["constant (median seed)"] = np.full((H, W), float(np.median(d)))
    return out


def score(F, ref, height, xyimg, lab, obj, split=1.05):
    """dense_depth's numbers: MAE above ground, on and around the object, its two parts, and the outline kept."""
    above = height > 0.3
    o = (np.hypot(xyimg[..., 0] - obj[0], xyimg[..., 1] - obj[1]) < obj[2]) & above
    sil = silhouette(o); masks = {"above_ground": above, "object": o, "around_object": around(sil) & above,
                                  "object, lower part": o & (height < split), "object, upper part": o & (height >= split)}
    ol = outline(sil, lab, ref); cls = classes(ol); res = {}
    for n, D in F.items():
        res[n] = {k: errors(D, ref, m)["0-20 m"] for k, m in masks.items()}; res[n]["edge_accuracy"] = edge_accuracy(D, sil, ol, cls)
    res["outline_px"] = {k: int(v.sum()) for k, v in cls.items()}
    return res, sil, cls


# ---------------------------------------------------------------- bronze and stone, per height

def mean_power(trace, t0, seconds, stride=6):
    """Range-azimuth power summed over Doppler, averaged over the take (the rig is still)."""
    rt = RadarTrace(trace); k0 = rt.index_at(t0); k1 = min(rt.index_at(t0 + seconds), len(rt.ts) - 1); P = []
    for k in range(k0, k1, stride):
        if rt.valid[k]:
            _, sa, _ = spectra(rt.frame(k)); P.append((np.abs(sa) ** 2).sum(axis=0))
    return np.mean(P, axis=0), rt.params.ranges - RANGE_OFFSET


def to_radar(x_ref, trace_k, T_k_ref):
    """LiDAR_ref points -> take k's radar frame."""
    Ti = np.linalg.inv(T_k_ref); xl = x_ref @ Ti[:3, :3].T + Ti[:3, 3]
    return RadarToLidar.load_joint(trace=trace_k).inverse(xl)


def statue_per_take(trace, T, ref_trace, P, rng, statue, part, centre, plane, beam=np.radians(7.0)):
    """What one take's radar sees of the object: beam-centre height at it, the power profile along its azimuth, the CFAR margin,
    and the LiDAR's range (from this radar) to each part inside the azimuth beam."""
    nd, cd = plane
    g = np.mgrid[-4:4:0.02, 0.5:5:0.02].reshape(2, -1).T; X = np.c_[g, np.zeros(len(g))]
    _, _, xr, _ = seeds_in_ref(X, trace, T, ref_trace); k = np.argmin(np.hypot(*(xr[:, :2] - centre).T))
    az = float(np.arctan2(-X[k, 0], X[k, 1])); beam_h = float(xr[k] @ nd + cd)
    ia = int(np.argmin(np.abs(AZIMUTHS - az))); thr, _ = ca_cfar_range(P)
    floor = float(np.median(10 * np.log10(P[:, (rng > 1) & (rng < 6)])))
    xs = to_radar(statue, trace, T); a = np.arctan2(-xs[:, 0], xs[:, 1]); r = np.linalg.norm(xs, axis=1); inb = np.abs(a - az) < beam
    parts = {}
    for name, m in [("stone", part == 0), ("bronze", part == 1)]:
        rr = r[inb & m]; parts[name] = dict(front=float(np.percentile(rr, 1)), median=float(np.median(rr)), back=float(np.percentile(rr, 95)))
    w = (rng > parts["stone"]["front"] - 0.1) & (rng < parts["bronze"]["back"] + 0.1); j = np.flatnonzero(w)[np.argmax(P[ia, w])]
    return dict(az_deg=float(np.degrees(az)), beam_height_m=beam_h, floor_db=floor, profile_db=(10 * np.log10(P[ia]) - floor).tolist(),
                peak_range_m=float(rng[j]), peak_db=float(10 * np.log10(P[ia, j]) - floor), cfar_margin_db=float(10 * np.log10(P[ia, j] / thr[ia, j])),
                threshold_db=float(10 * np.log10(thr[ia, j]) - floor), lidar=parts)


# ---------------------------------------------------------------- figures

def _cmap_h(hs):
    import matplotlib
    return matplotlib.colormaps["viridis"], min(hs) - 0.05, max(hs) + 0.05


def plot_stack(out, small, ref, seeds1, seedsN, F1, FN, r1, rN, sil, cls, statue_uv, vmin, vmax):
    """Rows: one height / all heights, depth then the object close-up. Columns: the input, then one per fill."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from scipy import ndimage as ndi
    H, W = ref.shape; names = SHOWN; nc = len(names) + 1
    gain = float(np.clip(0.85 / max(np.percentile(small, 95), 1e-3), 1.0, 4.0)); show = np.clip(small * gain, 0, 1)
    dist, (iy, ix) = ndi.distance_transform_edt(~np.isfinite(ref), return_indices=True); lidar = np.where(dist <= 3, ref[iy, ix], np.nan)
    cmap = matplotlib.colormaps["gray_r"].copy(); cmap.set_bad("#bcd3e8"); gkw = dict(cmap=cmap, vmin=vmin, vmax=vmax)
    y0, y1, x0, x1 = close_up(sil); hc = (y1 - y0) / (x1 - x0)
    hmap, hlo, hhi = _cmap_h(seedsN["h"])
    fig, ax = plt.subplots(4, nc, figsize=(2.75 * nc + 0.6, 2.75 * (2 * H / W + 2 * hc) + 1.4),
                           gridspec_kw=dict(height_ratios=[H / W, H / W, hc, hc]), constrained_layout=True)
    lab = dict(fontsize=8.5, labelpad=3)

    def dots(a):
        for c, col in [("distinct", DISTINCT), ("similar", SIMILAR)]:
            oy, ox = np.nonzero(cls[c]); a.scatter(ox - x0, oy - y0, s=4, c=col, marker="s", linewidths=0)

    for i, (S, n) in enumerate([(seeds1, "one height"), (seedsN, f"{len(set(seedsN['take']))} heights")]):
        a = ax[i, 0]; a.imshow(show, aspect="auto")
        a.scatter(S["uv"][:, 0], S["uv"][:, 1], s=22, c=S["h"], cmap=hmap, vmin=hlo, vmax=hhi, edgecolors="white", linewidths=.6)
        a.set_xlabel(f"{len(S['d'])} radar seeds", **lab)
    a = ax[2, 0]; a.imshow(lidar[y0:y1, x0:x1], **gkw); dots(a); a.set_xlabel("LiDAR", **lab)
    a = ax[3, 0]; a.imshow(show[y0:y1, x0:x1], aspect="auto")
    sc = a.scatter(seedsN["uv"][:, 0] - x0, seedsN["uv"][:, 1] - y0, s=30, c=seedsN["h"], cmap=hmap, vmin=hlo, vmax=hhi, edgecolors="white", linewidths=.6)
    a.set_xlim(0, x1 - x0); a.set_ylim(y1 - y0, 0); a.set_xlabel("camera, seeds of every height", **lab)
    for j, n in enumerate(names, start=1):
        ax[0, j].set_title(n, fontsize=10, fontweight="bold")
        for i, (F, r) in enumerate([(F1, r1), (FN, rN)]):
            im = ax[i, j].imshow(F[n], **gkw); ax[i, j].set_xlabel(f"MAE: {r[n]['above_ground']['mae']:.2f} m", **lab)
            a = ax[2 + i, j]; a.imshow(F[n][y0:y1, x0:x1], **gkw); dots(a)
            a.set_xlabel(f"MAE (object): {r[n]['object']['mae']:.2f} m\nMAE (around): {r[n]['around_object']['mae']:.2f} m", **lab)
    ax[0, 0].set_title("camera and radar seeds", fontsize=10, fontweight="bold")
    for i, t in enumerate(["one height: depth", "all heights: depth", "one height: close-up", "all heights: close-up"]):
        ax[i, 0].set_ylabel(t, fontsize=10, fontweight="bold")
    for a in ax.ravel():
        a.set_xticks([]); a.set_yticks([])
    fig.colorbar(im, ax=ax[:2, -1], location="right", shrink=.9, aspect=20, label="range (m), near is white")
    fig.colorbar(sc, ax=ax[2:, -1], location="right", shrink=.9, aspect=20, label="radar height at the statue (m)")
    fig.savefig(out, dpi=150); print("saved", out)


def plot_stack3d(out, trace, small, seeds1, seedsN, F1, FN, r1, rN, yaw=15.0, pitch=25.0):
    """Like the top row of the dense-depth figure: each fill in 3D in camera colours, seen from above and to the side.
    Rows: one height / all heights. Columns: the camera with its seeds, then one per fill."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from .view3d import intrinsics, rays, render
    H, W = small.shape[:2]; names = SHOWN; nc = len(names) + 1
    gain = float(np.clip(0.85 / max(np.percentile(small, 95), 1e-3), 1.0, 4.0)); show = np.clip(small * gain, 0, 1)
    R = rays(intrinsics(trace), H, W); hmap, hlo, hhi = _cmap_h(seedsN["h"])
    fig, ax = plt.subplots(2, nc, figsize=(2.75 * nc + 0.6, 2 * 2.75 * 270 / 480 + 1.3), constrained_layout=True)
    lab = dict(fontsize=8.5, labelpad=3)
    for i, (S, F, r) in enumerate([(seeds1, F1, r1), (seedsN, FN, rN)]):
        a = ax[i, 0]; a.imshow(show, aspect="auto")
        sc = a.scatter(S["uv"][:, 0], S["uv"][:, 1], s=22, c=S["h"], cmap=hmap, vmin=hlo, vmax=hhi, edgecolors="white", linewidths=.6)
        a.set_xlabel(f"{len(S['d'])} radar seeds", **lab)
        for j, n in enumerate(names, start=1):
            a = ax[i, j]; a.imshow(render(R, F[n], show, yaw=yaw, pitch=pitch), aspect="auto")
            a.set_xlabel(f"MAE: {r[n]['above_ground']['mae']:.2f} m", **lab)
            if i == 0:
                a.set_title(n, fontsize=10, fontweight="bold")
    ax[0, 0].set_title("camera and radar seeds", fontsize=10, fontweight="bold")
    ax[0, 0].set_ylabel("one height", fontsize=10, fontweight="bold")
    ax[1, 0].set_ylabel(f"{len(set(seedsN['take'].tolist()))} heights", fontsize=10, fontweight="bold")
    for a in ax.ravel():
        a.set_xticks([]); a.set_yticks([])
    fig.colorbar(sc, ax=ax[:, -1], location="right", shrink=.9, aspect=20, label="radar height at the statue (m)")
    fig.savefig(out, dpi=150); print("saved", out)


def plot_materials(out, statue, part, rfl, plane, per, rng, centre):
    """Side view of the object by part with each take's beam height; radar power along range; LiDAR reflectivity by part."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    nd, cd = plane; h = statue @ nd + cd
    STONE, BRONZE = "#8c8c8c", "#b5651d"
    hs = [p["beam_height_m"] for p in per.values()]; hmap, hlo, hhi = _cmap_h(hs)
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.6), gridspec_kw=dict(width_ratios=[0.8, 1.6, 1.0]), constrained_layout=True)
    # side view: across the line of sight against height
    los = np.r_[centre, 0.0]; los /= np.linalg.norm(los); side = np.cross([0, 0, 1.0], los)
    a = ax[0]; s = statue @ side
    for m, c, n in [(part == 0, STONE, "stone"), (part == 1, BRONZE, "bronze")]:
        a.scatter(s[m], h[m], s=0.6, c=c, linewidths=0, rasterized=True, label=n)
    for name, p in per.items():
        c = hmap((p["beam_height_m"] - hlo) / (hhi - hlo)); a.axhline(p["beam_height_m"], color=c, lw=1.4, ls="-" if p["seed"] else ":")
    a.set_aspect("equal"); a.set_xlabel("across (m)"); a.set_ylabel("height above ground (m)"); a.set_title("statue and radar height", fontsize=10, fontweight="bold")
    a.legend(loc="lower right", fontsize=7, markerscale=8, frameon=False)
    # power along range
    a = ax[1]; st = np.mean([p["lidar"]["stone"]["front"] for p in per.values()]), np.mean([p["lidar"]["stone"]["median"] for p in per.values()])
    br = np.mean([p["lidar"]["bronze"]["front"] for p in per.values()]), np.mean([p["lidar"]["bronze"]["median"] for p in per.values()])
    a.axvspan(*st, color=STONE, alpha=.25, lw=0); a.axvspan(*br, color=BRONZE, alpha=.25, lw=0)
    a.text(np.mean(st), -10.5, "stone", ha="center", fontsize=8, color="0.3"); a.text(np.mean(br), -10.5, "bronze", ha="center", fontsize=8, color=BRONZE)
    w = (rng > 1.6) & (rng < 3.8)
    for name, p in sorted(per.items(), key=lambda kv: kv[1]["beam_height_m"]):
        c = hmap((p["beam_height_m"] - hlo) / (hhi - hlo))
        a.plot(rng[w], np.array(p["profile_db"])[w], color=c, lw=1.3, ls="-" if p["seed"] else ":", label=f"{p['beam_height_m']:.2f} m" + ("" if p["seed"] else ", no seed"))
    a.set_xlabel("range from the radar (m)"); a.set_ylabel("power above the floor (dB)"); a.set_ylim(-12, 40)
    a.set_title("radar power along the statue's azimuth", fontsize=10, fontweight="bold")
    a.legend(fontsize=6.5, frameon=False, loc="upper left", bbox_to_anchor=(1.0, 1.0), title="radar height\nat the statue", title_fontsize=7)
    # reflectivity
    a = ax[2]; b = np.arange(0, 80, 2)
    for m, c, n in [(part == 0, STONE, "stone"), (part == 1, BRONZE, "bronze")]:
        a.hist(rfl[m], bins=b, color=c, alpha=.6, density=True, label=f"{n}, median {np.median(rfl[m]):.0f}")
    a.set_xlabel("LiDAR reflectivity (0-255)"); a.set_yticks([]); a.legend(fontsize=7.5, frameon=False)
    a.set_title("LiDAR reflectivity", fontsize=10, fontweight="bold")
    fig.savefig(out, dpi=150); print("saved", out)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="the reference take: its camera frame, its LiDAR (score) and frame")
    ap.add_argument("--takes", nargs="+", required=True, help="every take of the stack (the reference included)")
    ap.add_argument("--time", type=float, default=20.0, help="camera frame and LiDAR scan of every take (s from its first frame)")
    ap.add_argument("--start", type=float, default=3.0); ap.add_argument("--seconds", type=float, default=36.0)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--object", type=float, nargs=3, required=True, metavar=("X", "Y", "R"), help="the object in the reference LiDAR frame")
    ap.add_argument("--split", type=float, default=1.05, help="height (m) between the stone pedestal and the bronze figure")
    ap.add_argument("--vmin", type=float, default=2.0); ap.add_argument("--vmax", type=float, default=10.0)
    ap.add_argument("--hide", type=int, nargs=4, action="append", default=[], metavar=("X0", "Y0", "X1", "Y1"),
                    help="blur this box (full-frame px) in the shown frame only, e.g. a person behind a window; the fills use the frame as recorded")
    ap.add_argument("--out", required=True, help="stack figure; the materials figure and the JSON go next to it")
    a = ap.parse_args()
    ref_tr = a.ref.rstrip("/"); takes = sorted(t.rstrip("/") for t in a.takes)
    ts0 = lambda t: np.fromfile(os.path.join(t, "camera", "ts"), "<f8")[0]
    ref_cloud, ref_rfl = lidar_cloud(ref_tr, ts0(ref_tr) + a.time, rfl=True); plane = ground_plane(ref_cloud)
    centre = np.array(a.object[:2])
    hgt = ref_cloud @ plane[0] + plane[1]; on = (np.hypot(*(ref_cloud[:, :2] - centre).T) < a.object[2]) & (hgt > 0.05)
    statue, rfl = ref_cloud[on], ref_rfl[on]; part = (hgt[on] >= a.split).astype(int)

    S = dict(uv=[], d=[], w=[], h=[], take=[]); per = {}
    for i, tr in enumerate(takes):
        name = os.path.basename(tr); c = lidar_cloud(tr, ts0(tr) + a.time); T0, h_own, _ = initial_guess(c, ref_cloud)
        T, rms, inl = icp(c, ref_cloud, T0)
        sd, xyz, n = take_seeds(tr, a.start, a.seconds); uv, d, xr, ok = seeds_in_ref(xyz, tr, T, ref_tr)
        P, rng = mean_power(tr, ts0(tr) + a.start, a.seconds)
        p = statue_per_take(tr, T, ref_tr, P, rng, statue, part, centre, plane)
        near = ok & (np.hypot(*(xr[:, :2] - centre).T) < a.object[2])
        p.update(lidar_height_m=float(T[:3, 3] @ plane[0] + plane[1]), icp_rms_cm=100 * rms, icp_inliers=inl,
                 shift_m=T[:3, 3].round(3).tolist(), yaw_deg=float(np.degrees(np.arctan2(T[1, 0], T[0, 0]))),
                 tilt_deg=float(np.degrees(np.arccos(np.clip(T[:3, 2] @ plane[0], -1, 1)))), seeds=int(ok.sum()), frames=n,
                 seed=bool(near.any()), seed_range_m=[float(np.linalg.norm(x)) for x in xyz[near]], seed_snr_db=sd["snr"][near].tolist())
        per[name] = p
        print(f"{name}: LiDAR {p['lidar_height_m']:.2f} m, radar at the statue {p['beam_height_m']:.2f} m, ICP {p['icp_rms_cm']:.1f} cm, "
              f"{ok.sum()} seeds, statue seed {p['seed']}, peak {p['peak_db']:.0f} dB at {p['peak_range_m']:.2f} m, CFAR margin {p['cfar_margin_db']:+.1f} dB")
        S["uv"].append(uv[ok]); S["d"].append(d[ok]); S["w"].append(sd["share"][ok].astype(float))
        S["h"].append(np.full(ok.sum(), p["beam_height_m"])); S["take"].append(np.full(ok.sum(), i))
    S = {k: np.concatenate(v) for k, v in S.items()}
    iref = takes.index(ref_tr); one = {k: v[S["take"] == iref] for k, v in S.items()}

    rgb, _ = camera_frame(ref_tr, ts0(ref_tr) + a.time, a.out + "._frame.png"); small, lab = guide_image(rgb, a.scale)
    H, W = lab.shape[:2]; sx, sy = 100 * a.scale, 320 * a.scale
    ref, height, xyimg = lidar_depth_image(ref_tr, ts0(ref_tr) + a.time, a.scale, return_xy=True)
    sc = lambda Q: {**Q, "uv": Q["uv"] * a.scale}
    one, S = sc(one), sc(S)
    F1 = fills(lab, one["uv"], one["d"], one["w"], one["take"], sx, sy); FN = fills(lab, S["uv"], S["d"], S["w"], S["take"], sx, sy)
    r1, sil, cls = score(F1, ref, height, xyimg, lab, a.object, a.split); rN, _, _ = score(FN, ref, height, xyimg, lab, a.object, a.split)
    alone = {}                                                           # every take on its own, in the same image: is the reference take typical?
    for i, tr in enumerate(takes):
        m = S["take"] == i; Fi = fills(lab, S["uv"][m], S["d"][m], S["w"][m], S["take"][m], sx, sy)
        alone[os.path.basename(tr)] = score(Fi, ref, height, xyimg, lab, a.object, a.split)[0]
    print(f"{'stack against each take alone':34s}{'above ground':>22s}{'object':>22s}")
    for n in SHOWN:
        q = lambda k: np.array([alone[t][n][k]["mae"] for t in alone])
        print(f"{n:34s}" + "".join(f"{rN[n][k]['mae']:6.2f} vs {np.median(q(k)):.2f} ({q(k).min():.2f}-{q(k).max():.2f})" for k in ["above_ground", "object"]))
    print(f"{'MAE (m)':26s}{'above ground':>14s}{'object':>9s}{'around':>9s}{'stone':>9s}{'bronze':>9s}")
    for tag, r in [("one", r1), ("all", rN)]:
        for n in F1:
            x = r[n]; print(f"{tag + ', ' + n:26s}{x['above_ground']['mae']:14.2f}{x['object']['mae']:9.2f}{x['around_object']['mae']:9.2f}"
                            f"{x['object, lower part']['mae']:9.2f}{x['object, upper part']['mae']:9.2f}")
    base = os.path.splitext(a.out)[0]
    json.dump(dict(per_take=per, one_height=r1, all_heights=rN, each_take_alone=alone, ranges_m=rng.tolist(),
                   lidar_rfl=dict(stone=float(np.median(rfl[part == 0])), bronze=float(np.median(rfl[part == 1]))),
                   params=dict(ref=os.path.basename(ref_tr), takes=[os.path.basename(t) for t in takes], time=a.time, start=a.start,
                               seconds=a.seconds, scale=a.scale, object=a.object, split=a.split)), open(base + ".json", "w"), indent=1)
    shown = small.copy()
    if a.hide:
        import cv2
        blur = cv2.GaussianBlur(shown, (0, 0), 6)
        for x0, y0, x1, y1 in a.hide:
            sl = np.s_[int(y0 * a.scale):int(np.ceil(y1 * a.scale)), int(x0 * a.scale):int(np.ceil(x1 * a.scale))]; shown[sl] = blur[sl]
    plot_stack(a.out, shown, ref, one, S, F1, FN, r1, rN, sil, cls, None, a.vmin, a.vmax)
    plot_stack3d(base + "_3d.png", ref_tr, shown, one, S, F1, FN, r1, rN)
    plot_materials(base + "_materials.png", statue, part, rfl, plane, {k: v for k, v in per.items()}, rng, centre)


if __name__ == "__main__":
    main()
