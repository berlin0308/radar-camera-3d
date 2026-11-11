"""Dense depth of a static scene from radar seeds and one camera frame, scored against the LiDAR.

    python -m src.fusion.dense_depth data/raw/2026-11-05.16-10-35 --time 12 --start 5 --seconds 50 --out out/dense_static_b.png

Three fills of the same seeds, so the camera's share can be read off: the nearest seed per column
(no image), a Gaussian fill (image position only), and the joint bilateral fill (position and
colour). The LiDAR is the reference and is not used by any of them. Times are seconds from the
first camera frame. Writes the figure and, next to it, a JSON with the numbers.
"""
import argparse, json, os

import numpy as np

from .seeds import static_cells, drop_sidelobes, first_returns, to_image
from .jbu import upsample, nearest_column
from .wls import fill as wls_fill
from ..eval.lidar_depth import lidar_depth_image, errors, edge_band
from ..eval.silhouette import silhouette, outline, classes, edge_accuracy
from ..recon.closeup import camera_frame

DMAX = 20.0


def guide_image(rgb, scale):
    """Camera frame -> small Lab image scaled to about [0, 1], lightly blurred."""
    import cv2
    small = cv2.resize(rgb[..., :3].astype(np.float32), None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(cv2.GaussianBlur(small, (0, 0), 1.0), cv2.COLOR_RGB2LAB) / 100.0
    return small, lab


def seed_minus_lidar(uvs, d, ref):
    """Each seed against the LiDAR at its own pixel (median of a small patch); NaN where the LiDAR has nothing."""
    out = []
    for (u, v), dd in zip(uvs.astype(int), d):
        p = ref[max(v - 4, 0):v + 5, max(u - 3, 0):u + 4]; p = p[np.isfinite(p)]
        out.append(float(dd - np.median(p)) if len(p) else float("nan"))
    return np.array(out)


SHOWN = ["nearest column", "Gaussian", "joint bilateral", "edge-aware least squares"]   # the constant fill is in the JSON, not drawn
ROW = {"nearest column": "nearest column\n(no image)", "Gaussian": "Gaussian\n(image position)",
       "joint bilateral": "joint bilateral\n(position + colour)", "edge-aware least squares": "least squares\n(colour edges)"}
DISTINCT, SIMILAR = "#ff7f0e", "#1f9fff"          # outline colours, on grey depth


def plot(out, small, uvs, d, fills, ref, above, res, obj=None, sil=None, ol=None, cls=None, vmin=0.0, vmax=DMAX):
    """One row per fill, one column per view (depth, error, object close-up); the numbers under each panel."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from scipy import ndimage as ndi
    H, W = ref.shape; names = [n for n in SHOWN if n in fills]
    gain = float(np.clip(0.85 / max(np.percentile(small, 95), 1e-3), 1.0, 2.5)); show = np.clip(small * gain, 0, 1)
    box = None
    if sil is not None:
        sl = ndi.find_objects(sil.astype(int))[0]; m = 12
        box = (max(sl[0].start - m, 0), min(sl[0].stop + 4, H), max(sl[1].start - m, 0), min(sl[1].stop + m, W))
    ncol = 3 if box else 2; ratios = [W / H] * 2 + ([(box[3] - box[2]) / (box[1] - box[0])] if box else [])
    fig, ax = plt.subplots(len(names) + 1, ncol, figsize=(1.85 * sum(ratios) + 0.8, 2.15 * (len(names) + 1) + 0.9),
                           gridspec_kw=dict(width_ratios=ratios), constrained_layout=True)
    gkw = dict(cmap="gray_r", vmin=vmin, vmax=vmax); ekw = dict(cmap="magma", vmin=0, vmax=5)
    lab = dict(fontsize=8.5, labelpad=3); m20 = lambda r: r["0-20 m"]
    yy, xx = np.nonzero(np.isfinite(ref) & above)

    def outline_dots(a, y0=0, x0=0, s=4):
        for c, col in [("distinct", DISTINCT), ("similar", SIMILAR)]:
            oy, ox = np.nonzero(cls[c]); a.scatter(ox - x0, oy - y0, s=s, c=col, marker="s", linewidths=0)

    a = ax[0, 0]; a.imshow(show)
    a.scatter(uvs[:, 0], uvs[:, 1], s=46, c=d, edgecolors="#ff3b30", linewidths=1.4, **gkw)
    a.set_title("camera and radar seeds", fontsize=9.5); a.set_xlabel(f"{len(d)} seeds (red rings), shaded by range", **lab)
    a = ax[0, 1]; a.imshow(show * 0.45)
    if obj is not None:
        oy, ox = np.nonzero(obj); a.scatter(ox, oy, s=1.2, c="#ffcc00", marker="s", linewidths=0)
        a.set_xlim(0, W); a.set_ylim(H, 0); a.set_title("object, from the LiDAR", fontsize=9.5)
        a.set_xlabel(f"{int(obj.sum())} pixels, scored on their own", **lab)
    else:
        a.set_title("camera, dimmed", fontsize=9.5)
    if box:
        y0, y1, x0, x1 = box; a = ax[0, 2]; a.imshow(show[y0:y1, x0:x1]); outline_dots(a, y0, x0)
        o = res["outline"]
        a.set_title("object outline", fontsize=9.5)
        a.set_xlabel("orange: distinct\nblue: similar to background", **lab)
    ax[0, 0].set_ylabel("input", fontsize=10)

    for i, n in enumerate(names, start=1):
        r = res[n]; D = fills[n]
        a = ax[i, 0]; im = a.imshow(D, **gkw)
        a.set_xlabel(f"MAE {m20(r['above_ground'])['mae']:.2f} m · within 1.25×: {m20(r['above_ground'])['d125']:.0%}", **lab)
        a.set_ylabel(ROW.get(n, n), fontsize=10)
        a = ax[i, 1]; a.imshow(show * 0.3)
        ie = a.scatter(xx, yy, s=1.2, c=np.abs(D - ref)[yy, xx], marker="s", linewidths=0, **ekw); a.set_xlim(0, W); a.set_ylim(H, 0)
        a.set_xlabel(f"near depth edges {m20(r['edge_band'])['mae']:.2f} m · elsewhere {m20(r['off_band'])['mae']:.2f} m", **lab)
        if box:
            a = ax[i, 2]; a.imshow(D[y0:y1, x0:x1], **gkw); outline_dots(a, y0, x0)
            ea = r["edge_accuracy"]
            a.set_xlabel(f"object MAE {m20(r['object'])['mae']:.2f} m\ndistinct outline kept {ea['distinct']['accuracy']:.0%}\nsimilar outline kept {ea['similar']['accuracy']:.0%}", **lab)
    heads = ["depth", "error against the LiDAR", "object close-up"]
    for j in range(ncol):
        ax[1, j].set_title(heads[j], fontsize=10, fontweight="bold")
    for a in ax.ravel():
        a.set_xticks([]); a.set_yticks([])
    fig.colorbar(im, ax=ax[1:, 0], location="bottom", shrink=.8, aspect=30, label="range from the camera (m), near is white")
    fig.colorbar(ie, ax=ax[1:, 1], location="bottom", shrink=.8, aspect=30, label="absolute error (m)")
    fig.savefig(out, dpi=150); print("saved", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--time", type=float, required=True, help="camera frame and LiDAR scan (s)")
    ap.add_argument("--start", type=float, default=5.0); ap.add_argument("--seconds", type=float, default=50.0)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--sigma-x", type=float, default=100.0, help="px in the full frame"); ap.add_argument("--sigma-y", type=float, default=320.0)
    ap.add_argument("--sigma-c", type=float, default=0.08, help="Lab / 100")
    ap.add_argument("--ls-sigma", type=float, default=0.05, help="edge weight of the least-squares fill, Lab / 100")
    ap.add_argument("--ls-lambda", type=float, default=1.0, help="smoothness of the least-squares fill")
    ap.add_argument("--min-conf", type=float, default=0.05, help="share of the best confidence below which no depth is given")
    ap.add_argument("--verified", type=float, default=None, metavar="M",
                    help="upper bound, not a method: keep only seeds within M metres of the LiDAR at their own pixel")
    ap.add_argument("--object", type=float, nargs=3, default=None, metavar=("X", "Y", "R"),
                    help="also score one object: LiDAR points within R m (horizontally) of X, Y in the LiDAR frame")
    ap.add_argument("--object-split", type=float, default=None, metavar="H",
                    help="also score the object's parts below and above H m over the ground separately")
    ap.add_argument("--vmin", type=float, default=0.0, help="white end of the depth scale in the figure (m)")
    ap.add_argument("--vmax", type=float, default=DMAX, help="black end of the depth scale in the figure (m)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    trace = a.trace.rstrip("/"); cam_ts = np.fromfile(os.path.join(trace, "camera", "ts"), "<f8"); t_img = cam_ts[0] + a.time

    cells, n = static_cells(trace, cam_ts[0] + a.start, a.seconds)
    kept, lobes = drop_sidelobes(cells); seeds = first_returns(kept)
    uv, d, ok = to_image(seeds, trace); seeds, uv, d = seeds[ok], uv[ok], d[ok]
    uv_all, d_all, ok_all = to_image(cells, trace)
    print(f"{n} frames: {len(cells)} persistent cells, {len(lobes)} dropped as sidelobes, {len(seeds)} first returns in the image")

    rgb, _ = camera_frame(trace, t_img, a.out + "._frame.png"); small, lab = guide_image(rgb, a.scale)
    H, W = lab.shape[:2]; sx, sy = a.sigma_x * a.scale, a.sigma_y * a.scale
    ref, height, xyimg = lidar_depth_image(trace, t_img, a.scale, return_xy=True); above = height > 0.3
    obj = None
    if a.object is not None:
        obj = (np.hypot(xyimg[..., 0] - a.object[0], xyimg[..., 1] - a.object[1]) < a.object[2]) & above
    seed_err = seed_minus_lidar(uv * a.scale, d, ref); n_all = len(seeds)
    if a.verified is not None:                                           # the LiDAR picks the seeds: an upper bound for the fill
        v = np.abs(seed_err) < a.verified; seeds, uv, d, seed_err = seeds[v], uv[v], d[v], seed_err[v]
        print(f"upper bound: {len(seeds)} of {n_all} seeds are within {a.verified} m of the LiDAR")
    uvs = uv * a.scale
    w = seeds["share"].astype(float)
    fills, conf = {"nearest column": nearest_column(uvs, d, (H, W))}, {}
    fills["Gaussian"], _ = upsample(lab, uvs, d, sx, sy, None, w)
    fills["joint bilateral"], conf["joint bilateral"] = upsample(lab, uvs, d, sx, sy, a.sigma_c, w)
    # Point constraints alone are degenerate for a 2D smoothness term (the solution is flat with a dip at
    # each seed), so the least-squares fill is anchored on the whole seed row: every column takes its
    # nearest seed's depth at the row the seeds lie on, then depth spreads up and down along the image.
    order = np.argsort(uvs[:, 0]); cols_u = np.arange(W, dtype=float)
    row_v = np.interp(cols_u, uvs[order, 0], uvs[order, 1]); jn = np.abs(cols_u[:, None] - uvs[None, :, 0]).argmin(axis=1)
    fills["edge-aware least squares"], conf["edge-aware least squares"] = wls_fill(
        lab, np.stack([cols_u, row_v], axis=1), d[jn], a.ls_sigma, a.ls_lambda, radius=0, weight=w[jn])
    fills["constant (median seed)"] = np.full((H, W), float(np.median(d)))
    sure = {k: c > a.min_conf * c.max() for k, c in conf.items()}

    band = edge_band(ref)
    masks = {"above_ground": above, "edge_band": above & band, "off_band": above & ~band}
    if obj is not None:
        masks["object"] = obj
        if a.object_split is not None:
            masks["object, lower part"] = obj & (height < a.object_split); masks["object, upper part"] = obj & (height >= a.object_split)
    res = {}
    for name, D in fills.items():
        res[name] = {"all": errors(D, ref), **{k: errors(D, ref, m) for k, m in masks.items()}}
    for name, sm in sure.items():
        res[name + ", confident"] = {"all": errors(fills[name], ref, sm), **{k: errors(fills[name], ref, m & sm) for k, m in masks.items()},
                                     "coverage": float((sm & np.isfinite(ref) & above).sum() / max((np.isfinite(ref) & above).sum(), 1))}
    sil = ol = cls = None
    if obj is not None and obj.any():                                    # the object's outline, split by how well it shows in colour
        sil = silhouette(obj); ol = outline(sil, lab, ref); cls = classes(ol)
        res["outline"] = dict(px={k: int(v.sum()) for k, v in cls.items()}, contrast_lab=dict(distinct=15.0, similar=7.0),
                              range_inside_m=float(np.nanmedian(ol["d_in"][ol["edge"]])), range_outside_m=float(np.nanmedian(ol["d_out"][ol["edge"]])))
        for name, D in fills.items():
            res[name]["edge_accuracy"] = edge_accuracy(D, sil, ol, cls)
    res["seeds"] = dict(n=len(seeds), n_before_verification=n_all, az_deg=np.degrees(seeds["az"]).round(1).tolist(), range_m=d.round(2).tolist(), minus_lidar_m=np.round(seed_err, 2).tolist())
    res["params"] = dict(trace=os.path.basename(trace), time=a.time, start=a.start, seconds=a.seconds, frames=n, scale=a.scale,
                         sigma_x=a.sigma_x, sigma_y=a.sigma_y, sigma_c=a.sigma_c, ls_sigma=a.ls_sigma, ls_lambda=a.ls_lambda,
                         min_conf=a.min_conf, verified=a.verified, object=a.object, object_split=a.object_split)
    res["pixels"] = {k: int((m & np.isfinite(ref)).sum()) for k, m in masks.items()}
    json.dump(res, open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)
    rows = [k for k in res if isinstance(res[k], dict) and "all" in res[k]]
    if cls is not None:
        print("outline kept (share of pixels within 4 px on the right side): " + ", ".join(
            f"{n} {res[n]['edge_accuracy']['distinct']['accuracy']:.2f}/{res[n]['edge_accuracy']['similar']['accuracy']:.2f}" for n in fills) + "  (distinct/similar)")
    print(f"{'0-20 m, mean abs. error (m)':34s}" + "".join(f"{k[:14]:>15s}" for k in masks))
    for name in rows:
        print(f"{name:34s}" + "".join(f"{res[name][k]['0-20 m']['mae']:15.2f}" for k in masks)
              + (f"   coverage {res[name]['coverage']:.0%}" if "coverage" in res[name] else ""))

    plot(a.out, small, uvs, d, fills, ref, above, res, obj, sil, ol, cls, vmin=a.vmin, vmax=a.vmax)


if __name__ == "__main__":
    main()
