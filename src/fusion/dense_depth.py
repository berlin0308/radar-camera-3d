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
from ..eval.lidar_depth import lidar_depth_image, errors
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace"); ap.add_argument("--time", type=float, required=True, help="camera frame and LiDAR scan (s)")
    ap.add_argument("--start", type=float, default=5.0); ap.add_argument("--seconds", type=float, default=50.0)
    ap.add_argument("--scale", type=float, default=0.25)
    ap.add_argument("--sigma-x", type=float, default=100.0, help="px in the full frame"); ap.add_argument("--sigma-y", type=float, default=320.0)
    ap.add_argument("--sigma-c", type=float, default=0.08, help="Lab / 100")
    ap.add_argument("--min-conf", type=float, default=0.05, help="share of the best confidence below which no depth is given")
    ap.add_argument("--verified", type=float, default=None, metavar="M",
                    help="upper bound, not a method: keep only seeds within M metres of the LiDAR at their own pixel")
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
    ref, height = lidar_depth_image(trace, t_img, a.scale); above = height > 0.3
    seed_err = seed_minus_lidar(uv * a.scale, d, ref); n_all = len(seeds)
    if a.verified is not None:                                           # the LiDAR picks the seeds: an upper bound for the fill
        v = np.abs(seed_err) < a.verified; seeds, uv, d, seed_err = seeds[v], uv[v], d[v], seed_err[v]
        print(f"upper bound: {len(seeds)} of {n_all} seeds are within {a.verified} m of the LiDAR")
    uvs = uv * a.scale
    w = seeds["share"].astype(float)
    fills = {"nearest column": nearest_column(uvs, d, (H, W))}
    fills["Gaussian"], conf_g = upsample(lab, uvs, d, sx, sy, None, w)
    fills["joint bilateral"], conf = upsample(lab, uvs, d, sx, sy, a.sigma_c, w)
    sure = conf > a.min_conf * conf.max()

    res = {name: dict(all=errors(D, ref), above_ground=errors(D, ref, above)) for name, D in fills.items()}
    res["joint bilateral, confident"] = dict(all=errors(fills["joint bilateral"], ref, sure), above_ground=errors(fills["joint bilateral"], ref, sure & above),
                                             coverage=float((sure & np.isfinite(ref) & above).sum() / max((np.isfinite(ref) & above).sum(), 1)))
    res["seeds"] = dict(n=len(seeds), n_before_verification=n_all, az_deg=np.degrees(seeds["az"]).round(1).tolist(), range_m=d.round(2).tolist(), minus_lidar_m=np.round(seed_err, 2).tolist())
    res["params"] = dict(trace=os.path.basename(trace), time=a.time, start=a.start, seconds=a.seconds, frames=n, scale=a.scale,
                         sigma_x=a.sigma_x, sigma_y=a.sigma_y, sigma_c=a.sigma_c, min_conf=a.min_conf, verified=a.verified)
    json.dump(res, open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)
    for name in fills:
        e = res[name]["above_ground"]; print(f"{name:16s} above ground: " + "  ".join(f"{k} {v['mae']:.2f} m (n={v['n']})" for k, v in e.items()))
    e = res["joint bilateral, confident"]; print(f"{'JB, confident':16s} above ground: " + "  ".join(f"{k} {v['mae']:.2f} m (n={v['n']})" for k, v in e["above_ground"].items()) + f"  coverage {e['coverage']:.0%}")

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 5, figsize=(22, 6.4), constrained_layout=True)
    kw = dict(cmap="turbo", vmin=0, vmax=DMAX)
    ax[0, 0].imshow(np.clip(small * 2.2, 0, 1)); m = ok_all
    ax[0, 0].scatter(uv_all[m, 0] * a.scale, uv_all[m, 1] * a.scale, s=14, c="none", edgecolors="0.75", linewidths=.6)
    ax[0, 0].scatter(uvs[:, 0], uvs[:, 1], s=46, c=d, edgecolors="white", linewidths=.8, **kw)
    ax[0, 0].set_title(f"a · camera (brightened) and {len(seeds)} radar seeds\n(grey: cells dropped as sidelobes or behind a nearer one)", fontsize=10)
    for j, name in enumerate(fills):
        D = fills[name].copy()
        if name == "joint bilateral": D[~sure] = np.nan
        im = ax[0, j + 1].imshow(D, **kw); ax[0, j + 1].set_title(f"{'bcd'[j]} · {name}", fontsize=10)
    refd = ref.copy(); ax[0, 4].imshow(np.clip(small * 2.2, 0, 1) * 0.35)
    yy, xx = np.nonzero(np.isfinite(ref)); ax[0, 4].scatter(xx, yy, s=1.5, c=ref[yy, xx], marker="s", linewidths=0, **kw)
    ax[0, 4].set_title("e · LiDAR reference (not used by b, c, d)", fontsize=10)
    fig.colorbar(im, ax=ax[0, :], shrink=.85, label="range from the camera (m)", pad=0.01)
    ax[1, 0].imshow(conf / conf.max(), cmap="gray", vmin=0, vmax=1); ax[1, 0].set_title("f · confidence of d (its denominator)", fontsize=10)
    ekw = dict(cmap="magma", vmin=0, vmax=5)
    for j, name in enumerate(fills):
        ax[1, j + 1].imshow(np.zeros((H, W)), cmap="gray", vmin=0, vmax=1)
        mm = np.isfinite(ref) & above & (sure if name == "joint bilateral" else True); yy, xx = np.nonzero(mm)
        ie = ax[1, j + 1].scatter(xx, yy, s=1.5, c=np.abs(fills[name] - ref)[yy, xx], marker="s", linewidths=0, **ekw)
        ax[1, j + 1].set_xlim(0, W); ax[1, j + 1].set_ylim(H, 0)
        ax[1, j + 1].set_title(f"{'ghi'[j]} · |{name} − LiDAR|, above ground", fontsize=10)
    fig.colorbar(ie, ax=ax[1, 1:4], shrink=.85, label="absolute error (m)", pad=0.01)
    names = list(fills) + ["joint bilateral, confident"]; bins = list(res[names[0]]["above_ground"])
    x = np.arange(len(bins)); bw = 0.2
    for i, name in enumerate(names):
        ax[1, 4].bar(x + (i - 1.5) * bw, [res[name]["above_ground"][b]["mae"] for b in bins], bw, label=name)
    ax[1, 4].set_xticks(x); ax[1, 4].set_xticklabels(bins); ax[1, 4].set_ylabel("mean absolute error (m)"); ax[1, 4].legend(fontsize=8)
    ax[1, 4].set_title("j · error above ground, by LiDAR distance", fontsize=10)
    for axx in ax.ravel()[:9]:
        axx.set_xticks([]); axx.set_yticks([])
    note = f" · UPPER BOUND: seeds picked by the LiDAR ({len(seeds)} of {n_all})" if a.verified is not None else ""
    fig.suptitle(f"{os.path.basename(trace)} · dense depth from {len(seeds)} radar seeds ({n} frames, rig static) and one camera frame{note}", fontsize=12)
    fig.savefig(a.out, dpi=110); print("saved", a.out)


if __name__ == "__main__":
    main()
