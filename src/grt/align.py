"""Does GRT output sit on our LiDAR? Compare GRT with nrdk's LiDAR label on the same frames the way
the GRT paper scores itself: the first occupied range cell along each (elevation, azimuth) column is
a depth map. Depth error per column, Chamfer between the two depth-map point clouds, and a sweep of
a residual azimuth shift (cells of 1.4 deg) applied to GRT.

    .venv/bin/python -m src.grt.align /data/berlin/grt/2026-10-29.16-22-36 --trace data/raw/2026-10-29.16-22-36 --out figures/grt_align_scotty.png
"""
import argparse, json, os

import numpy as np
from scipy.spatial import cKDTree

from .occupancy import GridGeometry


def first_return(occ):
    """(N, el, az, rng) bool -> (N, el, az) index of the first occupied range cell, -1 if none."""
    any_ = occ.any(axis=-1); idx = occ.argmax(axis=-1); idx[~any_] = -1
    return idx


def depth_points(idx, G, rows):
    """Depth-map indices (el, az) -> xyz of the columns that have a return, rows restricted."""
    el, az = np.nonzero(idx[rows] >= 0); el = el + rows.start
    k = idx[el, az]
    return G.xyz(np.stack([el, az, k], axis=1))


def chamfer(a, b):
    if len(a) == 0 or len(b) == 0: return np.nan, np.nan
    d_ab = cKDTree(b).query(a)[0]; d_ba = cKDTree(a).query(b)[0]
    return float(np.median(d_ab)), float(np.median(d_ba))        # a->b (accuracy), b->a (completeness), medians


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--trace", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--threshold", type=float, default=0.5); ap.add_argument("--max-frames", type=int, default=300); ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--rows", default="4:44", help="elevation rows to score (default +40 .. -17 deg: structures, not the ground under the rig)")
    ap.add_argument("--frame", type=int, default=None)
    a = ap.parse_args()
    r0, r1 = (int(x) for x in a.rows.split(":")); rows = slice(r0, r1)
    logit = np.load(os.path.join(a.dump, "occ3d_logit.npy"), mmap_mode="r"); lab_all = np.load(os.path.join(a.dump, "occ3d_label.npy"), mmap_mode="r"); ts_all = np.load(os.path.join(a.dump, "ts.npy"))
    sel = np.arange(0, min(len(ts_all), a.max_frames * a.stride), a.stride)
    prob = 1 / (1 + np.exp(-np.asarray(logit[sel], dtype=np.float32))); lab = np.asarray(lab_all[sel]); ts = ts_all[sel]
    G = GridGeometry(a.trace); cell_deg = np.degrees(G.az[0] - G.az[1])
    d_lab = first_return(lab)
    print(f"{len(ts)} frames (every {a.stride}th), rows {r0}:{r1} = {np.degrees(G.el[r0]):+.0f}..{np.degrees(G.el[r1-1]):+.0f} deg; "
          f"label has a return in {(d_lab[:, rows] >= 0).mean()*100:.0f}% of columns")

    def score(pred_occ, daz=0):
        """Depth error (m) over columns where both have a return, and Chamfer medians, after shifting GRT by daz cells."""
        p = np.roll(pred_occ, daz, axis=2)
        if daz > 0: p[:, :, :daz] = False
        elif daz < 0: p[:, :, daz:] = False
        d_pred = first_return(p)
        both = (d_pred[:, rows] >= 0) & (d_lab[:, rows] >= 0)
        err = np.abs(d_pred[:, rows] - d_lab[:, rows])[both] * G.d_rng
        cov = (d_pred[:, rows] >= 0)[d_lab[:, rows] >= 0].mean()
        ch = [chamfer(depth_points(d_pred[i], G, rows), depth_points(d_lab[i], G, rows)) for i in range(len(ts))]
        ch = np.nanmedian(np.array(ch), axis=0)
        return dict(median_m=float(np.median(err)) if len(err) else np.nan, within_1cell=float((err <= G.d_rng).mean()) if len(err) else np.nan,
                    within_1m=float((err <= 1.0).mean()) if len(err) else np.nan, coverage=float(cov), chamfer_pred_to_lab=float(ch[0]), chamfer_lab_to_pred=float(ch[1]))

    pred = prob > a.threshold
    s0 = score(pred)
    print(f"zero shift, threshold {a.threshold}: depth error median {s0['median_m']:.2f} m, within 1 cell {s0['within_1cell']*100:.0f}%, within 1 m {s0['within_1m']*100:.0f}%, "
          f"GRT covers {s0['coverage']*100:.0f}% of label columns; Chamfer medians GRT->LiDAR {s0['chamfer_pred_to_lab']:.2f} m, LiDAR->GRT {s0['chamfer_lab_to_pred']:.2f} m")
    sweep = {d: score(pred, d) for d in range(-8, 9)}
    best = min(sweep, key=lambda d: sweep[d]["chamfer_pred_to_lab"] + sweep[d]["chamfer_lab_to_pred"])
    print(f"yaw sweep (Chamfer sum): best {best:+d} cells = {best*cell_deg:+.1f} deg -> {sweep[best]['chamfer_pred_to_lab']:.2f} / {sweep[best]['chamfer_lab_to_pred']:.2f} m "
          f"(zero: {s0['chamfer_pred_to_lab']:.2f} / {s0['chamfer_lab_to_pred']:.2f} m); per shift: " + " ".join(f"{d*cell_deg:+.0f}°:{sweep[d]['chamfer_pred_to_lab']+sweep[d]['chamfer_lab_to_pred']:.2f}" for d in sweep))
    thr = {}
    for t in (0.3, 0.5, 0.7, 0.9):
        thr[t] = score(prob > t); print(f"  threshold {t:.1f}: depth median {thr[t]['median_m']:.2f} m, within 1 m {thr[t]['within_1m']*100:.0f}%, coverage {thr[t]['coverage']*100:.0f}%, Chamfer {thr[t]['chamfer_pred_to_lab']:.2f} / {thr[t]['chamfer_lab_to_pred']:.2f} m")
    json.dump(dict(frames=int(len(ts)), stride=a.stride, rows=[r0, r1], threshold=a.threshold, cell_deg=float(cell_deg), cell_m=float(G.d_rng),
                   zero_shift=s0, best_shift=dict(cells=int(best), deg=float(best * cell_deg), **sweep[best]),
                   sweep={str(d): v["median_m"] for d, v in sweep.items()}, thresholds={str(t): v for t, v in thr.items()}),
              open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    k = a.frame if a.frame is not None else len(ts) // 2
    d_pred = first_return(pred[k:k + 1])[0]
    dl = np.where(d_lab[k] >= 0, G.rng[np.clip(d_lab[k], 0, None)], np.nan); dp = np.where(d_pred >= 0, G.rng[np.clip(d_pred, 0, None)], np.nan)
    ext = [np.degrees(G.az[0]), np.degrees(G.az[-1]), np.degrees(G.el[-1]), np.degrees(G.el[0])]
    fig, ax = plt.subplots(1, 4, figsize=(16.64, 4.6), constrained_layout=True, gridspec_kw=dict(width_ratios=[1.2, 1.2, 1.2, .9]))
    for axx, img, title in [(ax[0], dl, f"a · LiDAR label depth, frame {sel[k]}"), (ax[1], dp, "b · GRT first-return depth, same frame")]:
        im = axx.imshow(img, extent=ext, aspect="auto", cmap="turbo", vmin=0, vmax=22); axx.set_title(title, fontsize=11); axx.set_xlabel("azimuth (deg, lidar frame)"); axx.set_ylabel("elevation (deg)")
        axx.axhline(np.degrees(G.el[r0]), color="w", lw=.6, ls="--"); axx.axhline(np.degrees(G.el[r1 - 1]), color="w", lw=.6, ls="--")
    fig.colorbar(im, ax=ax[1], shrink=.8, label="depth (m)")
    im2 = ax[2].imshow(dp - dl, extent=ext, aspect="auto", cmap="coolwarm", vmin=-3, vmax=3); ax[2].set_title("c · GRT minus LiDAR (m), columns both see", fontsize=11); ax[2].set_xlabel("azimuth (deg)")
    fig.colorbar(im2, ax=ax[2], shrink=.8)
    ax[3].plot([d * cell_deg for d in sweep], [sweep[d]["chamfer_pred_to_lab"] + sweep[d]["chamfer_lab_to_pred"] for d in sweep], "o-", color="#2C43B0", ms=4)
    ax[3].axvline(0, color="#B9C2CC"); ax[3].axvline(best * cell_deg, color="#E07B39", ls="--")
    ax[3].set_xlabel("azimuth shift applied to GRT (deg)"); ax[3].set_ylabel("Chamfer, both directions (m)"); ax[3].set_title(f"d · residual yaw sweep, best {best*cell_deg:+.1f} deg", fontsize=11)
    fig.suptitle(f"{os.path.basename(a.trace.rstrip('/'))}: GRT base vs our LiDAR as first-return depth, {len(ts)} frames, rows {np.degrees(G.el[r0]):+.0f}..{np.degrees(G.el[r1-1]):+.0f} deg · "
                 f"median error {s0['median_m']:.2f} m, Chamfer {s0['chamfer_pred_to_lab']:.2f} / {s0['chamfer_lab_to_pred']:.2f} m", fontsize=11)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True); fig.savefig(a.out, dpi=130); print("saved", a.out)


if __name__ == "__main__":
    main()
