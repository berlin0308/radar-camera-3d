"""Does GRT's elevation follow the object or the radar? Its occupancy on the statue, take by take, in the height stack.

    .venv/bin/python -m src.grt.heights --ref data/raw/2026-11-19.17-40-44 --takes data/raw/2026-11-19.17-{40-44,...} \
        --object -2.65 0.08 --out figures/grt_heights_scotty_1119.png

A CFAR seed has no height: it is placed on the radar's plane. GRT outputs a full 3D grid, so it can
put the statue where it is, or where the radar is. For each take: GRT's probability averaged over
the take (the rig is still), the cells within `radius` of the statue's axis, and the height of each
above the ground. Compared with nrdk's LiDAR label in the same cells and with the height of the
radar's plane at the statue (from `src.fusion.stack`).
"""
import argparse, json, os

import numpy as np

from .occupancy import GridGeometry
from ..fusion.stack import lidar_cloud, ground_plane, initial_guess, icp, seeds_in_ref


def statue_cells(G, centre, radius, plane, h_max=2.4):
    """Indices (el, az, rng) of the grid cells whose centres lie within `radius` of the statue's vertical axis, and their heights."""
    el, az, rg = np.meshgrid(np.arange(G.n_el), np.arange(G.n_az), np.arange(G.n_rng), indexing="ij")
    cells = np.stack([el.ravel(), az.ravel(), rg.ravel()], axis=1); xyz = G.xyz(cells)
    h = xyz @ plane[0] + plane[1]; m = (np.hypot(*(xyz[:, :2] - centre).T) < radius) & (h > 0.05) & (h < h_max)
    return cells[m], h[m]


def profile(p, h, bins):
    """Mean of p per height bin."""
    i = np.digitize(h, bins) - 1; return np.array([p[i == k].mean() if (i == k).any() else np.nan for k in range(len(bins) - 1)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True); ap.add_argument("--takes", nargs="+", required=True)
    ap.add_argument("--dumps", default="/data/berlin/grt"); ap.add_argument("--object", type=float, nargs=2, required=True)
    ap.add_argument("--radius", type=float, default=0.6); ap.add_argument("--time", type=float, default=20.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ts0 = lambda t: np.fromfile(os.path.join(t, "camera", "ts"), "<f8")[0]
    ref = a.ref.rstrip("/"); rc = lidar_cloud(ref, ts0(ref) + a.time); centre = np.array(a.object)
    bins = np.arange(0.0, 2.45, 0.1); mid = (bins[:-1] + bins[1:]) / 2; res = {}
    for tr in sorted(t.rstrip("/") for t in a.takes):
        name = os.path.basename(tr); c = lidar_cloud(tr, ts0(tr) + a.time); T0, _, _ = initial_guess(c, rc); T, _, _ = icp(c, rc, T0)
        plane = ground_plane(c)                                          # the take's own ground, in its own LiDAR frame
        Ti = np.linalg.inv(T); ck = (np.r_[centre, 0.0] @ Ti[:3, :3].T + Ti[:3, 3])[:2]   # statue axis in this take's LiDAR frame
        g = np.mgrid[-4:4:0.02, 0.5:5:0.02].reshape(2, -1).T; X = np.c_[g, np.zeros(len(g))]
        _, _, xr, _ = seeds_in_ref(X, tr, T, ref); rp = ground_plane(rc)
        beam_h = float(xr[np.argmin(np.hypot(*(xr[:, :2] - centre).T))] @ rp[0] + rp[1])
        G = GridGeometry(tr); cells, h = statue_cells(G, ck, a.radius, plane)
        logit = np.load(os.path.join(a.dumps, name, "occ3d_logit.npy"), mmap_mode="r"); lab = np.load(os.path.join(a.dumps, name, "occ3d_label.npy"), mmap_mode="r")
        idx = tuple(cells.T); P = np.zeros(len(cells)); L = np.zeros(len(cells)); n = 0
        for k in range(0, len(logit), 4):
            P += 1 / (1 + np.exp(-np.asarray(logit[k][idx], dtype=np.float32))); L += np.asarray(lab[k][idx]); n += 1
        P /= n; L /= n
        w = lambda q: float((q * h).sum() / max(q.sum(), 1e-9))
        occ = h[P > 0.5]
        res[name] = dict(beam_height_m=beam_h, grt_mean_height_m=w(P), label_mean_height_m=w(L),
                         grt_span_m=[float(np.percentile(occ, 5)), float(np.percentile(occ, 95))] if len(occ) else None,
                         label_span_m=[float(np.percentile(h[L > 0.5], 5)), float(np.percentile(h[L > 0.5], 95))] if (L > 0.5).any() else None,
                         grt_profile=profile(P, h, bins).tolist(), label_profile=profile(L, h, bins).tolist(), frames=n, cells=int(len(cells)))
        r = res[name]
        print(f"{name}: radar plane at the statue {beam_h:.2f} m | GRT mean height {r['grt_mean_height_m']:.2f} m, p>0.5 span {np.round(r['grt_span_m'], 2) if r['grt_span_m'] else '-'} "
              f"| label mean {r['label_mean_height_m']:.2f} m, span {np.round(r['label_span_m'], 2) if r['label_span_m'] else '-'} ({n} frames, {len(cells)} cells)")
    bh = np.array([r["beam_height_m"] for r in res.values()]); gh = np.array([r["grt_mean_height_m"] for r in res.values()])
    slope = float(np.polyfit(bh, gh, 1)[0]); print(f"GRT mean height against the radar's height: slope {slope:.2f} (1 = follows the radar, 0 = follows the statue)")
    json.dump(dict(per_take=res, bins_m=bins.tolist(), slope=slope, params=vars(a)), open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    cmap = matplotlib.colormaps["viridis"]; lo, hi = bh.min() - 0.05, bh.max() + 0.05
    fig, ax = plt.subplots(1, 2, figsize=(8.4, 3.6), constrained_layout=True, sharey=True)
    for name, r in sorted(res.items(), key=lambda kv: kv[1]["beam_height_m"]):
        c = cmap((r["beam_height_m"] - lo) / (hi - lo))
        ax[0].plot(r["grt_profile"], mid, color=c, lw=1.4, label=f"{r['beam_height_m']:.2f} m")
        ax[1].plot(r["label_profile"], mid, color=c, lw=1.4)
        for A in ax:
            A.plot([0, 1], [r["beam_height_m"]] * 2, color=c, lw=0.8, ls=":")
    ax[0].set_title("GRT on the statue", fontsize=10, fontweight="bold"); ax[1].set_title("LiDAR label on the statue", fontsize=10, fontweight="bold")
    ax[0].set_ylabel("height above ground (m)")
    for A in ax:
        A.axhline(1.05, color="0.6", lw=0.8); A.set_xlim(0, 1); A.set_xlabel("mean occupancy")
    ax[0].text(0.98, 1.07, "pedestal top", ha="right", fontsize=7, color="0.4")
    ax[0].legend(fontsize=6.5, frameon=False, loc="upper right", title="radar height\n(dotted)", title_fontsize=7)
    fig.savefig(a.out, dpi=150); print("saved", a.out)


if __name__ == "__main__":
    main()
