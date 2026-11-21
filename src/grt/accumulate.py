"""GRT through the same accumulation score as our CFAR chain: world frame, the same poses, the same LiDAR layer.

    .venv/bin/python -m src.grt.accumulate /data/berlin/grt/2026-11-19.17-55-35 --trace data/raw/2026-11-19.17-55-35

Per dump frame (every `stride`th), GRT's first occupied cell per (elevation, azimuth) column at
p > threshold, kept if it lies 0.3-2.5 m above the ground (the LiDAR layer of `src.recon.accumulate`,
taken here as LiDAR-frame z between -h+0.3 and -h+2.5 with h the LiDAR's height on its ground plane),
put in the world with the KISS-ICP pose at that radar time. Scored exactly as the CFAR points in
`src.recon.accumulate`: the share within 0.5 m (xy) of the LiDAR layer, among points within 15 m of
the path, and the coverage of the 0.5 m LiDAR cells. Needs data/recon/<trace>/lidar_points.npz.
"""
import argparse, json, os

import numpy as np
from scipy.spatial import cKDTree

from .occupancy import GridGeometry
from .align import first_return
from ..recon.accumulate import coverage
from ..fusion.stack import ground_plane, lidar_cloud


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--trace", required=True); ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--thresholds", type=float, nargs="+", default=[0.5, 0.9]); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    name = os.path.basename(a.trace.rstrip("/")); G = GridGeometry(a.trace)
    logit = np.load(os.path.join(a.dump, "occ3d_logit.npy"), mmap_mode="r"); ts = np.load(os.path.join(a.dump, "ts.npy"))
    P = np.load(os.path.join("data/poses", name, "radar.npz")); lid = np.load(os.path.join("data/recon", name, "lidar_points.npz"))["xyz"]
    ts0 = np.fromfile(os.path.join(a.trace, "camera", "ts"), "<f8")[0]
    h = ground_plane(lidar_cloud(a.trace, ts[len(ts) // 2]))[1]          # LiDAR height, from a scan mid-window
    el, az, rg = np.meshgrid(np.arange(G.n_el), np.arange(G.n_az), np.arange(G.n_rng), indexing="ij")
    res = {}; tree = cKDTree(lid[:, :2])
    for thr in a.thresholds:
        world, path = [], []
        for i in range(0, len(ts), a.stride):
            k = int(np.argmin(np.abs(P["t"] - ts[i]))); T = P["T"][k]
            occ = 1 / (1 + np.exp(-np.asarray(logit[i], dtype=np.float32))) > thr
            idx = first_return(occ[None])[0]; e, z = np.nonzero(idx >= 0)
            xyz = G.xyz(np.stack([e, z, idx[e, z]], axis=1)); xyz = xyz[(xyz[:, 2] > -h + 0.3) & (xyz[:, 2] < -h + 2.5)]
            world.append(xyz @ T[:3, :3].T + T[:3, 3]); path.append(T[:2, 3])
        world = np.concatenate(world); path = np.array(path)
        near = cKDTree(path).query(world[:, :2])[0] < 15.0; d = tree.query(world[near, :2])[0]
        cov, n_cells = coverage(lid[:, :2], world[:, :2], path)
        res[str(thr)] = dict(points=int(len(world)), frames=int(len(path)), points_per_frame=float(len(world) / len(path)),
                             within_0_5m=float((d < 0.5).mean()), median_m=float(np.median(d)), coverage=float(cov), lidar_cells=int(n_cells))
        r = res[str(thr)]
        print(f"{name} GRT p > {thr}: {r['frames']} frames, {r['points_per_frame']:.0f} points/frame, within 0.5 m of the LiDAR layer {r['within_0_5m']:.0%} "
              f"(median {r['median_m']:.2f} m), coverage {r['coverage']:.0%} of {n_cells} cells")
    res["params"] = dict(dump=a.dump, trace=name, stride=a.stride, lidar_height_m=float(h), window_s=[float(ts[0] - ts0), float(ts[-1] - ts0)])
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1); print("saved", a.out)


if __name__ == "__main__":
    main()
