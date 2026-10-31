# Contributing (for Yining)

Thanks for helping. The goal for you: **rebuild objects in 3D from the walk-around recordings** (radar + LiDAR + camera). How you do it is open. This page says where the data is, how to get radar points, and what we already know.

## The task

We walked around four things on 10/29 with the rig. Pick one, get a 3D model of it, then try the others.

| Object | Trace | Time in trace | Notes |
|---|---|---|---|
| Jeep (parked car) | `2026-10-29.15-49-40` | 11:00–17:00 | full orbit, best one to start with |
| Walking to the Sky (metal pole) | `2026-10-29.16-12-38` | 5:30–9:40 | full orbit, rig often tilted up |
| Scotty statue | `2026-10-29.16-22-36` | 2:59–3:45 | partial loop; flat wall behind gives a ghost at 8–9 m |
| Planters | `2026-10-29.16-22-36`, `2026-10-29.16-27-55` | ~2:00 and ~0:00, ~1:30 | passes, not full loops |

What "done" looks like: a point cloud or mesh of the object in one fixed frame, a picture of it, and a short write-up in `experiments/obj-orbit-360/README.md` (question, method, numbers, figures, what did not work). Radar-only, LiDAR-only, or fused: your call. A web viewer would be great but is optional.

## Where the data is

On `nunga`: `/data/christineh/DATASET/radarhd/2026-10-29/<trace>/` (read-only). The README there explains the format. In this repo, `data/raw/<trace>` links to it.

Each trace has four folders, all on one Unix-time clock:

- `radar/` 20 Hz raw ADC. Read it with `src/radar` (below).
- `lidar/` 10 Hz, Ouster OS-0-128, `[128, 2048]` range image in mm, LZMA per frame. Read with `roverd.sensors.OSLidar`, or by hand: offsets in `rng_i`, `lzma.decompress`, beam angles in `lidar.json`.
- `camera/` 30 fps, `video.avi` + `ts`. Frame k ↔ `ts[k]`.
- `imu/` 100 Hz, `acc`, `avel`, `rot`.

**Match sensors by timestamp, not by index.** Radar starts ~2 s after the camera. `np.searchsorted(ts, t)`.

## Getting radar points

```bash
source .venv/bin/activate
python -m src.radar data/raw/2026-10-29.15-49-40 --time 720 --accumulate 20   # one second at t=720 s, saves a PNG
```

In Python:

```python
from src.radar import RadarTrace, detect, to_xyz, spectra

tr = RadarTrace("data/raw/2026-10-29.15-49-40")
k = tr.index_at(t_unix)                 # frame nearest a timestamp
pts, info = detect(tr.frame(k), tr.params)   # pts: r, az, el, v, snr per detection
xyz = to_xyz(pts)                       # x right, y forward, z up, metres, radar frame
info["power"]                           # dense range-azimuth map (64 az x 256 range) if you want more than points
```

What you need to know about these points:

- About 10–20 per frame. `snr` tells you which to trust. Lower `pfa` in `detect()` for more points.
- `az` + is left. Range is good to ~1 cm after a +0.18 m offset. Azimuth beam is ~14° wide, so a point target smears into an arc.
- **`z` (elevation) is not usable yet.** Treat points as range + azimuth; height must come from LiDAR or camera.
- `v` is radial velocity. While walking the static world shows v ≈ −v_walk·cos(az). Points off that curve are ghosts, sidelobes or moving people.
- Ghost echoes exist at 2× a strong target's range, and behind flat walls.

## What is missing (and you may need to make)

- **Poses.** No pose per frame yet. LiDAR SLAM on the 128-beam scans is the obvious route; the IMU `rot` is a check.
- **Radar ↔ LiDAR extrinsics.** Done in 2D (yaw + xy, ~8 cm residual), see `experiments/m4-extrinsics/`. Use it:
  ```python
  from src.calib.transform import RadarToLidar
  T = RadarToLidar.load()
  xyz_lidar = T(to_xyz(pts))          # radar points -> LiDAR frame; z is passed through, not calibrated
  ```
  Check it on any trace: `python -m src.calib.overlay data/raw/<trace> --time <s> --accumulate 10 --out x.png`.
- **Radar ↔ camera extrinsics.** Done by PnP, 35 px reprojection error (radar bearing limited). `C = RadarToCamera.load(); uv = C.pixels(xyz_radar)` gives pixels in the 1920×1080 frame. Intrinsics: `calib/camera_fisheye_iq1m.json` (OpenCV fisheye, from red-rover).

## Rules (short)

1. Branch from `main`: `yining/<topic>`. Code in `src/ext/<topic>/`. PR into `main`, small ones.
2. Use `src/radar`, do not edit it. If it needs a change, say so in the PR.
3. Results go in `experiments/<id>/README.md` with a line in `EXPERIMENTS.md`. Every figure says which command and commit made it.
4. English only in code, figures and docs.
5. Camera frames show people and plates. Do not publish them without blurring.
