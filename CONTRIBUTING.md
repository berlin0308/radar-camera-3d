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

On `nunga`: `/data/christineh/DATASET/radarhd/2026-10-29/<trace>/` (read-only). The README there explains the format. In this repo, `data/raw/<trace>` links to it. Which session recorded what, and how good each trace is: `SESSIONS.md`.

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
- `v` is radial velocity, **negative = approaching**. While walking the static world shows v ≈ −v_walk·cos(az − heading). Points off that curve are ghosts, sidelobes or moving people. `src/radar/ego.py` does this test for you: `filter_static(pts, info, tr.params)` keeps the static-world points using the radar's own ego-velocity estimate (no poses needed), `static_mask_from_velocity(pts, v_xy)` does the same with a velocity you know (e.g. from the LiDAR poses).
- Ghost echoes exist at 2× a strong target's range, and behind flat walls.

## What is missing (and you may need to make)

- **Poses.** A first version exists: KISS-ICP odometry on the LiDAR scans, `python -m src.recon.poses <trace> --start <s> --seconds <n>` → `data/poses/<trace>/lidar.npz` (per scan) and `radar.npz` (per radar frame, keys `t`, `T` [N,4,4] lidar→world, `index`). Keep the defaults (IMU-gyro deskew, 30 m / 0.3 m map): KISS-ICP's own deskew and its 50 m / 0.5 m defaults both lose the heading on fast swings of this handheld rig, see `experiments/obj-orbit-360/`. No loop closure, so drift over a long orbit is yours to check (or replace with FAST-LIO2 / a loop-closing SLAM). Per-scan jitter is ~0.1 m, so smooth before you differentiate.
- **Accumulation.** `python -m src.recon.accumulate <trace> --start <s> --seconds <n> --out x.png` puts the static-filtered CFAR points and the LiDAR (ground removed) into the world frame with those poses → `data/recon/<trace>/radar_points.npz`, `lidar_points.npz`. Start from this for the object reconstruction; results so far are in `experiments/obj-orbit-360/README.md`. Flatten radar points with `to_xy` (slant range in the plane), not `to_xyz` then z = 0: the elevation estimate is noise and cos(el) shrinks the range.
- **One object up close.** `python -m src.recon.closeup <trace> --centre x y --far <s> --near <s> --orbit <s0> <s1> --split --out x.png` projects the LiDAR scan and the radar points on two camera frames, cuts the object out of every scan of the orbit, re-centred on its pedestal (a fixed-radius circle fit, which takes the odometry's jitter out but not its yaw), and draws the radar points and power around it. `--centre` is the object's world-frame position, a guess good to 0.5 m (the orbit's centre). The `Scan` class in there is the piece to reuse: one scan, its ground, the object's refined centre.

## Calibration (done, use it)

Radar ↔ LiDAR ↔ camera extrinsics exist, from the 10/21 board shots, solved jointly with the LiDAR frame as world. Numbers and figures: `experiments/m4-extrinsics/`.

```python
from src.calib.transform import RadarToLidar, RadarToCamera
T = RadarToLidar.load_joint()
xyz_lidar = T(to_xyz(pts))          # radar points -> LiDAR frame (Ouster sensor frame). This is what you feed SLAM poses.
C = RadarToCamera.load_joint()
uv = C.pixels(to_xyz(pts))          # radar points -> pixels, 1920x1080 fisheye frame
```

What to expect: yaw good to ±1.5°, in-plane offset ±5 cm, height ±15 cm, camera ±3° / ±10–14 cm. Radar pitch and roll are assumed 0. `z` of radar points is still unusable, so set it to 0 (or take height from LiDAR) before projecting.

Look at it before trusting it:

```bash
python -m src.calib.overlay  data/raw/<trace> --time <s> --accumulate 10 --joint --out x.png   # radar points on the LiDAR scan, top view
python -m src.calib.examples --wide --pick 0 3 5 --out x.png                                    # radar points on photos, none vs calibrated
```

The 10/29 traces were recorded after a LiDAR swap (OS-0-64 → OS-0-128). The transform still holds there to ~20–30 cm; if your reconstruction looks offset, that is the first suspect.

## GRT occupancy (learned radar), if you want a denser input

`GRT.md` has the full story. Short version: the released GRT checkpoint runs on our traces unchanged and outputs a 64 × 128 × 64 occupancy grid **already in the LiDAR frame** (no calibration needed, just the LiDAR pose). Dumps for the Scotty and Jeep windows are in `/data/berlin/grt/<trace>/` (`occ3d_logit.npy` float16 `[N, el, az, rng]`, `occ3d_label.npy`, `ts.npy`).

```python
from src.grt.occupancy import GridGeometry
G = GridGeometry("data/raw/2026-10-29.15-49-40")
xyz, p = G.points(logit[k], threshold=0.7)      # occupied cells of frame k -> xyz in the LiDAR frame, with probabilities
```

Expect it to be smeared: 14° wide in azimuth, little elevation, first returns biased 1–3 m near at p > 0.5. Use the probabilities (log-odds fusion) rather than a hard threshold, and compare with the CFAR points on the same poses.

## Rules (short)

1. Branch from `main`: `yining/<topic>`. Code in `src/ext/<topic>/`. PR into `main`, small ones.
2. Use `src/radar`, do not edit it. If it needs a change, say so in the PR.
3. Results go in `experiments/<id>/README.md` with a line in `EXPERIMENTS.md`. Every figure says which command and commit made it.
4. English only in code, figures and docs.
5. Camera frames show people and plates. Do not publish them without blurring.
