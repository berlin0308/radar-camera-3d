# Radar–camera 3D camera (15-663 final project)

Classical FMCW radar (TI AWR1843 + DCA1000 on the WiSE Lab red-rover rig) plus a fisheye camera,
turned into a 3D camera: raw ADC → range / Doppler / angle → 3D points → camera-guided dense depth,
evaluated against LiDAR.

**[Experiment reports →](EXPERIMENTS.md)** every experiment with its numbers, figures and what is still pending.

Author: Po-Lin Chen. Contributor: Yining (extensions, 3D reconstruction; acknowledged in the report).

## Layout
| Path | What |
|---|---|
| `shoot.sh`, `sync.sh`, `validate.sh`, `check_valid.py` | rig-day recording and data checks |
| `AGENDA_session2.md` | rig-day procedure and shot list for the next sessions |
| `notes/shots.csv` | which trace is which shot |
| `notes/fig/` | figures used in the Notebook deck |
| `src/radar/` | own radar chain: `adc.py` → `range_doppler.py` → `angle.py` → `cfar.py` → `pointcloud.py`; `python -m src.radar <trace>` runs it on one frame |
| `src/calib/` | extrinsics from the 10/21 board shots: `radar_lidar` (Kabsch), `board_image` + `radar_camera` (PnP), `joint` (one weighted solve, LiDAR frame); `RadarToLidar` / `RadarToCamera` to apply them; `overlay`, `compare`, `examples` to look at the result |
| `calib/` | `extrinsics_joint.json` (use this), the pairwise `extrinsics_radar_lidar.json` / `extrinsics_radar_camera.json` / `extrinsics_lidar_camera.json`, `board_pixels.json`, fisheye intrinsics from red-rover |
| `src/grt/` | the learned model next to the classical chain: `infer` runs a released GRT checkpoint on a trace (nrdk env, see `GRT.md`), `occupancy` turns grid cells into xyz with the real Ouster angles, `align` scores GRT against our LiDAR as first-return depth, `compare_frame` draws camera / CFAR / GRT / label for one instant |
| `src/recon/` | from frames to a world frame: `poses` (KISS-ICP on the Ouster scans, gyro-deskewed, poses at radar times), `ego_check` (radar Doppler of the static world against the odometry), `accumulate` (CFAR points with the static-world filter into the world frame next to the LiDAR built the same way, ground removed; `--dense` exports the range-azimuth power of every frame) |
| `src/radar/ego.py` | ego-velocity from Doppler and the static-world consistency test (static targets must show v = −v_sensor · u(az); approaching = negative on our Doppler axis) |
| `data/poses/<trace>/` | `lidar.npz`, `radar.npz`: 4×4 poses per scan / per radar frame, world = first scan (not in git) |
| `GRT.md` | proposal and first results for GRT occupancy as a reconstruction input, CFAR vs GRT |
| `tests/` | each stage against `xwr.rsp` on the same frame (`python -m pytest tests/ -q`) |
| `src/ext/<X-id>/` | extensions (see CONTRIBUTING.md) |
| `figures/` | report figures, named by trace and frame |
| `EXPERIMENTS.md`, `experiments/<id>/` | index of all experiments; one report per experiment (template: `experiments/_TEMPLATE.md`) |
| `data/raw/<trace>/` | captures, **not in git** |

## Setup
```bash
uv venv -p 3.12 && source .venv/bin/activate
uv pip install "roverd[video] @ git+https://github.com/RadarML/red-rover.git#subdirectory=format" xwr tqdm matplotlib scipy pyyaml pytest
```
Without `uv` (e.g. on nunga, where the system Python is PEP 668 managed): `python3 -m venv .venv && .venv/bin/pip install ...` with the same packages.

Put the captures under `data/raw/` with their original trace names (e.g. `data/raw/2026-10-21.15-58-19/`); symlinks are fine (on nunga they point into `/data/christineh/DATASET/radarhd/`).

```bash
python -m pytest tests/ -q                                                   # stages vs xwr.rsp
python -m src.radar data/raw/2026-10-29.15-49-40 --time 720 --accumulate 20  # point cloud of one second
```

## Calibration (extrinsics)

Radar, LiDAR and camera are rigid on the rig; the fixed transforms between them come from the eight board placements of 2026-10-21 (`calib_01`…`08`, 10 s each). Full method, numbers and figures: [experiments/m4-extrinsics](experiments/m4-extrinsics/README.md).

| Transform | Method | Result | Error |
|---|---|---|---|
| radar → LiDAR | joint solve (LiDAR as world, range / azimuth / pixel residuals weighted by their noise) | yaw +94.4°, t = (−0.15, +0.02, −0.27) m | ±1.5°, ±(5, 2, 15) cm; range residual 2.9 cm |
| LiDAR → camera | same solve | `calib/extrinsics_joint.json` | ±3°, ±10–14 cm; 23 px |
| radar → camera | composed from the two above | `calib/extrinsics_joint.json` | consistent by construction |

Pairwise solves (Kabsch 2D, PnP) are kept in `calib/` for comparison; the joint one is the one to use. Pitch and roll of the radar are fixed at 0 (unobservable without radar elevation). Limits come from the flat board, not the solver; a corner reflector session would bring them down.

```python
from src.calib.transform import RadarToLidar, RadarToCamera
T = RadarToLidar.load_joint();  xyz_lidar = T(xyz_radar)     # radar frame -> LiDAR frame (x right, y forward, z up -> Ouster sensor frame)
C = RadarToCamera.load_joint(); uv = C.pixels(xyz_radar)     # radar frame -> pixels in the 1920x1080 fisheye frame
```

```bash
python -m src.calib.radar_lidar                                  # pairwise radar->LiDAR (Kabsch)
python -m src.calib.board_image && python -m src.calib.radar_camera   # board centres in the images, pairwise PnP
python -m src.calib.joint                                        # joint solve -> calib/extrinsics_joint.json
python -m src.calib.overlay  data/raw/<trace> --time <s> --joint --out x.png   # radar points on the LiDAR scan
python -m src.calib.compare  data/raw/<trace> --time <s> --out x.png           # one frame, four calibrations
python -m src.calib.examples --wide --pick 0 3 5 --out x.png                   # chosen scenes, none vs joint
```
Rerun the whole chain after any change to the rig.
