# Radar–camera 3D camera (15-663 final project)

Classical FMCW radar (TI AWR1843 + DCA1000 on the WiSE Lab red-rover rig) plus a fisheye camera,
turned into a 3D camera: raw ADC → range / Doppler / angle → 3D points → camera-guided dense depth,
evaluated against LiDAR.

**[Experiment reports →](EXPERIMENTS.md)** every experiment with its numbers, figures and what is still pending.

**[Three Sensors, One Statue →](https://claude.ai/artifact/Cc611gCdM8FRype53aoxeC)** Yining's write-up of the Scotty 11/03 orbit: radar, LiDAR and camera reconstructions in one object frame, with interactive viewers (a Claude artifact, needs the share; the numbers match [experiments/obj-orbit-360](experiments/obj-orbit-360/README.md)).

Author: Po-Lin Chen. Contributor: Yining (extensions, 3D reconstruction; acknowledged in the report).

## Layout
| Path | What |
|---|---|
| `shoot.sh`, `sync.sh`, `validate.sh`, `check_valid.py` | rig-day recording and data checks |
| `SESSIONS.md` | one entry per rig session: rig state, traces, data quality, what each trace is for; the shot list for the next session on top |
| `AGENDA_session2.md` | rig-day procedure (start-up, recording, shut-down, copying) |
| `notes/shots.csv` | which trace is which shot |
| `notes/sessions/` | full log per session, and the script behind each session's numbers |
| `notes/handover-2026-11-08-berlin.md` | handover to Yining: where the GRT dumps are, their format, what the night trace holds |
| `notes/fig/` | figures used in the Notebook deck |
| `src/radar/` | own radar chain: `adc.py` → `range_doppler.py` → `angle.py` → `cfar.py` → `pointcloud.py`; `python -m src.radar <trace>` runs it on one frame |
| `src/calib/` | extrinsics: `plane_solve` (current rig, 11/3 board shots, radar on the board's specular point); for the 10/21 rig `radar_lidar` (Kabsch), `board_image` + `radar_camera` (PnP), `joint` (one weighted solve); `RadarToLidar` / `RadarToCamera` to apply them; `overlay`, `compare`, `examples` to look at the result |
| `calib/` | `extrinsics_joint.json` (current rig, use this), `extrinsics_joint_2026-10-21.json` (the rig before the LiDAR swap, picked with `load_joint(trace=...)`), `board_corners_2026-11-03.json`, the pairwise `extrinsics_radar_lidar.json` / `extrinsics_radar_camera.json` / `extrinsics_lidar_camera.json`, `board_pixels.json`, fisheye intrinsics from red-rover |
| `src/grt/` | the learned model next to the classical chain: `infer` runs a released GRT checkpoint on a trace (nrdk env, see `GRT.md`), `occupancy` turns grid cells into xyz with the real Ouster angles, `align` scores GRT against our LiDAR as first-return depth, `compare_frame` draws camera / CFAR / GRT / label for one instant |
| `src/recon/` | from frames to a world frame: `poses` (KISS-ICP on the Ouster scans, gyro-deskewed, poses at radar times), `ego_check` (radar Doppler of the static world against the odometry), `accumulate` (CFAR points with the static-world filter into the world frame next to the LiDAR built the same way, ground removed; `--dense` exports the range-azimuth power of every frame); `closeup` (one object up close: camera, LiDAR and radar, every scan re-centred on its pedestal) |
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

Radar, LiDAR and camera are rigid on the rig; the fixed transforms between them come from the eight board placements of 2026-11-03 on the current rig (OS-0-128, every trace from 10/29 on). Traces before the 10/29 LiDAR swap have their own solve. Full method, numbers and figures: [experiments/m4-extrinsics](experiments/m4-extrinsics/README.md).

| Transform | Method | Result | Error |
|---|---|---|---|
| radar → LiDAR | LiDAR plane of the board; radar range and azimuth on its specular point | yaw +92.75°, t = (−0.133, +0.012, −0.269) m | ±0.71°, ±(0.8 forward, 2.0 sideways) cm; range 1.6 cm, azimuth 0.58°; height held from 10/21, not measured |
| LiDAR → camera | PnP on the board's corners | `calib/extrinsics_joint.json` | ±0.3°, ±1 cm; 6.4 px |
| radar → camera | composed from the two above | `calib/extrinsics_joint.json` | consistent by construction |

Radar pitch and roll are fixed at 0 (unobservable without radar elevation). Nothing yet tests the solve beyond 3 m. `load_joint(trace=...)` picks the solve for a trace's rig. The pairwise solves in `calib/` belong to the 10/21 rig and are kept for comparison.

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
