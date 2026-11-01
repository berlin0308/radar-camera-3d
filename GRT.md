# GRT.md — Proposal: learned radar occupancy (GRT) as an input to 3D reconstruction, side by side with the classical CFAR chain

Status: draft, 2026-11-01. Facts in sections 1–2 were read from the GRT and NRDK source and the red-rover docs (commits listed at the end) and cross-checked against the paper by a separate literature pass (22 sources fetched, 25 claims checked by three independent readers each, 19 confirmed); nothing there is from memory. Section 7 lists what is still unverified, section 8 what the literature pass could not cover.

## 0. In one paragraph

GRT (Generalizable Radar Transformer; Huang, Prabhakara, Chen, Karhade, Ramanan, O'Toole, Rowe, "Towards Foundational Models for Single-Chip Radar", ICCV 2025, [arXiv 2509.12482](https://arxiv.org/abs/2509.12482); trained on I/Q-1M) takes one frame of **uncalibrated radar data** from exactly our radar (AWR1843Boost, 3 TX × 4 RX, 64 chirps × 256 samples, the `grt-o` chirp we record with): the raw I/Q is turned into a dense 4D range-Doppler-azimuth-elevation spectrum inside the model's data pipeline, no CFAR, no calibration. It outputs a **3D occupancy grid in the LiDAR's own spherical coordinates**: 64 elevation × 128 azimuth × 64 range cells, front 180°, ±45°, 0.35 m range bins to 22.4 m. The labels it was trained on are the Ouster range image itself, binned by radar range resolution with no radar→LiDAR transform applied anywhere, so the rig's radar→LiDAR offset is absorbed by the network; the output is already "in the LiDAR frame" of the red-rover rig. That is the key fact for integration: **GRT output needs no extrinsic calibration from us, only a LiDAR pose per frame**, which is the same pose Yining needs for everything else. Our classical chain produces sparse CFAR points in the radar frame and needs our own radar→LiDAR calibration (done, M4) to land in the same frame. Both then go through one identical accumulation and meshing path, and both are scored against the LiDAR itself. The comparison is therefore clean: same poses, same reference, same reconstruction code, two radar front-ends.

## 1. What GRT is and what it outputs (from the code)

| Item | Fact | Source |
|---|---|---|
| Input | one radar frame, raw I/Q `[64 chirp, 3 TX, 4 RX, 256 samples]`, 4D-FFT'd by `xwr.rsp.numpy.AWR1843Boost` (no window, no padding) into a dense `(256 range, 64 Doppler, 8 azimuth, 2 elevation)` complex cube, fed as two channels (amplitude as `sqrt`, scaled 1e-6, and phase angle) and patched along range and Doppler into tokens. Not a CFAR point cloud (CFAR appears only as an ablation input in the research repo) | paper §Radar Processing, App. A.1; `nrdk/grt/config/transforms/radar.yaml`; `grt/config/grt/grt.yaml` |
| Antenna layout requirement | AWR1843Boost 2 × 8 virtual array (the old repo asserts this with `AssertTx2`); AWR1843AOP data cannot be fed | `grt/deepradar/transforms/fft.py`, capstone notes on Tartan |
| Chirp | `grt-o` outdoor: 33.506 MHz/µs, 256 samples at 5 Msps, 64 loops, 20 Hz → Δr 8.74 cm, 22.4 m, ±1.79 m/s. **Our `radarhd-o-polin` config is the same chirp** (checked field by field, section 7) | `red-rover/collect/config/grt-o.yaml`, our `config.yaml` |
| Base model output | 3D polar occupancy logits, shape `[64 el, 128 az, 64 range]` (the paper writes it 64 range × 128 azimuth × 64 elevation; nrdk stores `el, az, range`). Loss: BCE on "polar grid cells created by the product set of the radar's range resolution with the Lidar's azimuth and elevation resolution", range-weighted | paper §4.2, App. B.3; `nrdk/grt/config/model/decoder/lidar3d.yaml` (`shape: [1, 64, 128, 64]`) |
| How the label is built | Ouster range image `[128 beams, 2048 columns]` → destagger → keep the middle half of the columns (`crop_az 0.25` on each side = the front 180° of the sensor frame) → range → bin index with `bin = range // (4 × radar range_resolution)` = 0.35 m → decimate (2, 8, 4) → `[64, 128, 64]` boolean; every LiDAR return sets one cell; bin 0 cleared | `grt/deepradar/transforms/lidar.py::Map3D`, `nrdk/src/nrdk/roverd/lidar.py::Occupancy3D` |
| Angular axes | not uniform: cell (i, j) covers Ouster beams 2i..2i+1 and encoder columns 512+8j..512+8j+7. The repo's `polar3_to_bev` approximates them as linspace over ±45° / ±90° | `Map3D`, `deepradar/utils.py` |
| Range axis | 64 bins × 0.3495 m = 22.4 m (4 × radar Δr; distances are normalised by the radar range resolution, so the grid is fixed-size in range-bin units: 17.6 cm cells indoor, 34.8 cm outdoor), index 0 unused | `Map3D`; paper App. B.3 |
| Other heads | `occ2d`: 256 × 1024 range-azimuth polar BEV occupancy (native radar range × native LiDAR azimuth, front-facing); `semseg`: 640 × 640 azimuth-elevation, 8 classes (ceiling, flat, nature, object, person, sky, structure, vehicle) from SegFormer-b5 pseudo-labels; `vel`: 3-vector ego-velocity in the sensor's orientation, Cartographer ground truth. Depth is **not** a head: it is the first occupied cell along range | paper App. B.3; `nrdk/docs/grt/index.md`; `osn.md` |
| Checkpoints | OSN bucket `cmu-wiselab-iq1m/checkpoints/{base,occ2d,semseg,vel}` (2025-11-22) | `osn.md` |
| Reference code | `wiselabcmu/grt` is frozen research code; `RadarML/nrdk` + `nrdk/grt/` is the maintained reference. The capstone repo already runs it as a submodule pinned at nrdk `@682b4fb` | README of each; capstone plan 9/23 |
| Metrics in the paper | Chamfer distance in radar range-bin units (8.7 cm per bin outdoor) on the occupancy, and depth error from the first occupied cell. Two Chamfer definitions exist across nrdk versions (first-return depth map ≈ 1.6 m in the paper vs full occupancy point cloud ≈ 0.6 m in the released eval files); the capstone uses the pinned version | paper App. B.3; capstone plan 9/24 |
| Poses in I/Q-1M | Cartographer 3D (ROS, IMU + LiDAR) with the rig URDF, `published_frame = radar`, so `_slam/trajectory.csv` is the radar's pose via the URDF joint; `roverp sensorpose` interpolates it to each sensor's timestamps (`_radar/pose.npz`, `_lidar/pose.npz`) with no per-sensor extrinsic applied | `roverp/_cli/sensorpose.py`, `cart_3d.lua`, `docs/iq1m/index.md` |
| Sensor registration in the paper | temporal only: one system clock, each sensor at its native rate, radar (20 Hz) and camera (30 Hz) matched to the 10 Hz LiDAR by nearest sample in time. The words "extrinsic" and "intrinsic" do not occur; "calibration" only in a cited title | paper App. A.2 (full-text search of arXiv HTML, camera-ready and supplement) |

## 2. Frames and calibration: what is aligned and what is not

**GRT output frame = the LiDAR sensor frame of the training rig.** The label is the LiDAR range image, so the network maps radar I/Q (radar frame, uncalibrated) to occupancy expressed from the LiDAR's position, looking along the LiDAR's forward direction (the middle of the encoder sweep, which on the red-rover URDF is `base_link +x`, i.e. the Ouster `os_lidar` frame's −x; on our rig the joint calibration found the same: radar forward ≈ LiDAR −x within ~10°). The radar→LiDAR offset of the rig is learned, not supplied.

**So the input is uncalibrated and the output is already aligned, as long as our rig is the training rig.** It is the same red-rover design (Tianshu, 10/1: "same AWR1843 layout as I/Q-1M, so the GRT checkpoint runs unchanged"). Three things can still differ and must be measured, not assumed:

1. Mounting twist and offset. Three published numbers for the radar's position relative to the LiDAR disagree: the URDF joint encodes 6 mm ahead, 11 mm below the plate centre, zero rotation; its own comment says "60 mm in front and 110 mm below" (every other joint in the file is consistent at mm/1000, so the radar joint is the odd one out); and the nrdk `Occupancy2D` docstring says "the radar is at approximately z = −0.15" relative to the LiDAR (110 mm + 22 mm plate + 36 mm lidar-to-sensor ≈ 0.17 m, consistent with the comment, not with the value). Nothing in the paper, the dataset page, red-rover or nrdk is a measured calibration. Our joint solve gives yaw +99.8° ± 2° where exact alignment would be +90°, and t = (−0.11, +0.04, −0.50 ± 0.21) m; the t_z is 1.6σ from the nominal −0.15 to −0.17 m and should be treated as unreliable until a tape measure settles it (open item in the M4 report). A 10° twist would be visible in GRT output as a rotation relative to our LiDAR; a 0.15 m drop is below one range cell and would not.
2. The LiDAR swap (OS-0-64 → OS-0-128 on 10/29). GRT output is defined on 128-beam geometry; evaluation against our 64-beam 10/21 scans needs the beam tables handled, against 10/29 scans it is direct.
3. Timing. Radar starts ~2.2 s after the camera; all sensors share one clock, so match by timestamp.

**Alignment plan (cheap, decisive):** run GRT on 10/29 static-ish moments, build our own `Occupancy3D` label from the simultaneous LiDAR scan with the same transform, and compare the two grids per azimuth column. Sweep a residual yaw of ±15° (shift the azimuth axis by k cells, 1.4° each) and a range offset of ±2 bins; take the maximum IoU / minimum Chamfer. If the residual is within one cell, GRT output is used as is; if not, the residual is one yaw and one range offset applied to GRT cells before accumulation, and it is reported. This also settles the URDF-vs-joint-solve disagreement independently of the board calibration.

**What our own calibration is used for:** only the classical chain. CFAR points live in the radar frame and go through `RadarToLidar.load_joint()` into the LiDAR frame. Nothing from M4 touches GRT output.

## 3. How to feed GRT occupancy into 3D reconstruction

Two routes, both ending in the same accumulation code so that CFAR and GRT are treated identically.

```
radar I/Q ──► GRT base ──► occ3d logits [64,128,64] (LiDAR frame)  ──┐
                                                                     ├──► per-frame samples in the LiDAR frame
radar I/Q ──► src.radar (CFAR) ──► points (r, az, snr) ──► T_joint ──┘
                                                                     │
LiDAR scans ──► LiDAR SLAM (KISS-ICP) ──► pose per scan ──► interpolate to radar time
                                                                     │
                                     world-frame accumulation: (A) points + Poisson/TSDF, (B) log-odds voxel grid
                                                                     │
                                     reference: LiDAR scans accumulated with the same poses, same voxel size
```

### 3.1 Per-frame conversion to the LiDAR frame

- **GRT:** sigmoid the logits → p(el, az, r). Convert a cell to a point with the real Ouster angles, not the linspace approximation: elevation = mean of `beam_altitude_angles[2i:2i+2]`, azimuth = encoder angle of columns 512+8j..519+8j (minus `beam_azimuth_angles`), range = (r + 0.5) × 0.3495 m. This reuses the same geometry roverd uses for LiDAR xyz, so GRT cells and LiDAR points share one convention. Carry p with each point.
- **CFAR:** `detect()` → `to_xyz()` → `RadarToLidar.load_joint()`; z is set to 0 before the transform (radar elevation is unusable), or to the LiDAR-estimated height of the nearest LiDAR return if we want a fairer 3D comparison (state which in the report). Carry SNR with each point.
- **Reference:** LiDAR xyz from `src/calib/lidar.py` (or roverd), ground removed by height, same frames.

### 3.2 Poses

KISS-ICP on the 10/29 Ouster scans (10 Hz, 128 beams; pip-installable, no ROS). Output: pose per scan. Interpolate to radar timestamps with a spline + smoothing (what `roverp sensorpose` does; reuse it if we write `trajectory.csv` in Cartographer's column format, otherwise 30 lines of scipy). IMU `rot` is the sanity check on orientation. Orbits are slow (0.5–1.4 m/s) so a 50 ms radar frame moves < 7 cm; no per-chirp motion correction is needed at the 0.35 m cell size.

### 3.3 Accumulation (the part that is identical for both inputs)

- **(A) Point route:** for each frame, keep GRT cells with p > τ (or CFAR points with SNR > s), transform to world, concatenate over the segment, voxel-downsample (0.1 m), then Open3D Poisson or a TSDF (`ScalableTSDFVolume`) if a mesh is wanted. Simple, honest, and the point count per frame is the knob.
- **(B) Probabilistic route (the right one for GRT):** a world voxel grid (0.2 m) with log-odds; for each frame add `logit(p)` along the cell's ray for GRT, and a fixed hit log-odds for a CFAR point (with free-space decrement along the ray in both cases, optional). Threshold at the end. This is OctoMap-style fusion and is where GRT's probabilities, rather than a hard threshold, pay off. Marching cubes on the grid gives a mesh.

Both routes take the same poses, voxel size and segment, so the only difference between the CFAR row and the GRT row of the results table is the radar front-end.

## 4. Experiment: CFAR vs GRT as reconstruction inputs

**Scenes (all 10/29, 128-beam LiDAR, GRT-compatible chirp):** Jeep orbit (`15-49-40` 11–17 min, full loop, large metal), Walking to the Sky pole (`16-12-38` 5:30–9:40, thin metal, tilted rig), Scotty (`16-22-36` 2:59–3:45, partial, flat wall behind → multipath), planters (`16-22-36` ~2:00, `16-27-55` ~0:00 / 1:30). One static sanity scene: `scene_room` 10/21 (64-beam; alignment check only).

**Inputs:** R-CFAR (ours), R-GRT-3D (base checkpoint), optionally R-GRT-2D (`occ2d`, BEV only, for the capstone's BEV metric) and R-GRT-3D with the `semseg` head to drop `person` cells (moving people are the main non-static clutter).

**Reference:** LiDAR accumulated with the same poses, cropped to the radar's field of view (front 180°, ±45°, 22.4 m), ground removed. Evaluation is within the object's bounding region (hand-drawn once per scene) and over the whole segment.

**Metrics (all computed by one script, on the world-frame output):**
- Chamfer distance (m), both directions reported separately: accuracy (output → LiDAR) and completeness (LiDAR → output). Chamfer is what both precedents report: RadarHD gives medians (0.36 m Chamfer, 0.24 m modified Hausdorff in a seen environment, 2.7× / 3.5× better than CA-CFAR at 1–8 dB thresholds; 0.8–1.0 m in unseen environments), GRT gives Chamfer in range-bin units. Report the modified Hausdorff too so the RadarHD numbers are comparable.
- Precision / recall / F-score at 0.5 m (the capstone's F@0.5 m) and at 0.2 m.
- Voxel IoU at 0.2 m after the log-odds route.
- Points-per-frame vs F-score curve: sweep τ for GRT and Pfa for CFAR, so the comparison is a curve, not one operating point. A single "anchor" operating point for the tables: equal occupied-cell count per frame (the capstone's R1 rule).
- Per-object: extent error (bounding box of the reconstruction vs LiDAR) and whether the object is visible from every side of the orbit (the obj-orbit-360 question).

**Ablations:** frames accumulated (1, 10, 100, whole orbit); z handling for CFAR (z = 0 vs LiDAR height); GRT with and without the residual yaw correction from section 2; route A vs B.

**Qualitative figures:** per scene, four panels in the world frame, top view and one oblique view: LiDAR reference, CFAR accumulation, GRT accumulation, GRT log-odds mesh. Plus the single-frame comparison GRT occupancy vs LiDAR occupancy label (same grid) as a 2 × 2 image, which is the clearest explanation of what GRT does.

**Expected outcome (to be tested, not assumed):** GRT gives 10–100× more cells per frame and fills surfaces (walls, car sides) that CFAR only samples; CFAR has sharper range on strong reflectors and no learned prior, so it should be better on thin metal (the pole) and worse on everything else; both share the radar's multipath ghost behind the Scotty wall unless the semseg / velocity heads or the log-odds free-space term remove it.

## 5. Who does what, and when

| Piece | Owner | Status | Note |
|---|---|---|---|
| GRT inference on our traces → `occ3d` per frame as an lzmaf channel in a `_grt/` folder next to each trace | Po-Lin (capstone side already has nrdk + checkpoints on the server) | to do | `evaluate.py --trace` in `nrdk/grt` writes metrics; a small script that saves the logits is needed (`Result` + dataloader, ~40 lines) |
| Occupancy cell → xyz with real beam angles; alignment check (section 2) | Po-Lin | to do | `src/grt/occupancy.py`, `src/grt/align.py` |
| CFAR points → LiDAR frame | done | `src.radar` + `RadarToLidar.load_joint()` |  |
| LiDAR SLAM poses, interpolation to radar time | Yining | to do | KISS-ICP; `_lidar/pose.npz`, `_radar/pose.npz` in roverp's format so both projects read the same file |
| Accumulation routes A and B, meshing, viewer | Yining | to do | `src/ext/360/` |
| Metrics script (Chamfer / F@0.5 / IoU / curves) | shared; Po-Lin writes the first version against `scene_room` | to do | `src/eval/recon_metrics.py` |
| Figures and the comparison table | both | — | `experiments/obj-orbit-360/`, new `experiments/grt-vs-cfar/` |

Timeline against the course: proposal 11/20 lists this as a *hope to achieve* (the plan-to-achieve stays CFAR + dense depth + LiDAR evaluation); alignment check and single-frame GRT-vs-LiDAR figure by 11/15 (they also feed M4/M6); accumulation and the first table by 11/30 together with M6; nothing new after 12/1.

## 6. Why this is worth doing for both projects

For the course project it is the "learned vs classical" comparison with the same hardware and the same ground truth, which is exactly the question the proposal's background section raises (classical pixel-level fusion vs learned radar). For the capstone it is a second use of the same GRT inference and the same poses, and it gives Yining's reconstruction a probabilistic input instead of 12 points per frame. The code boundary stays clean: GRT and its checkpoints are external (declared in Resources), inference is run with the maintained reference implementation, and everything after the logits is ours.

## 7. Related work: what is verified, what is a lead

**Verified (primary sources read, claims checked 3-0):**
- *RadarHD* (Prabhakara et al., "High Resolution Point Clouds from mmWave Radar", ICRA 2023, [arXiv 2206.09273](https://arxiv.org/abs/2206.09273)) is the direct precedent for "learned radar → point cloud → SLAM". Input is a low-thresholded 2D range-azimuth heatmap (64 × 256, 0–10 m, ±90°, log-magnitude) with 40 past frames stacked as channels; output a 512 × 256 LiDAR-resolution range-azimuth sigmoid image trained with BCE against binary LiDAR images, thresholded into (x, y) points. Strictly 2D (LiDAR ground truth limited to a ±30 cm band). Mapping: the learned per-frame clouds are fed, with no IMU, into Google Cartographer 2D for a 3-DoF pose and map; map quality is scored by Euclidean error between hand-matched keypoints (room corners), odometry by ATE against LiDAR and CFAR. It did **not** use the red-rover sensor pair (a claim to that effect was refuted), so its numbers are a reference for the metric, not for our hardware.
- *GRT* itself evaluates with Chamfer in range-bin units and first-occupied-cell depth error (section 1). No Cartesian or world-frame conversion of its output exists in the paper or either repo; the angular extent (front 180° × 90°) follows from the label being the LiDAR's own forward-facing bins.

**Leads, not verified in this pass (names only; no numbers or claims should be cited from them until read):** RadarOcc (3D occupancy from 4D imaging radar, [arXiv 2405.14014](https://arxiv.org/abs/2405.14014), fetched but no claim survived verification), DART (WiSE Lab Doppler-aided radar tomography), RadarFields, radar Gaussian splatting, radar SLAM with learned clouds; for fusion, OctoMap-style log-odds voxel maps (natural for a probabilistic polar grid) vs Open3D TSDF integration (takes posed depth images, so GRT's first-occupied-cell depth could be integrated once rendered to a projection) vs Poisson on accumulated points; for poses, KISS-ICP (LiDAR-only), FAST-LIO2 (LiDAR-inertial) and the Ouster SDK's own mapping CLI; for metrics, Tanks-and-Temples precision/recall/F-score at a distance threshold, DTU accuracy/completeness, voxel IoU. These are the standard tools and are what section 3 assumes; each needs its primary source read before the report cites it. Capstone side: the Chamfer definition is pinned to nrdk `@682b4fb` and matches the authors' released eval files.

## 8. Open points and what is unverified

- [x] Field-by-field check of our `radarhd-o-polin` chirp against `grt-o` (done 11/1): identical (77 GHz, 33.506 MHz/µs, 256 samples at 5 Msps, 64 loops, 50 ms, idle 120 µs, ramp 59 µs, ADC start 5.7 µs); only the key spelling differs (`num_tx: 3` vs `device: AWR1843`). LiDAR identical too (128 beams, `2048x10`).
- [ ] Whether the released `base` checkpoint is the `small` size and which nrdk version its config expects (`@682b4fb` is what the capstone pinned); confirm it loads under the maintained `nrdk/grt` `evaluate.py`. Generalisation of the checkpoint across chirp configs is unestablished in the paper, but moot for us: our chirp is `grt-o`.
- [ ] Radar-to-LiDAR offset on our unit: URDF value 6 / 11 mm, URDF comment 60 / 110 mm, nrdk docstring ≈ 0.15 m below, our joint solve 0.50 ± 0.21 m below. Tape-measure the rig (radar array centre to LiDAR optical centre) and ask Tianshu; whether our physical rig is the same unit and the same URDF geometry as the I/Q-1M rig is also unverified. This matters for the CFAR row, not for GRT.
- [ ] The exact angle of GRT azimuth column 0 and the beam-row pairing after decimation (section 3.1 derives them from `Map3D`: columns 512..1535 of the destaggered image in groups of 8, beams in pairs); confirm on one frame by overlaying a GRT cell map on our LiDAR scan.
- [ ] Residual yaw of GRT output on our rig (section 2 sweep); expected within ±2 cells.
- [ ] Which Chamfer definition to report (pinned nrdk `PolarChamfer3D` on the grid, vs the world-frame Chamfer of section 4); report both, say which is which.
- [ ] GRT on walking data: trained on handheld / bike data at 1.0–5.4 m/s, so our 0.5–1.4 m/s orbits are inside its distribution; the tilted-up pole segment is not (BEV guidance in the red-rover tips says keep roll level).
- [ ] Whether `semseg` or `vel` heads help remove people / multipath; optional.
- [ ] I/Q-1M has no published camera–LiDAR calibration (capstone 9/28–29); nothing here depends on it, but the dense-depth stage (M6) does, and that is our own M4 result.

## 9. Commands (what exists today)

```bash
# classical chain, points in the LiDAR frame
python -m src.radar data/raw/2026-10-29.15-49-40 --time 720 --accumulate 20
python -c "from src.calib.transform import RadarToLidar; T = RadarToLidar.load_joint()"   # xyz_lidar = T(xyz_radar)

# GRT (capstone server, nrdk/grt checkout with the base checkpoint as a Result directory)
uv run python evaluate.py <results_dir> --data-root <root with our traces> --trace 2026-10-29.15-49-40 --output <out>
# to be written: dump occ3d logits per frame (src/grt/infer.py), cells -> xyz (src/grt/occupancy.py), alignment sweep (src/grt/align.py)
```

Sources read locally: `wiselabcmu/grt` @4a04614 (`deepradar/transforms/lidar.py`, `deepradar/objectives/occupancy3.py`, `deepradar/utils.py`, `config/obj/map.yaml`, `config/grt/grt.yaml`, `demo/`), `RadarML/nrdk` @d606113 (`grt/config/**`, `grt/evaluate.py`, `docs/grt/*.md`, `src/nrdk/roverd/lidar.py`, `src/nrdk/metrics/pointcloud.py`), `RadarML/red-rover` @6687094 (`docs/iq1m/*.md`, `docs/roverc/tips.md`, `collect/config/grt-o.yaml`, `processing/catkin_ws/src/slam/red-rover.urdf`, `processing/src/roverp/_cli/sensorpose.py`, `format/src/roverd/transforms/ouster.py`), and the capstone plan page (Notion, entries of 9/23–10/17).

Sources from the literature pass (primary, fetched 2026-10-31): [GRT paper, arXiv 2509.12482](https://arxiv.org/abs/2509.12482) and its [HTML](https://arxiv.org/html/2509.12482v1); [NRDK GRT reference](https://radarml.github.io/nrdk/grt); [I/Q-1M page](https://radarml.github.io/red-rover/iq1m/); [roverp CLI docs](https://radarml.github.io/red-rover/roverp/cli/); [nrdk `roverd/lidar.py`](https://github.com/RadarML/nrdk/blob/main/src/nrdk/roverd/lidar.py); [red-rover URDF](https://github.com/RadarML/red-rover/blob/main/processing/catkin_ws/src/slam/red-rover.urdf); [red-rover `iq1m/calibrate.py`](https://github.com/RadarML/red-rover/blob/main/iq1m/calibrate.py); [RadarHD, arXiv 2206.09273](https://arxiv.org/html/2206.09273) and [code](https://github.com/akarsh-prabhakara/RadarHD).
