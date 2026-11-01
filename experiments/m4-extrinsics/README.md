# M4: radar to LiDAR and radar to camera extrinsics

**Status:** Radar→LiDAR solved in 2D and radar→camera solved by PnP (2026-11-01, both preliminary)

## Question
Where is the radar relative to the LiDAR and the camera, and how far do transformed radar points land from the target?

## Method
8 calibration placements of the board, 10 s each, rig static. Radar→LiDAR (`src/calib/radar_lidar.py`): radar board = median of the strongest CFAR point over 40 frames, range minus the 0.183 m offset; LiDAR board = largest upright cluster (floor removed, ≥ 25 cm tall) within 1 m of where a prior (LiDAR −x forward) predicts it, re-picked within 0.5 m after a first solve; 2D rigid fit (yaw, tx, ty) by Kabsch. Pitch, roll and tz are not estimated because radar elevation is unusable. Radar→camera (`src/calib/board_image.py`, `radar_camera.py`): board centre in each camera frame found as a grey, smooth, square blob (three shots automatically, five grown from a hand seed inside a range-sized window); 3D board point = radar x, y plus the LiDAR board height (a constant z offset is absorbed by the translation); pixels undistorted with the red-rover OPENCV_FISHEYE intrinsics (`calib/camera_fisheye_iq1m.json`), `cv2.solvePnP` from the co-located guess; reprojection error with the fisheye model.

## Data
`calib_01`, `calib_02`, `calib_03b`, `calib_04`, `calib_05b`, `calib_06`, `calib_07`, `calib_08` (2026-10-21). Checks: `scene_room` (10/21), `2026-10-29.16-22-36` t = 200 s, `2026-10-29.15-49-40` t = 720 s.

## Results

### Radar → LiDAR
| Quantity | Value |
|---|---|
| yaw (radar → lidar) | +101.9° |
| t (x, y) in the lidar frame | (−0.114, +0.157) m |
| shots used | 7 (calib_06 skipped: board at 0.25 m, inside the radar leakage zone) |
| residual per shot | 01 16.1 cm · 02 6.3 · 03b 11.0 · 04 3.0 · 05b 11.2 · 07 1.0 · 08 11.2; mean 8.5 cm |

Check on traces not used in the fit (nearest above-ground LiDAR point to each transformed radar point, xy): room scene 10/21 median 11 cm, 68 % within 0.5 m; Scotty segment 10/29 (after the swap to the OS-0-128) median 22 cm, 60 %; parking lot 10/29 median 145 cm, 29 % (the radar sees cars at 15 m where this scan has almost no returns, and the rig is walking; the Jeep itself lines up).

![Board found in LiDAR and the radar detection after the transform, per shot](../../figures/calib_radar_lidar_shots.png)

*Board found in LiDAR (orange) and the radar detection after the transform (blue x), per shot. `python -m src.calib.radar_lidar`, 2026-11-01.*

![Before and after the transform, residual per shot](../../figures/calib_radar_lidar_solve.png)

*Before and after the transform, residual per shot.*

![Check on the Scotty segment of 10/29](../../figures/calib_overlay_scotty.png)

*Check on the Scotty segment of 10/29: radar points on the LiDAR walls. `python -m src.calib.overlay data/raw/2026-10-29.16-22-36 --time 200 --accumulate 10`.*

### Radar → camera
| Placement | Range (offset removed) | Azimuth (+ left) | Reprojection error, co-located guess | After PnP |
|---|---|---|---|---|
| 01 | 3.05 m | −14.5° | 156 px | 59 px |
| 02 | 2.18 m | +18.2° | 192 px | 16 px |
| 03b | 1.48 m | −5.4° | 226 px | 27 px |
| 04 | 1.74 m | −5.4° | 168 px | 27 px |
| 05b | 2.96 m | −14.5° | 131 px | 37 px |
| 06 | 0.25 m | — | not used (radar leakage zone; board fills the frame) | — |
| 07 | 2.53 m | −25.9° | 108 px | 36 px |
| 08 | 3.75 m | +10.8° | 22 px | 43 px |

Worked example, the point pairs PnP was given (all in `calib/extrinsics_radar_camera.json`):

| Shot | 3D point, radar frame (x, y, z) m | Board centre (u, v) px | Projected, co-located guess | Projected, after PnP |
|---|---|---|---|---|
| calib_01 | (+0.76, +2.95, −0.65) | (1064, 671) | (1202, 743), 155 px | (1113, 705), 59 px |
| calib_02 | (−0.68, +2.07, −0.73) | (660, 667) | (653, 859), 192 px | (652, 681), 16 px |
| calib_03b | (+0.14, +1.47, −0.66) | (964, 737) | (1043, 949), 226 px | (971, 711), 27 px |
| calib_04 | (+0.16, +1.73, −0.21) | (1004, 491) | (1048, 653), 168 px | (997, 517), 27 px |
| calib_05b | (+0.74, +2.87, −0.65) | (1076, 715) | (1202, 749), 131 px | (1112, 706), 37 px |
| calib_07 | (+1.10, +2.27, −0.68) | (1302, 747) | (1394, 804), 107 px | (1267, 738), 36 px |
| calib_08 | (−0.70, +3.68, −0.61) | (776, 672) | (771, 694), 22 px | (744, 643), 43 px |

| Extrinsics (radar → camera) | Before (co-located guess) | After |
|---|---|---|
| Rotation (Rodrigues, deg, camera axes) | 0, 0, 0 | −7.2, −2.7, +7.6 |
| Translation (m, camera axes) | 0, 0, 0 | (+0.06, −0.48, +0.64) |
| Mean reprojection error (px) | 143 | 35 (max 59) |

35 px at 3 m with a 987 px focal length is 2°, the same size as the radar's azimuth error on a flat board (section M1), so the fit is azimuth-limited. A LiDAR→camera PnP on the same pixel centres with the LiDAR board centroids as 3D points reaches only 29 px (`calib/extrinsics_lidar_camera.json`), so the image-side centres (blob centroids of a flat board) are the next limit; composing radar→LiDAR→camera and comparing with the direct solve gives a rotation difference of 2.2°, the loop closing within the radar's bearing error. The translation is weakly constrained by seven points with degree-level bearing noise and should not be read as the physical offset; the rotation is the usable part. The other intrinsics file in red-rover (`camera_fisheye_notebooks.json`) gives the same answer to within 1° and 4 cm.

![Board in the camera frames with the radar detection projected under the co-located guess](../../figures/calib_board_in_images.png)

*Board found in each camera frame (orange) and the radar detection projected with no extrinsics (blue x). `python -m src.calib.board_image`.*

![Radar detections before and after PnP](../../figures/calib_radar_camera.png)

*Radar detections projected into the camera before (blue x) and after (green o) PnP, crops around the board. `python -m src.calib.radar_camera`.*

On real scenes (points at radar height, z = 0, since elevation is unusable): the co-located guess puts every radar point on one horizontal line; the calibrated projection moves them by the solved yaw and roll, so the Jeep's returns sit on the Jeep and the 15 m returns on the car row behind it. Vertical placement is not calibrated and should not be read.

![Radar points on the Jeep frame, before and after](../../figures/calib_project_jeep.png)

*Radar points (10 frames, SNR > 12 dB) on the parking-lot frame, co-located guess vs calibrated. `python -m src.calib.project_image data/raw/2026-10-29.15-49-40 --time 720 --accumulate 10`.*

![Radar points on the Scotty frame, before and after](../../figures/calib_project_scotty.png)

*Same on the Scotty segment: returns from the wall behind the statue and from the planters to the right.*

![Preview: radar detections projected with no extrinsics; all 8 land on or at the edge of the board](../../notes/fig/rd2/f10_calib_projection.png)

*Preview: radar detections projected with no extrinsics; all 8 land on or at the edge of the board*
![Pending: same 8 images after calibration, with the target centre marked and the residual drawn as a line](../_assets/placeholder.svg)

*Pending: same 8 images after calibration, with the target centre marked and the residual drawn as a line. Source: TBD.*
![Pending: reprojection error vs range and vs azimuth](../_assets/placeholder.svg)

*Pending: reprojection error vs range and vs azimuth. Source: TBD.*


### Joint solve, LiDAR frame as world (`src/calib/joint.py`)
One least-squares problem instead of pairwise chains: unknowns are the radar pose (yaw, tx, ty, tz; pitch and roll fixed at 0) and the camera pose (6) relative to the LiDAR, 10 in all; the 28 residuals are the seven shots' radar range (σ 2 cm), radar azimuth (σ 2°) and camera pixel (σ 20 px) against the LiDAR board point, radar elevation left out. Started from the pairwise answers, cost 95 → 31 (σ units).

| Quantity | Pairwise | Joint | 1σ (Jacobian) |
|---|---|---|---|
| radar→lidar yaw | +101.9° | +99.8° | ± 2.0° |
| radar→lidar t (x, y) | (−0.11, +0.16) m | (−0.11, +0.04) m | ± (0.06, 0.02) m |
| radar→lidar t_z | not estimated | −0.50 m | ± 0.21 m |
| lidar→camera t | (−0.15, −0.54, +0.28) m (PnP) | (−0.15, −0.54, +0.28) m | ± (0.12, 0.12, 0.19) m |
| radar range residual | 8.6 cm | 3.7 cm | — |
| radar azimuth residual | 1.6° | 1.7° | — |
| camera pixel residual | 29 px | 29 px | — |

What changed and what did not: the range residual halves because t_z is now a free parameter and the board heights (−0.2 to −0.7 m in the LiDAR frame) make it observable through range, so the radar comes out 0.50 ± 0.21 m below the LiDAR; azimuth and pixel residuals stay where the sensors put them, since no amount of joint fitting beats a flat board's bearing noise; and the radar→camera transform is now composed from the two LiDAR-referenced poses, so the loop closes by construction instead of to 2.2°. The 1σ column is the real product: with seven flat-board shots, yaw is known to 2°, in-plane offsets to a few cm, heights to 20 cm. `RadarToLidar.load_joint()` and `RadarToCamera.load_joint()` apply it. The IMU is not in the solve because it observes no target; it enters through hand-eye on the orbit segments (rotation between LiDAR scans against integrated gyro), which is the next piece.

![Residuals per shot, pairwise vs joint](../../figures/calib_joint_residuals.png)

*Residuals per shot, pairwise start (grey) vs joint solve (blue); the band is the assumed 1σ. `python -m src.calib.joint`.*

## Summary for the report

Quantitative, every method on the same seven 10/21 board placements; checks on scenes not used in any fit (nearest above-ground LiDAR return to each transformed radar point, xy median; room = 10/21 `scene_room`, Scotty = 10/29 `16-22-36` t = 200 s after the LiDAR swap). "—" means the method does not produce that quantity.

| Method | Unknowns | Radar range res. | Radar azimuth res. | Camera px res. | Loop closure | t_z radar→LiDAR | Check: room | Check: Scotty |
|---|---|---|---|---|---|---|---|---|
| None (co-located guess) | 0 | — | — | 143 px | — | — | — | — |
| Kabsch radar→LiDAR, 2D | 3 | 8.5 cm (2D) | — | — | — | not estimated | 11 cm, 68 % | 22 cm, 60 % |
| PnP radar→camera | 6 | — | — | 35 px | — | — | — | — |
| PnP LiDAR→camera, composed | 6 (+3) | 8.5 cm | — | 29 px | 2.2° vs direct | — | — | — |
| Joint, LiDAR frame, weighted | 10 | 3.7 cm | 1.7° | 29 px | by construction | −0.50 ± 0.21 m | 10 cm, 78 % | 29 cm, 61 % |

1σ from the joint solve: yaw ±2.0°, t_x ±6 cm, t_y ±2 cm, t_z ±21 cm; camera rotation ±4°, translation ±12–19 cm. The residuals that do not move between rows (azimuth, pixels) are the flat board's limit, not the solver's.

Qualitative, the same second of the parking-lot frame under each calibration:

![Same Jeep frame, four calibrations](../../figures/calib_compare_jeep.png)

*Radar points of one second (SNR > 12 dB, drawn at radar height) on `2026-10-29.15-49-40` t = 720 s: (a) no calibration, (b) pairwise PnP, (c) composed Kabsch + PnP, (d) joint solve. `python -m src.calib.compare data/raw/2026-10-29.15-49-40 --time 720 --accumulate 10`.*

More examples, no calibration vs the joint solve, chosen because the effect is easy to see: a single board off-axis (the radar point moves from the empty desk onto the board), a food truck, a car at a crosswalk, and the wall behind Scotty.

![Six scenes, no calibration vs joint](../../figures/calib_examples.png)

*Radar points (SNR > 15 dB, drawn at radar height) on six scenes: 10/21 board at 40° right, 20° left and 26° right; 10/29 food truck, crosswalk, Scotty. `python -m src.calib.examples`.*

## Conclusion
Radar→LiDAR: a 2D rigid transform explains the seven board placements to 8.5 cm on average and still holds on Session 2 data after the LiDAR swap. The residual is dominated by the radar's azimuth error on a flat board, not by range. Radar→camera: PnP on seven board centres brings the reprojection error from 143 to 35 px, which is the radar's own bearing error; the rotation (about 7° about two axes) is usable, the translation is not yet trustworthy.

## Open items
- Repeat with a corner reflector (apex visible in the image, strong point return) to bring both residuals down and pin the translation
- Pitch, roll, tz for radar→LiDAR: from the board height in LiDAR plus a tape measure, or after X9
- Close the loop: compose radar→LiDAR with a LiDAR→camera estimate and compare with radar→camera

## B-roll to capture (for the 5-minute video)
- [ ] Phone photo of the rig showing where radar and camera sit
- [ ] Overlay video: radar points on the camera feed, before vs after calibration

## Reproduce
```bash
python -m src.calib.radar_lidar          # calib/extrinsics_radar_lidar.json + the two figures
python -m src.calib.overlay data/raw/2026-10-29.16-22-36 --time 200 --accumulate 10 --out figures/calib_overlay_scotty.png
```
```bash
python -m src.calib.board_image          # board centres in the 8 camera frames -> calib/board_pixels.json
python -m src.calib.radar_camera         # PnP -> calib/extrinsics_radar_camera.json + figure
```
Use them: `from src.calib.transform import RadarToLidar, RadarToCamera; T = RadarToLidar.load(); xyz_lidar = T(xyz_radar); C = RadarToCamera.load(); uv = C.pixels(xyz_radar)`.

[Back to all experiments](../../EXPERIMENTS.md)
