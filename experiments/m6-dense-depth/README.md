# M6: camera-guided dense depth, evaluated against LiDAR

**Status:** Planned (due 11/30); first preview from rig day 1

## Question
How much of the camera's structure can be lent to sparse radar depth, and where does it break?

## Method
Project radar points into the fisheye image; fill with joint bilateral upsampling (normalized convolution) and an edge-aware weighted least-squares solver; compare with a monocular-depth baseline. Ground truth: 128-beam LiDAR projected into the camera.

## Data
`scene_room` (indoor, 64-beam LiDAR), Session 2 static scenes (128-beam), night repeats if recorded.

## Results
| Method | MAE 0-5 m | MAE 5-10 m | MAE 10-20 m | Day | Night |
|---|---|---|---|---|---|
| Radar points only (nearest) | TBD | TBD | TBD | TBD | TBD |
| Joint bilateral upsampling | TBD | TBD | TBD | TBD | TBD |
| Weighted least squares | TBD | TBD | TBD | TBD | TBD |
| Monocular depth (baseline) | TBD | TBD | TBD | TBD | TBD |

![Pending: four columns: camera, radar points, dense depth, LiDAR ground truth; one row per scene](../_assets/placeholder.svg)

*Pending: four columns: camera, radar points, dense depth, LiDAR ground truth; one row per scene. Source: TBD.*
![Pending: error map: dense depth minus LiDAR, coloured](../_assets/placeholder.svg)

*Pending: error map: dense depth minus LiDAR, coloured. Source: TBD.*
![Pending: MAE vs distance bin, one line per method](../_assets/placeholder.svg)

*Pending: MAE vs distance bin, one line per method. Source: TBD.*
![Preview from rig day 1 (no LiDAR): radar points and camera-coloured dense depth in the 3D viewer](figures/viewer_preview_2026-10-19.png)

*Preview from rig day 1 (no LiDAR): radar points and camera-coloured dense depth in the 3D viewer*


## Conclusion
Pending.

## Open items
- Needs M4 extrinsics and LiDAR-to-camera alignment first

## B-roll to capture (for the 5-minute video)
- [ ] Orbit video of the coloured 3D point cloud in the web viewer
- [ ] Side-by-side: camera frame vs dense depth vs LiDAR

## Reproduce
TBD in `src/fusion/`, `src/eval/`.

[Back to all experiments](../../EXPERIMENTS.md)
