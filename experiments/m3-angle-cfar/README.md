# M3: angle estimation, CFAR and 3D points

**Status:** Planned (due 11/08)

## Question
Are azimuth errors within ±7° (half a beam), and how many reliable points does one frame give?

## Method
8-element azimuth FFT with TDM compensation, elevation from the second row, CA-CFAR with the exponential threshold, azimuth-peak rule.

## Data
M1 angle shots, calibration shots, `scene_room`, Session 2 scenes.

## Results
| Placement | Reference error | Mine | Pass (±7°) |
|---|---|---|---|
| 20° left | +9.2° | TBD | TBD |
| 0° | 0° (bias) | TBD | TBD |
| 20° right | −6.3° | TBD | TBD |
| 40° right | −1.1° | TBD | TBD |

| CFAR setting | Points / frame | Matched to LiDAR within 0.5 m | False alarms |
|---|---|---|---|
| range only | TBD | TBD | TBD |
| + azimuth peak | TBD | TBD | TBD |
| + 20-frame accumulation | TBD | TBD | TBD |

![Pending: 3D radar point cloud of `scene_room` over the 128-beam LiDAR cloud](../_assets/placeholder.svg)

*Pending: 3D radar point cloud of `scene_room` over the 128-beam LiDAR cloud. Source: TBD.*
![Pending: points per frame vs CFAR threshold (ROC-style curve against LiDAR)](../_assets/placeholder.svg)

*Pending: points per frame vs CFAR threshold (ROC-style curve against LiDAR). Source: TBD.*


## Conclusion
Pending.

## Open items
- Decide on super-resolution by 11/02 based on these errors

## B-roll to capture (for the 5-minute video)
- [ ] Screen recording: radar points appearing frame by frame over the LiDAR cloud

## Reproduce
TBD in `src/radar/`.

[Back to all experiments](../../EXPERIMENTS.md)
