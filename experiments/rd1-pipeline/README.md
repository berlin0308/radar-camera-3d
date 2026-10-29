# Rig day 1: the full radar pipeline on real data (2026-10-19)

**Status:** Done (analysis 2026-10-20)

## Question
Does the whole chain, raw ADC to range, Doppler, angle, CFAR, 3D points and projection into the camera, work on our own captures?

## Method
Two traces with the grt-o chirp (Δd 8.74 cm, Δv 0.056 m/s, ±1.79 m/s, 22.4 m). Every step was checked against `xwr.rsp` on the same frame (relative error 5e-8). Analysis code was a reference implementation kept outside this repo.

## Data
| Trace | Content | Radar | Camera | LiDAR |
|---|---|---|---|---|
| `2026-10-19.15-45-38` | indoor desk, static, 50 s | 948 frames, all valid | radar starts 2.23 s after camera | 0 bytes |
| `2026-10-19.16-14-47` | lab, corridor, campus walk, 25 min | complete copy since 10/21 | AVI index broken at frame 9,663; `video_full.mkv` | 0 bytes |

## Results
| Finding | Number |
|---|---|
| Usable range | about 0.3 to 20 m; 20.5 to 22.4 m is the I/Q mirror of near echoes (about 25 dB down) |
| TDM-MIMO phase jump while walking | −1.59 rad between TX slots, +0.04 rad after per-slot compensation |
| Ego-motion from Doppler | static world traces −v·cosθ; walking speed about 1.4 m/s |
| CFAR | range-only CFAR keeps angle sidelobes; requiring an azimuth peak leaves 5 to 10 reliable points per frame |
| Azimuth sign | positive azimuth = left (from projecting a parked car onto the camera; confirmed on 10/21) |
| Radar to camera time offset | 2.23 s (trace A, timestamps), 2.4 s (trace B, cross-correlation) |

![Pending: pipeline diagram with the processor that runs each stage](../_assets/placeholder.svg)

*Pending: pipeline diagram with the processor that runs each stage. Source: Rig Day Radar Walkthrough handout.*
![Pending: range-Doppler map while walking, before and after TDM compensation](../_assets/placeholder.svg)

*Pending: range-Doppler map while walking, before and after TDM compensation. Source: handout.*
![Pending: top view of one frame: CFAR points with and without the azimuth-peak rule](../_assets/placeholder.svg)

*Pending: top view of one frame: CFAR points with and without the azimuth-peak rule. Source: handout.*
![First 3D output: radar points (orange) and a camera-coloured dense depth cloud, street scene at 800 s](../m6-dense-depth/figures/viewer_preview_2026-10-19.png)

*First 3D output: radar points (orange) and a camera-coloured dense depth cloud, street scene at 800 s*


## Conclusion
The pipeline works end to end on real data. Two steps that textbooks skip turned out to be required: TDM-MIMO phase compensation when moving, and an azimuth-peak check after CFAR.

## Open items
- Export the handout figures into `figures/` (the handout is the source)
- Re-run on the complete trace B copied on 10/21

## B-roll to capture (for the 5-minute video)
- [ ] Phone clip of the rig being carried on the campus walk
- [ ] Screen recording: scrolling the range-Doppler map while walking

## Reproduce
Reference analysis to be re-created under `src/` by the author (M2, M3).

[Back to all experiments](../../EXPERIMENTS.md)
