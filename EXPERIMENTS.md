# Experiments

Every experiment has a folder under [`experiments/`](experiments/) with the question, method, data, results (numbers and figures), conclusion, open items and the b-roll still to film. Figures not made yet are marked with a placeholder and a caption saying what goes there.

Legend: ✅ done · 🟡 preliminary · ⬜ planned · 💡 candidate extension

| | Experiment | When | Data | Key result | Report |
|---|---|---|---|---|---|
| ✅ | Full radar pipeline on real data | 2026-10-19 | trace A, trace B | TDM phase −1.59 → +0.04 rad; 5-10 points/frame; + azimuth = left | [rd1-pipeline](experiments/rd1-pipeline/README.md) |
| ✅ | Range and angle accuracy | 2026-10-21 | range ×4, angle ×5 | radar = 0.993 × tape + 0.18 m (±1.2 cm); 40° R −1.1°, ±20° +9/−6° | [m1-range-angle](experiments/m1-range-angle/README.md) |
| ✅ | LiDAR, timing, 128-beam swap | 10-21, 10-29 | dry runs | LiDAR fixed; OS-0-128 [128, 2048] at 10 Hz | [rig-health](experiments/rig-health/README.md) |
| 🟡 | Radar to camera extrinsics | 2026-10-21 | calib ×8 | 8/8 detections land on the target before calibration | [m4-extrinsics](experiments/m4-extrinsics/README.md) |
| 🟡 | Noise statistics | 2026-10-21 | scene_room | exponential after removing the static echo (p99 4.60 vs 4.61) | [x10-noise](experiments/x10-noise/README.md) |
| 🟡 | Multipath ghosts | 2026-10-21 | range ×4 | 2× echo at −12.5 dB (3 ft) | [x4-multipath](experiments/x4-multipath/README.md) |
| ⬜ | Own range and Doppler code | due 11-01 | M1, scene | TBD | [m2-range-doppler](experiments/m2-range-doppler/README.md) |
| ⬜ | Own angle, CFAR, 3D points | due 11-08 | M1, scenes | TBD | [m3-angle-cfar](experiments/m3-angle-cfar/README.md) |
| ⬜ | Dense depth vs LiDAR | due 11-30 | scenes | TBD | [m6-dense-depth](experiments/m6-dense-depth/README.md) |
| ⬜ | Object size and material, 360° orbits | Session 2 | orbits | TBD | [obj-orbit-360](experiments/obj-orbit-360/README.md) |
| 💡 | Radar through smoke | — | — | TBD | [x3-smoke](experiments/x3-smoke/README.md) |
| 💡 | Multi-frame accumulation | — | — | TBD | [x5-accumulation](experiments/x5-accumulation/README.md) |
| 💡 | Colour points by Doppler | — | — | TBD | [x6-doppler-colour](experiments/x6-doppler-colour/README.md) |
| 💡 | Confidence map | — | — | TBD | [x7-confidence](experiments/x7-confidence/README.md) |
| 💡 | Radome contamination | — | — | TBD | [x8-radome](experiments/x8-radome/README.md) |
| 💡 | Elevation accuracy | — | — | TBD | [x9-elevation](experiments/x9-elevation/README.md) |

## Capture sessions
| Date | Session | Traces | LiDAR | Notes |
|---|---|---|---|---|
| 2026-10-19 | Rig day 1 | 2 | none (0 bytes) | indoor desk, 25-min campus walk |
| 2026-10-21 | Rig day 2 | 23 | OS-0-64 from 15:04 | range, angle, calibration, room scene; 44 GB with day 1 |
| 2026-10-29 | Session 2 | 5 | OS-0-128 | campus walk, parking lot, crosswalk, Walking to the Sky, Scotty, plaza; 61 GB ([log](notes/sessions/2026-10-29.md)) |
| TBD | Session 3 | — | OS-0-128 | off campus; rain or fog if the weather allows |

## B-roll for the final video (all experiments)
- [ ] Rig close-up: radar, LiDAR, camera, how it is carried
- [ ] Start-up: recorder web page, LiDAR spinning
- [ ] Target placement with a tape measure (rig day 2 style)
- [ ] Walking orbits filmed by a second person
- [ ] Screen recordings: range peak moving, 3D viewer orbit, dense depth vs LiDAR side by side
- [ ] Night or bad-weather shot where the camera fails and the radar does not

New experiment: copy [`experiments/_TEMPLATE.md`](experiments/_TEMPLATE.md) into `experiments/<id>/README.md` and add a row above.
