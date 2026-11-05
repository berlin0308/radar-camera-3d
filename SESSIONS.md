# Rig sessions

One entry per capture day: the rig as it was, what was recorded, how good the data is, what it has been used for, and what the next session has to fix. The shot list for the next session is at the top. The recording procedure (start-up, shut-down, copying) is in `AGENDA_session2.md`, trace labels in `notes/shots.csv`, results in `EXPERIMENTS.md`, decisions in the Notion plan.

Data lives on nunga under `/data/christineh/DATASET/radarhd/<date>/<trace>/` `data/raw/<trace>` in this repo links there. Every trace is four streams on one Unix clock: `radar/` (20 Hz raw ADC), `lidar/` (10 Hz Ouster range image), `camera/` (30 fps), `imu/` (100 Hz). The radar starts about 2 s after the camera in every trace; align by timestamp, never by frame index.

## Where the data stands (2026-11-03)

Four sessions, 43 traces, 118 GB on nunga. The radar chain, the range and angle checks, the extrinsics, the GRT comparison and the orbit reconstructions all run on it; the numbers are in `EXPERIMENTS.md`. What 11/3 added and what is still missing:

| Needed by | Have | Missing |
|---|---|---|
| M5–M6 dense depth, day vs night | handheld by day (10/29), handheld at sunset (11/3), where the image still has most of its edges | **after dark: nothing. Any static outdoor scene: nothing.** No pair of the same view in two lights |
| M4 extrinsics | solved on the current rig from the 11/3 board shots: radar → LiDAR 1.6 cm, 0.58°; LiDAR → camera 6.4 px | the radar's height is held from 10/21 (these shots cannot see it): a tape measure. Nothing tests the solve beyond 3 m: a corner reflector if there is one |
| M2–M3 azimuth | the ±20° bias is explained: a flat board returns from its specular point, not its centre. With that model the 10/21 angle shots, held out, are within 0.75° | beyond 3 m, as above |
| Object reconstruction (obj-orbit-360, Yining) | Jeep orbit, clean. Scotty on 11/3: two passes at 2.6 m, pedestal repeats to 2 cm, wall still behind it. Walking to the Sky, tilted. A 192 m loop around the lawn, a six-turn spin | a small object with nothing behind it; ask Yining whether a second car is worth it |
| Capstone detection scenes | handheld walks with people and cars, corridors and an atrium with people | rig on the ground; off-campus places (`AGENDA_session2.md`) |

Verdict: 11/3 closed the calibration gap and gave a much better small-object orbit, but it stopped recording at 17:20, five minutes after sunset and before any darkness. The report's main claim, a 3D camera that works in the dark, still has no data behind it. That is the one thing the next session has to deliver.

## Next session: proposal (the last capture, before 2026-11-20)

Capture ends 11/20 (milestone M5); after that it is evaluation and writing. Aim for a clear evening between 11/9 and 11/14: M4 is due 11/15, and it leaves a spare evening if it rains. Sunset that week is about 17:06, civil dusk 17:35, nautical dusk 18:08, so **the night takes start at 18:10**, not at 17:30 as this list said before. About 3 h 15 min with the wait, from 15:30 to 18:45.

Before the day:

- **A corner reflector, if the lab has one.** Ask Tianshu. It is no longer what the calibration hangs on, so do not build one for this.
- **The camera's exposure.** Find out whether it is automatic. Lock it, or at least write down what it is for each take.
- **Battery.** Ask Tianshu whether the rig lasts 3 h. Powering it off in the gap is fine; after the restart wait 30 s past `lidar Initialized`.
- **A tripod or a cart**, chalk and a tape. **Ask Yining** what orbit they want, if any.

| # | When | What | How | Time | Serves |
|---|---|---|---|---|---|
| 0 | 15:30 | Dry run | 30 s, started 30 s after `lidar Initialized`; all four streams, LiDAR `[128, 2048]` | 5 min | — |
| 1 | daylight | **Rig geometry by tape** | radar board to LiDAR base (vertical, forward, sideways), radar to camera lens; a side photo and a top photo; numbers into `calib/` | 5 min | M4: the radar's height, which the board shots cannot see, and a check on the solve: it says the camera is 14 cm below the LiDAR and the radar 13 cm ahead of it |
| 2 | by 16:30 | **Static scenes, day** | three spots marked with chalk and a photo; rig on the tripod or cart, level; 60 s each. Fixed objects at three depths: (a) Scotty plaza: statue 3–4 m, planters 5–8 m, the wall 10–12 m square to the rig, not oblique; (b) a lamp-lit path with the lamp on one side only: a bench or bike rack at 2–3 m, a post or bin at 6 m, a building at 15 m; (c) a parking lot, cars at 4–10 m, as the bonus (count the cars; they may leave). A reflector at a taped 5 m in each | 15 min | M5, M6 ground truth by distance bin |
| 3 | daylight | Corner reflector, if we have one | rig static; reflector on a stand at 3, 5, 8, 12 m, across ±30°, two heights, 8 placements, 10 s each (`./shoot.sh calib3_<n> 10`); a card behind it, tilted so it mirrors away from the radar, gives the camera and the LiDAR the same spot; a photo and the tape distance for each | 20 min | M4: the first test of the solve beyond 3 m (0.7° of yaw is 12 cm at 10 m) |
| 4 | daylight | Same walks by day | the 11/3 lawn loop (about 3.5 min) and the two-pass Scotty orbit, same paths; on the loop a person walks toward and away from the rig at 3–6 m | 8 min | M5: day against 11/3 sunset against night on one route; X6 |
| 5 | 16:45–18:00 | If Yining wants it, then if time | a small object in the open (hydrant, bin) at 3 m radius, 0.5 m/s, two laps in one direction, nothing flat within 6 m behind it; a tree; X4 wall (rig parallel at 1.5 m, 3 × 30 s); X10 open sky 60 s | up to 45 min | obj-orbit-360, X4, X10 |
| 6 | 18:10 | **Static scenes, night** | the same three chalk marks, the reflector where it was, the camera exposure and gain exactly as by day and written down; a person walks through the lamp-lit path at 3–6 m | 15 min | M5–M6: the "works in the dark" claim, X6 at night |
| 7 | after | Same walks, night | the lawn loop and the Scotty orbit again | 8 min | M5: the third point on the route, odometry and radar independent of light |

If the session is cut short, items 1, 2 and 6 are the session: about 1 h 15 min of work plus the wait for dark. Drop the rest first.

Day and night: the only variable is the light, so the objects are fixed ones (a car that drives off between the two takes loses the pair), the station is the same chalk mark, and the camera settings are the same. Each scene has a lit half and a dark half, so one night frame shows where the image still has edges for the fill and where the radar is on its own. The Scotty plaza is the one we know best in all three sensors: by night the camera loses the dog and the LiDAR and radar do not change. Re-walking the 11/3 routes makes those traces useful as the middle point of a day, sunset, night series.

Not again: another calibration inside 3 m; walks in place of static scenes; stopping before 18:00; an object with a flat wall close behind it; a tilted rig; START before the LiDAR has settled.

Afterwards: copy to nunga, `./validate.sh`, labels into `notes/shots.csv`, the tape numbers and photos with the upload, a session entry below, then `python -m src.calib.plane_solve` with the taped height and `pytest`.

## 2026-11-05 · Session 4 · static scenes, Scotty plaza, a campus walk

Rig unchanged (OS-0-128, same config). Nine traces, 38 GB, all pass `roverd validate`: an empty lab and a person squatting at 2 m, an outdoor static scene with a car and tripods (twice), a walk to the Scotty plaza, two static takes at Scotty, a Scotty orbit, and a 13-minute campus walk that was **cut off** (use 0-616 s; the camera needs rewrapping to read past 297 s). Seven figures on it and on 11/03 (session health, board placements, static-scene heatmaps, rig geometry, ego-motion Doppler, radar on LiDAR, Scotty dusk against day): [notes/sessions/2026-11-05.md](notes/sessions/2026-11-05.md). Still not recorded: anything after dark.

## 2026-11-03 · Session 3 · lab calibration, campus at sunset

Rig: unchanged since 10/29 (OS-0-128, same config). 13 traces, 24 GB (uploaded to `transfer_2026-11-03/`, since moved to `2026-11-03/`). Content is read from the frames and the odometry because no notes came with the upload. Full log with per-trace tables and how each number was made: [notes/sessions/2026-11-03.md](notes/sessions/2026-11-03.md).

| Trace | Length | Content |
|---|---|---|
| `16-03-39` to `16-27-20` (8) | 10 s each | lab, rig static, flat board at about 3, 2, 1.5 m across ±41° |
| `16-55-34` | 7 s | dry run outdoors |
| `17-01-12` | 195 s | walk to the plaza → **Scotty, two passes at 2.6 m (30–165 s)** → a table with people |
| `17-07-11` | 211 s | **192 m loop around a large lawn** (looks like the Cut) |
| `17-11-37` | 410 s | paths, a lit ramp → **indoors from 205 s**: corridors, a bridge, an atrium balcony |
| `17-18-46` | 84 s | **spin in place on the balcony, six turns**, often tilted up |

Quality. `roverd validate` passes on all 13; radar at most 4 invalid frames per trace; LiDAR up to 1.1 % of scans missing outdoors and 4.4 % in `17-11-37`, where the gaps cluster indoors; camera gaps ≤ 67 ms (108 ms once). Light: sunset was at about 17:15 and the walk went indoors at that moment, so there is no outdoor frame after sunset; edge strength on the ground is 8–12 against 12–14 by day. The trace does not say whether the exposure was automatic.

Calibration. The board shows in radar, LiDAR and image in all eight shots, so they became the current rig's solve (`python -m src.calib.plane_solve`). Radar → LiDAR: yaw +92.75° ± 0.71°, t = (−0.133, +0.012, −0.269 held) m, range residual 1.6 cm, azimuth 0.58°, modelling the board as a mirror. LiDAR → camera: 6.4 px on 29 corners, where the 10/21 solve gives 52 px. Correction: an earlier version of this entry said the LiDAR may have moved about 20 cm in the swap. It did not; that residual came from a board position pulled toward the floor. Details and the checks on other traces: [experiments/m4-extrinsics](experiments/m4-extrinsics/README.md).

The orbit. −360° then +355° at 0.35 m/s, range to the pedestal 2.6 m median (1.9–5.0 m), the pedestal repeats between passes to 2 cm. Radar: 68 % of 67,106 CFAR points pass the static filter, 1 cm median to the LiDAR, GRT Chamfer 0.49 / 0.64 m. In range the radar sees the near side; across the beam it is still one blob. The spin holds too: 26° against the IMU over six turns, where the 10/29 spin slipped about 150°.

Every long trace went through the same chain as the Jeep and Scotty of 10/29: poses, Doppler check, accumulation with dense power maps, radar on camera and LiDAR, GRT dumps and comparisons. Per-trace numbers and commands are in the session log, figures in `figures/*_1103.png`. Indoors the CFAR points spread well outside the corridor walls, probably multipath.

Used for: `experiments/m4-extrinsics`, the current rig's calibration; `experiments/obj-orbit-360`; `GRT.md` §10.

Next time: after dark, static, same chalk marks by day and by night; the rig by tape.

## 2026-10-29 · Session 2 · campus, handheld

Rig: OS-0-128 LiDAR (swapped in that morning, serial 992112000195; NUC config `beams: 128`, explicit IPv4 address), radar chirp `radarhd-o` as before, camera 1920×1080 at 30 fps. Five traces, 61 GB. The full log with per-trace timelines, stream counts and what each segment is for: [notes/sessions/2026-10-29.md](notes/sessions/2026-10-29.md).

| Trace | Length | Content |
|---|---|---|
| `2026-10-29.15-18-16` | 30 s | dry run after the LiDAR swap |
| `2026-10-29.15-49-40` | 20.5 min | lab → campus paths → **parking lot, full orbit of a Jeep (11–17 min)** → garden |
| `2026-10-29.16-12-38` | 9.9 min | crosswalk with cars, stairs with people → **Walking to the Sky orbit (5:30–9:40)**, rig often tilted up |
| `2026-10-29.16-22-36` | 5.2 min | the Cut, colonnade → planters → **Scotty statue (2:59–3:45)**, flat wall behind → plaza |
| `2026-10-29.16-27-55` | 4.3 min | people past a planter, tree-lined path, red food truck |

Quality. All four streams in every trace: radar at most 8 invalid frames per trace (< 0.07 %), LiDAR 1.5–1.8 % of scans missing with gaps up to 0.7 s while turning, camera gaps ≤ 65 ms (175 ms once, at the start of `16-12-38`). The calibration is the 10/21 one, solved on the OS-0-64 rig; on this rig it holds to 20–30 cm, enough for the overlays and the static-world filter but not for a final number. KISS-ICP odometry (gyro-deskewed, 30 m map) is stable on the orbits: scan-to-scan jitter 0.1 m, the orbit centre off by 0.45 m over the Scotty loop; the yaw slips ~150° while the rig stands still or spins in place (785–890 s of the Jeep trace), so poses are per segment, not per trace.

Per object. The Jeep orbit is the good one: wide, level, in an open lot; static-filtered radar points sit 2 cm (median) from the LiDAR, 88 % within 0.5 m. Scotty is 45 s at 1–1.3 m, less than a loop: the LiDAR sees the statue in full (8,864 points on it in one scan at 2.2 m) while the radar sees one blob the size of the pedestal, because at 1 m the 14° beam and the 9 cm range bin both cover the whole object; the flat wall behind it puts a ghost at 8–9 m. Walking to the Sky is a full orbit but the rig points up for much of it, so the ground estimate and the odometry suffer. The planters are drive-bys. The operator walks inside the radar's view in every orbit; the static-world filter removes them without segmentation. No night data.

Used for: `experiments/obj-orbit-360` (poses, static-world filter, accumulation, Scotty close-up), `GRT.md` §10 (GRT on the Scotty and Jeep windows), `experiments/rig-health`.

Next time: calibration on this rig; orbits at 3 m with two closed laps; night.

## 2026-10-21 · Rig day 2 · indoor, 12 ft room

Rig: OS-0-64 LiDAR, `radarhd-o` chirp, same camera. 23 traces (`notes/shots.csv`), 44 GB together with 10/19. The room is 3.7 m deep, so nothing is farther than 12 ft.

- `14-17-33` (30 s): LiDAR still empty (START pressed before the LiDAR had reconfigured). `15-04-35` (50 s): the first trace with all four streams, after the fix. Every trace from here on has LiDAR and no radar packet loss.
- Range: a flat board at 3, 6, 9, 12 ft, 30 s each.
- Angle: the board at 8 ft, 0°, ±20°, ±40°, 30 s each. `15-36-20` holds the right-40° shot only in its last 30 s; `15-37-37` is empty.
- Calibration: `calib_01`–`08` plus `03b`, `05b` (03 and 05 were missed and redone), 10 s each, the board spread in range, azimuth and height.
- `scene_room` (60 s): the room, all four sensors.

Quality. Range: radar = 0.993 × tape + 0.183 m, residuals within ±1.2 cm; the fixed +0.183 m is now subtracted in `detect()`. Azimuth: sign confirmed (positive = left); 0° reads +4.5° (taken as bias), then −1° at right 40°, −6° at right 20°, +9° at left 20°; left 40° returned 17 dB too little and is dropped. The board is the limit, not the array: at 4 mm wavelength a flat board is a mirror, and a small tilt moves its apparent centre by degrees. Extrinsics from the seven usable placements (joint solve, LiDAR as world): radar→LiDAR yaw +94.4°, t = (−0.15, +0.02, −0.27) m, range residual 2.9 cm, 1σ ±1.5° and ±(5, 2, 15) cm; LiDAR→camera ±3°, 23 px. Height is the weak axis: all placements are within 3.7 m and at similar heights.

Used for: `experiments/m1-range-angle`, `experiments/m4-extrinsics`, `experiments/rig-health`, the deck's m1 and calibration sections.

Next time: the same placements with a corner reflector, outdoors, 2–8 m, on the current rig.

## 2026-10-19 · Rig day 1 · lab desk and a campus walk

Rig: OS-0-64 LiDAR nominally, but both traces have empty LiDAR files (START pressed too early; understood on 10/21). `radarhd-o` chirp.

- `2026-10-19.15-45-38` (50 s): rig static on a lab desk, the lower half of the view blocked by near objects. Radar complete (948 frames, all valid), camera complete; the radar starts 2.23 s after the camera.
- `2026-10-19.16-14-47` (25 min): lab → corridor → campus (about 300–840 s outdoors) → lab. The radar has only the first 841 s (16,828 frames) and no `ts`, `meta.json` or `valid`; the AVI index breaks at frame 9,663 (use `video_full.mkv`). The radar clock was rebuilt by cross-correlating the radar's off-zero-Doppler energy with the camera's frame differences: offset 2.4 s, consistent with the first trace.

Quality: radar and camera only; no ground truth; timing in the long trace is reconstructed, not recorded. The walk is still the best handheld sample of the ego-motion Doppler signature (the static world on the −v·cos θ curve at 1.4 m/s), the TDM phase jump, the IIQQ sample order and the I/Q mirror at 20.5–22.4 m.

Used for: the Rig Day Radar Walkthrough handout, the `experiments/m6-dense-depth` preview (co-located guess, no LiDAR), `experiments/rig-health`.
