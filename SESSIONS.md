# Rig sessions

One entry per capture day: the rig as it was, what was recorded, how good the data is, what it has been used for, and what the next session has to fix. The shot list for the next session is at the top. The recording procedure (start-up, shut-down, copying) is in `AGENDA_session2.md`, trace labels in `notes/shots.csv`, results in `EXPERIMENTS.md`, decisions in the Notion plan.

Data lives on nunga under `/data/christineh/DATASET/radarhd/<date>/<trace>/`; `data/raw/<trace>` in this repo links there. Every trace is four streams on one Unix clock: `radar/` (20 Hz raw ADC), `lidar/` (10 Hz Ouster range image), `camera/` (30 fps), `imu/` (100 Hz). The radar starts about 2 s after the camera in every trace; align by timestamp, never by frame index.

## Where the data stands (2026-11-01)

Three sessions, 30 traces, about 105 GB. The radar chain, the range and angle checks, the extrinsics, the GRT comparison and the orbit reconstructions all run on it; the numbers are in `EXPERIMENTS.md`. What it cannot yet show, and what the next session is for:

| Needed by | Have | Missing |
|---|---|---|
| M2–M3 radar chain, range and azimuth accuracy | 10/21 range shots (residuals ±1.2 cm) and angle shots on a flat board | the ±20° azimuth bias (−6°, +9°) is the board's, not the array's; a corner reflector would settle it |
| M4 extrinsics | 10/21 board placements, solved jointly, on the OS-0-64 rig | the LiDAR was swapped on 10/29; on the current rig the transform is only checked to 20–30 cm. Corner-reflector placements on this rig make it final |
| Object reconstruction (obj-orbit-360, Yining) | Jeep: one full orbit, clean. Scotty: 45 s partial loop at 1 m with a wall behind. Walking to the Sky: full orbit, rig tilted up. Planters: drive-bys | one more clean orbit at 3 m radius, two laps, closed, object in the open; a small object the same way |
| M5–M6 dense depth, day vs night | day only, handheld only | night: nothing. Static scenes from a marked spot, by day and again after dark |
| Capstone detection scenes | handheld walks with people and cars | rig on the ground; off-campus places (`AGENDA_session2.md`, Session 3) |

Verdict: the data is good enough for everything built so far. The two things that would make the report weak are not in it yet: a calibration taken on the rig as it is now, and night captures. Both fit in one afternoon.

## Next session: the last capture (before 2026-11-20)

Capture ends 11/20 (milestone M5); after that it is evaluation and writing. The list is in priority order: if the session is cut short, the top is what matters. About 2.5 h on campus plus 40 min after dark (dark by about 17:30 now). Procedure as in `AGENDA_session2.md`; the capstone's off-campus list there can share the day if there is time.

| # | What | How | Time | Serves |
|---|---|---|---|---|
| 0 | Dry run | 30 s, started 30 s after `lidar Initialized`; check all four streams, LiDAR `[128, 2048]` | 5 min | — |
| 1 | **Calibration with a corner reflector** | reflector on a stand, the flat board (or a sheet of card) directly behind it so the camera and the LiDAR see something at the same spot; 10 placements at 2–8 m, azimuth −40° to +40°, two heights; rig static, 10 s each (`./shoot.sh calib_<n> 10`); a photo from behind the rig and the tape distance for every placement | 20 min | M4 final (rerun `python -m src.calib.joint`), M1 ±20° repeat |
| 2 | **Rig geometry by tape** | radar board to LiDAR base (vertical and forward), radar to camera lens; write the numbers into `calib/` | 5 min | M4 (`GRT.md` found three inconsistent published offsets) |
| 3 | **Static scenes, day** | three spots marked with chalk and a photo; rig on the ground or a tripod, level; 60 s each; objects at 2–15 m (parked cars, a wall, a tree, people passing) | 15 min | M5, M6 ground truth by distance bin |
| 4 | **Handheld route, day** | 2 min, under 1 m/s, level; somewhere in it a person walks toward and away from the rig at 3–6 m | 5 min | M5, X6 Doppler |
| 5 | **Orbits** | one car and one small object (hydrant, bike rack, bin): radius 3 m, 0.5 m/s, two full laps, start and stop on a chalk mark, rig level, nothing flat within 6 m behind the object; one corner reflector in the scene at a taped distance; a second person films with a phone and photographs the object | 25 min | obj-orbit-360, Yining's reconstruction, b-roll for the video |
| 6 | **Spin in place** | open spot, 2 × one slow turn per minute | 3 min | odometry yaw (it slips ~150° in the 10/29 spin) |
| 7 | **Static scenes and route, night** | the same three spots and the same route after dark, same settings | 25 min | M5–M6: the "works in the dark" claim |
| 8 | If time | orbit of a tree (non-metal), X4 wall test (rig parallel to a wall at 1.5 m, 3 × 30 s), X10 open sky 60 s | 15 min | X4, X10, obj-orbit-360 |

Not again: orbits at 1 m (at that range the radar sees one blob and the odometry swings), a tilted rig, an object with a flat wall close behind it, START before the LiDAR has settled.

Afterwards: copy to nunga, `./validate.sh`, labels into `notes/shots.csv`, a session entry below (rig, traces, quality, what each trace is for), then rerun the calibration chain and `pytest`.

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
