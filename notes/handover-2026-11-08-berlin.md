# Handover: GRT dumps and the night trace

*Berlin → Yining, 2026-11-08. Answers `notes/handover-2026-11-08-yining.md` on branch `yining/obj-orbit-360`.*

## 1. GRT inference: done, for every trace of 10/29, 11/03 and 11/05

Your item 1 asked for the two 11/05 orbits (`16-38-55`, `16-25-42`). Both are done, and so is everything
else on those three days that had no dump yet (20 traces, about 67,000 frames, 100 GB). Nothing was run
on 10/21 or 10/19: that is the rig before the LiDAR swap and the GRT label does not apply to it.

Same checkpoint as before (`/data/berlin/iq1m-checkpoints/base/small`, `src/grt/infer.py`, batch 16), one
4090. The whole run took about 15 minutes.

```
/data/berlin/grt/<trace>/
    occ3d_logit.npy   float16  [N, 64 el, 128 az, 64 range]   model output (logits)
    occ3d_label.npy   bool     [N, 64 el, 128 az, 64 range]   nrdk's own LiDAR label, same frames
    ts.npy            float64  [N]                            radar timestamps (Unix seconds)
```

```python
import numpy as np
d = "/data/berlin/grt/2026-11-05.19-11-08"
logit = np.load(f"{d}/occ3d_logit.npy", mmap_mode="r")   # about 1.6 MB per frame, memory-map it
ts = np.load(f"{d}/ts.npy")                              # keys the poses, see src/recon/poses.py
```

| Trace | Frames | Window in the trace (s) | Size | What it is |
|---|---|---|---|---|
| `2026-10-29.15-18-16` | 509 | 1-27 | 0.8 GB | a 28 s take (content not looked at) |
| `2026-10-29.15-49-40` | 1,200 | 690-750 | 1.9 GB | Jeep window (earlier dump) |
| `2026-10-29.16-12-38` | 11,817 | 1-592 | 18.6 GB | the sculpture and campus paths |
| `2026-10-29.16-22-36` | 1,400 | 160-230 | 2.2 GB | Scotty (earlier dump) |
| `2026-10-29.16-27-55` | 5,078 | 1-255 | 8.0 GB | campus paths |
| `2026-11-03.16-*` (8 shots) | 110-114 each | 1-7 | 0.17 GB each | the calibration board shots |
| `2026-11-03.16-55-34` | 59 | 1-4 | 0.09 GB | |
| `2026-11-03.17-01-12` | 2,700 | 30-165 | 4.3 GB | Scotty orbit (earlier dump) |
| `2026-11-03.17-07-11` | 4,000 | 5-205 | 6.3 GB | lawn (earlier dump) |
| `2026-11-03.17-11-37` | 8,078 | 1-405 | 12.7 GB | paths and indoors (earlier dump) |
| `2026-11-03.17-18-46` | 1,579 | 1-80 | 2.5 GB | spin in place (earlier dump) |
| `2026-11-05.15-43-05` | 510 | 1-27 | 0.8 GB | empty lab |
| `2026-11-05.15-44-13` | 1,110 | 1-57 | 1.8 GB | lab, a person at about 2 m |
| `2026-11-05.16-10-35` | 1,111 | 1-57 | 1.8 GB | outdoors, rig static |
| `2026-11-05.16-12-32` | 1,112 | 1-57 | 1.8 GB | same spot |
| `2026-11-05.16-25-42` | 5,060 | 1-254 | 8.0 GB | the plaza loop (your item 1) |
| `2026-11-05.16-34-29` | 1,110 | 1-57 | 1.8 GB | Scotty, rig static |
| `2026-11-05.16-36-41` | 1,110 | 1-57 | 1.8 GB | Scotty, second static take |
| `2026-11-05.16-38-55` | 2,987 | 1-150 | 4.7 GB | Scotty orbit (your item 1) |
| `2026-11-05.16-46-03` | 12,299 | 1-616 | 19.3 GB | campus walk, **cut off**, see below |
| `2026-11-05.19-11-08` | 25,795 | 1-1,291 | 40.6 GB | **night walk**, see section 3 |

Things to know before using them:

- **The frame count is 40 or so short of the radar's.** GRT's loader synchronises the radar to the LiDAR
  and drops the first and last frames (for example 509 for the 552 radar frames of `15-18-16`). Use
  `ts.npy` to find the frame, not the radar frame index.
- **`16-46-03` stops at 616 s on purpose.** The recording was cut off; the radar's `valid` flags end at
  616.7 s, so the run was limited to that (`--seconds 616`). See `notes/sessions/2026-11-05.md`.
- **Four older dumps are windows,** not whole traces (the four marked above).
- Logits, not probabilities: `p = sigmoid(logit)`. Your finding that p > 0.9 is needed, not 0.5, applies to
  all of these. The log-odds fusion of `GRT.md` is still untried.
- Frame axes are `(el, az, range)` as nrdk stores them, in the LiDAR sensor frame, so no extrinsic enters.
  The label is nrdk's occupancy of the simultaneous LiDAR scan.
- Permissions are `rwxrwxr-x`; the whole of `/data/berlin/grt` is now 132 GB and `/data` has about 790 GB free.

## 2. Corrections to `SESSIONS.md`: done

Both of your corrections were right and are fixed (commit `88148c9`): 2026-11-05 has **ten** traces, 66 GB,
and the entry no longer says nothing was recorded after dark. What is still missing is a *static* scene
after dark; the lab-dark and Scotty static takes of the plan are not on nunga.

## 3. The night trace, `2026-11-05.19-11-08`

21.5 minutes from 19:11, 28 GB, all four streams complete: 25,836 radar frames at 20 Hz with 6 invalid
(0.0 %), 12,640 LiDAR scans, 38,821 camera frames, 129,419 IMU samples. The files end cleanly (the rig was
shut down properly), unlike `16-46-03`. Camera exposure was not recorded.

What is in it, from the IMU and from contact sheets of the camera frames (the frames show people and stay
local; nothing in the repo):

| Time (s) | What |
|---|---|
| 0-60 | rig still, on a desk in the lab |
| 60-200 | picked up and carried through the lab and a lobby |
| about 200-1,140 | **a walk outdoors, at night, along a road**: street lamps, headlights and tail lights of cars (about 330, 430 and 490 s), a person (about 380 s), a hydrant, a sidewalk |
| 1,140-1,260 | walking, slower, back to a door |
| 1,260-1,292 | rig set down, still |

Nothing but the validation and the GRT dump has been run on it: no poses, no CFAR points, no
accumulation. It is the only trace after dark, and the only one that has a road with cars and people at
night. The nearest day counterparts are `2026-10-29.15-49-40` (a crossing with pedestrians, the Jeep and a
row of parked cars, about 220-1,230 s) and `2026-10-29.16-12-38` (a crossing with two cars and a
pedestrian, about 50-260 s). They are not the same route.

For your open car / tree / wall rows: this does not give an orbit of a car, but it has cars at the side of
a road at night, seen by the radar and the LiDAR.

## 4. The two review calls on your PR

Not answered yet; Berlin will reply there. Neither decision has been made, and nothing of yours was
touched or merged.

## 5. Not done

- Nothing was compared for the night trace: no GRT against CFAR, no figure.
- The log-odds fusion, and the per-object table (car, tree, wall), are not started.
- The radar's mounting height is still the held value from 10/21, not measured.
