# Handover: obj-orbit-360, the reconstruction side

*Yining → Berlin, 2026-11-08. Covers the work on branch `yining/obj-orbit-360`.*

Branch `yining/obj-orbit-360`, 8 commits off `e374bbd`. Code in `src/ext/`,
results in `experiments/obj-orbit-360/README.md`, a paragraph in `EXPERIMENTS.md`. `src/radar/` is
untouched.

## What's done

Four captures through one chain, each sensor's reconstruction of the same object put in the object's
own frame (the pedestal fit from `recon/closeup.py`), so they overlay with no registration step.

Against the LiDAR, symmetric trimmed Chamfer, no fit:

| | Chamfer | → LiDAR | LiDAR → |
|---|---|---|---|
| radar, GRT p > 0.9 | 7.35 cm | 17.1 cm | 1.3 cm |
| radar, CFAR static | 13.70 cm | 1.1 cm | 40.2 cm |
| 3DGS, rendered depth | 9.24 cm | 22.2 cm | 0.8 cm |
| RGB, DA3 + the rig's poses, 96 frames | **1.01 cm** | 1.0 cm | 0.9 cm |
| LiDAR reference | — | — | — |

Four things that might change how you read your own numbers:

1. **The reference's own repeatability is 0.76 cm.** Scotty orbited twice (11/03 dusk, 11/05 daylight,
   different paths) and reconstructed independently, the two agree to 0.76 cm. Anything at or below
   about 1 cm is at the floor of what this comparison can resolve — the camera's 1.01 cm included.

2. **Both radar clouds fail in opposite directions, and a one-directional score hides it.** 99 % of
   LiDAR points have a GRT point within 5 cm, but only 22 % of GRT's own points are that close. CFAR is
   the mirror image at 94 % / 27 %. Scored one way, the fog wins.

3. **GRT needed p > 0.9, not the 0.5 used for scoring,** before its cloud concentrated on the object at
   all. At 0.5 it fills the whole 1.2 m cut-out evenly. The log-odds fusion `GRT.md` recommends over a
   hard threshold is still untried — that may be the right answer rather than a higher threshold.

4. **A splatfacto model trained on the same frames repeats GRT's failure exactly**, from a completely
   different method: covers the LiDAR better than anything else (0.8 cm reverse) and is second-worst
   forwards (22.2 cm). Its depth is an alpha-weighted mean along the ray, so at a silhouette it returns
   a value between the object and the background.

Also: the radar's **coverage follows enclosure, not walking speed.** Four traces separate the two —
the plaza loop is walked 1.8× faster than the 11/05 orbit for the same 84–86 %, while the Cut, 1.9×
faster again, gives 23 %. Accuracy is flat at 0–2 cm across all four.

## Three things that need you

1. **GRT inference for the two 11/05 traces** — `2026-11-05.16-38-55` (Scotty orbit) and
   `16-25-42` (the 122 m plaza loop). `/data/berlin/grt/` has 10/29 and 11/03 only, so those two
   captures have the CFAR chain and no learned occupancy. I could not run it: there is no nrdk
   environment on my account and `/home/berlin/nrdk-venv` is not readable. The checkpoint at
   `/data/berlin/iq1m-checkpoints/base/small/` is readable, so if it is easier to point me at an env
   than to run it yourself, that works too. About 12 GB of output; `/data` has room.

2. **Two corrections to the session log**, which I recorded in the experiment page rather than editing
   your files:
   - `SESSIONS.md` lists nine traces for 2026-11-05. There are **ten** on disk.
   - The entry ends "Still not recorded: anything after dark", but `2026-11-05.19-11-08` is **21.6
     minutes beginning at 19:11**, all four streams complete, 6 invalid radar frames in 25,836. It is
     the longest trace in the dataset and the only one after dark, and nothing has been run on it.
     Worth fixing before anyone plans a trip to re-shoot night data.

3. **Two review calls on the PR**: `out/` is now gitignored (1 GB of clouds and frames), and three.js
   is fetched on first run rather than committed (`three.module.js` alone is 56k lines and would bury
   the diff). Happy to flip either.

## Two open items closed, one still open

- **Closed:** "drift on the orbit itself, by closing a lap next time". `16-25-42` does close a lap on a
  marked spot — first and last camera frames show the same laptop on the same table — and it closes to
  **0.62 m over 121.9 m, 0.5 % of path**, yaw to 13° over two full turns. That number can be carried to
  the other traces.
- **Closed:** the Scotty close-up now has the per-object numbers for the small-metal row.
- **Still open:** the car / tree / wall rows. The Jeep orbit of 10/29 is recorded *and* has a GRT dump
  and has not been run — that is the cheapest next result by a distance. A tree and a GHC wall are not
  recorded at all.

## How to look at it

```bash
.venv/bin/python -m src.ext.viewer.serve --port 8422        # then http://localhost:8422/
```

A three.js page, four captures in a dropdown, the clouds of each as toggles, the surroundings behind
one switch. `--bundle DIR` writes a self-contained copy (page, three.js, every cloud, ~50 MB) that runs
off any static server, for a machine with no access to nunga.

Every command is in the Reproduce section of `experiments/obj-orbit-360/README.md`.
