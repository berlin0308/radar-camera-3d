# Progress against the proposal

The submitted proposal (15-663, "3D Imaging with a mmWave Radar and a Camera", 2026-11-20) lists six items to achieve and three hopes. This page tracks each one: the data on nunga, the code in this repo, where it stands and what is next. Numbers come from `EXPERIMENTS.md` and the experiment folders. Checked 2026-11-20.

## Resources

| What | Where |
|---|---|
| Raw traces (six sessions, 64 traces, 208 GB) | nunga `/data/christineh/DATASET/radarhd/<date>/<trace>/`, linked from `data/raw/<trace>` |
| Session log: what was recorded, data quality | [SESSIONS.md](SESSIONS.md); labels in `notes/shots.csv` |
| GRT dumps, 10/29 to 11/05 (28 traces, 132 GB) | nunga `/data/berlin/grt/<trace>/`; method in [GRT.md](GRT.md) |
| LiDAR poses (KISS-ICP) and accumulations | `data/poses/<trace>/`, `data/recon/<trace>/` (local, gitignored) |
| Extrinsics | `calib/extrinsics_joint.json` (11/03 solve; `joint_path()` picks the 10/21 solve before 10/29) |
| Results, one section per experiment | [EXPERIMENTS.md](EXPERIMENTS.md), `experiments/<name>/` |
| Report draft | [report/draft.md](report/draft.md); Overleaf "CP Final" (`report.tex`, 5 of 10 commits used in November) |
| Disk | `/` 99 % full (52 GB free); `/data` 95 % (355 GB free). New GRT dumps go to `/data/berlin/grt/` |

## The six items

| # | Item | Data | Code | Status | Next |
|---|---|---|---|---|---|
| 1 | **Calibration**, checked at the statue's range | 11/03 board shots (solve); 11/05 and 11/19 statue takes (check) | `src/calib/` | **Done** except the radar's height (from the joint solve, not taped). At the statue's range the check passes in the ground plane: on 11/05 the radar's statue cells land 4 cm from the statue in the LiDAR frame at 3.1 m; on 11/19 the near cells land within 4–6 cm of LiDAR surfaces | Write the check up as one paragraph in `experiments/m4-extrinsics` |
| 2 | **Single height**, five fills, day and night | Day: `2026-11-05.16-34-29` (+ take 2). Night: any take of the 11/19 stack | `src/fusion/` (seeds, fills, figure), `src/eval/` | **Day done** (report Figure 1). **Night: data recorded, not yet run.** A smoke test on `17-43-28` ran end to end, but that take has no seed on the statue; use `17-40-44` (LiDAR 1.66 m, statue seeded) | Run on `17-40-44`. Note: the 11/19 mark is about 0.6 m closer to the statue than the 11/05 one, so day and night are not the same view; the report calls it the same object from a different station (no reshoot) |
| 3 | **Multi-height**, night only: one mark, several heights, stacked, dense depth from several rows | Night stack, 11/19: 7 takes, 6 LiDAR heights 1.41, 1.54, 1.65 (×2), 1.72, 1.82, 1.91 m, 40–70 s each, statue 2.5 m from the radar. No day set (decided 11/20: no reshoot) | None yet | **Data done, code not started.** The radar puts a seed on the statue in 5 of the 7 takes. The rig shifts between takes (the statue moves up to 0.5 m sideways in the rig frame), so the takes need registering | `src/fusion/stack.py`: register each take to one reference with the LiDAR, seeds of all takes into one image, the four fills from several rows against one height (`17-40-44`) |
| 4 | **3D reconstruction**, day and night orbits, static points kept by Doppler, against the LiDAR | Day: Jeep `2026-10-29.15-49-40`, Scotty `2026-11-03.17-01-12`. Night: Scotty orbit `2026-11-19.17-55-35` (10–143 s) | `src/recon/` (poses, Doppler test, accumulation), `src/recon/day_night.py` (figure) | **Done, day and night.** Night, same chain and settings: 75 % of 78,868 detections kept, 90 % within 0.5 m of the LiDAR (day 92 %), coverage 72 % (day 71 %); odometry closes to 0.57 m over 43 m. [experiments/night-orbit](experiments/night-orbit/README.md), report Table 2 and Figure 14 | CFAR against GRT on the night orbit (item 5) |
| 5 | **CFAR and GRT** on the same frames and orbits, scored the same way | GRT dumps for every 10/29–11/05 trace; none for 11/19 | `GRT.md` §11, `src/grt/` | **Partly done**: CFAR against GRT on the 11/03 Scotty orbit, first-return Chamfer against the LiDAR (report section 6.5). Not yet on the night orbit or the stack | Run GRT on the 11/19 stack and orbit (GPU on nunga), score as for 11/03 |
| 6 | **Bronze and stone**, now part of the multi-height discussion | The 11/19 night stack; the LiDAR's `rfl` channel is in every trace | Statue mask and its split at 1.05 m exist (`dense_depth --object --object-split`) | **Not started** | Zoom on the statue: per height, which part (bronze figure or stone pedestal) returns to the radar and how strongly, and the LiDAR's reflectivity of each part; explains which heights miss the statue |

## Hopes

| Hope | Status |
|---|---|
| Elevation from the phase between the two array rows | Not started (`experiments/x9-elevation` is a stub, no data of its own). The 11/19 stack is a usable test: the same statue seen from six known heights |
| Backprojection across heights in place of stacking | Not started; needs item 3's registration first |
| A smooth metal panel against rough stone | No data |

## Schedule (from the proposal)

| Dates | Plan | State on 11/20 |
|---|---|---|
| Nov 20–22 | Multi-height capture, by day and one set at night | Night set done (11/19). No day set: multi-height is night only (decided 11/20) |
| Nov 23–30 | No captures. Stacking, night orbit reconstruction, material statistics | Night orbit done (11/20); stacking and material code to write |
| Dec 1–6 | Multi-height dense depth; single against multi-height, CFAR against GRT, day against night | — |
| Dec 7–14 | Figures, 3D renders, report and video | Report draft exists; it is 16 pages in Overleaf against a target of about 10 |
