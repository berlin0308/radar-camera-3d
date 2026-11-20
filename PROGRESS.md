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
| 2 | **Single height**, five fills, day and night | Day: `2026-11-05.16-34-29` (+ take 2). Night: any take of the 11/19 stack | `src/fusion/` (seeds, fills, figure), `src/eval/` | **Day done** (report Figure 1). **Night: data recorded, not yet run.** A smoke test on `17-43-28` ran end to end, but that take has no seed on the statue; use `17-40-44` (LiDAR 1.66 m, statue seeded) | Run on `17-40-44`. Note: the 11/19 mark is about 0.6 m closer to the statue than the 11/05 one, so day and night are not the same view; the day stack from the 11/19 mark would give a true pair |
| 3 | **Multi-height**: five or more heights from one mark, stacked with poses, dense depth from several rows | **Night stack recorded 11/19**: 7 takes, 6 LiDAR heights 1.41, 1.54, 1.65 (×2), 1.72, 1.82, 1.91 m, 40–70 s each, statue 2.5 m from the radar. **Day stack not recorded** | None yet | **Data half done, code not started.** The radar puts a seed on the statue in 5 of the 7 takes, so stacking also fills the takes that miss it. The rig's position and yaw shift between takes (statue moves by up to 0.5 m sideways in the rig frame), so the takes need registering | Shoot the day stack from the 11/19 mark (Nov 20–22, as scheduled). Write `src/fusion/stack.py`: LiDAR-to-LiDAR registration of each take to one reference, seeds of all takes into one image, fills from several rows |
| 4 | **3D reconstruction**, day and night orbits, static points kept by Doppler, against the LiDAR | Day: Jeep `2026-10-29.15-49-40`, Scotty `2026-11-03.17-01-12` (both reconstructed); Scotty `2026-11-05.16-38-55` (recorded, no poses). Night: **Scotty orbit `2026-11-19.17-55-35`** (148 s), the night walks `2026-11-05.19-11-08`, `2026-11-19.17-26-10`, `2026-11-19.18-00-07` | `src/recon/` (poses, Doppler test, accumulation) | **Day done** (91 % on the car, 92 % on the statue, of the kept radar points within 0.5 m of a LiDAR surface). **Night: data recorded, no poses yet** | KISS-ICP poses for `17-55-35`, then the same accumulation and score as the day orbits (N4 in the report) |
| 5 | **CFAR and GRT** on the same frames and orbits, scored the same way | GRT dumps for every 10/29–11/05 trace; none for 11/19 | `GRT.md` §11, `src/grt/` | **Partly done**: CFAR against GRT on the 11/03 Scotty orbit, first-return Chamfer against the LiDAR (report section 6.5). Not yet on the night orbit or the stack | Run GRT on the 11/19 stack and orbit (GPU on nunga), score as for 11/03 |
| 6 | **Bronze and stone**: radar power and LiDAR return per material | 11/05 day takes (2), 11/19 night stack (7); the LiDAR's `rfl` channel is in every trace | Statue mask and its split at 1.05 m exist (`dense_depth --object --object-split`) | **Not started** | Small script: radar power of the cells on the bronze figure and on the pedestal, LiDAR reflectivity of each part, day and night |

## Hopes

| Hope | Status |
|---|---|
| Elevation from the phase between the two array rows | Not started (`experiments/x9-elevation` is a stub, no data of its own). The 11/19 stack is a usable test: the same statue seen from six known heights |
| Backprojection across heights in place of stacking | Not started; needs item 3's registration first |
| A smooth metal panel against rough stone | No data |

## Schedule (from the proposal)

| Dates | Plan | State on 11/20 |
|---|---|---|
| Nov 20–22 | Multi-height capture, by day and one set at night | Night set done early (11/19). **Day set still to record** |
| Nov 23–30 | No captures. Stacking, night orbit reconstruction, material statistics | Night orbit is on nunga; stacking and material code to write |
| Dec 1–6 | Multi-height dense depth; single against multi-height, CFAR against GRT, day against night | — |
| Dec 7–14 | Figures, 3D renders, report and video | Report draft exists; it is 16 pages in Overleaf against a target of about 10 |
