# Progress against the proposal

The submitted proposal (15-663, "3D Imaging with a mmWave Radar and a Camera", 2026-11-20) lists six items to achieve and three hopes. This page tracks each one: the data on nunga, the code in this repo, where it stands and what is next. Numbers come from `EXPERIMENTS.md` and the experiment folders. Checked 2026-11-20, updated after the night orbit and the height stack.

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
| 2 | **Single height**, five fills, day and night | Day: `2026-11-05.16-34-29` (+ take 2). Night: `2026-11-19.17-40-44` | `src/fusion/` (seeds, fills, figure), `src/eval/` | **Done, day and night.** Night, first take of the stack: 1.22–1.30 m above the ground; none of the statue's outline is distinct in colour, so every fill keeps 48–52 % of it (constant 46 %) and least squares' lead over Gaussian shrinks from 0.28 m by day to 0.07 m. [m7-multi-height](experiments/m7-multi-height/README.md), report Figure 15 (top rows) | The night mark is 2.5 m from the statue, the day one 3.3 m: the report calls it the same object from a different station (no reshoot) |
| 3 | **Multi-height**, night only: one mark, several heights, stacked, dense depth from several rows | Night stack, 11/19: 7 takes, LiDAR 1.20–1.62 m above the ground, tilted 2.5–8° (the 1.41–1.91 m first logged were wrong), radar plane at the statue 0.83–1.49 m. No day set (decided 11/20) | `src/fusion/stack.py` (ICP registration, seeds of all takes in one image, fills on several rows) | **Done.** Registration 1.8–2.6 cm. Seven heights, 66 seeds: 1.76–2.35 m above the ground, against 1.22–1.30 m from the first take and a median of 1.46–2.52 m over the takes alone; least squares better on the statue (1.81 against 2.44 m, better than 6 of 7 takes alone). The radar's range profile is the same from every height, so a seed's height is the radar's, not the reflector's. [m7-multi-height](experiments/m7-multi-height/README.md), report Figure 15 | Elevation from the two array rows (hope 1) is what would add vertical information; the stack is its test set |
| 4 | **3D reconstruction**, day and night orbits, static points kept by Doppler, against the LiDAR | Day: Jeep `2026-10-29.15-49-40`, Scotty `2026-11-03.17-01-12`. Night: Scotty orbit `2026-11-19.17-55-35` (10–143 s) | `src/recon/` (poses, Doppler test, accumulation), `src/recon/day_night.py` (figure) | **Done, day and night.** Night, same chain and settings: 75 % of 78,868 detections kept, 90 % within 0.5 m of the LiDAR (day 92 %), coverage 72 % (day 71 %); odometry closes to 0.57 m over 43 m. [experiments/night-orbit](experiments/night-orbit/README.md), report Table 2 and Figure 14 | CFAR against GRT on the night orbit (item 5) |
| 5 | **CFAR and GRT** on the same frames and orbits, scored the same way | GRT dumps for every 10/29–11/05 trace, the 11/19 night orbit and the seven stack takes (`/data/berlin/grt/`) | `GRT.md` §11–12, `src/grt/` (`align`, `vs_cfar`, `accumulate`, `heights`) | **Done, day and night.** Night orbit: Chamfer 0.61 / 0.76 m at p > 0.5 (day 0.49 / 0.64), identical to day at p > 0.9 (0.35 / 0.67 m). Same score for both, day / night: per frame from above, CFAR 0.12 / 0.13 m to the label but the label 1.5 m from it; GRT p > 0.9 0.09–0.11 / 0.12–0.14 m. Accumulated: CFAR 92 / 90 % within 0.5 m, coverage 71 / 72 %; GRT 81 / 71 %, coverage 82 / 84 %. Height stack: GRT's height on the statue does not follow the radar's (slope 0.01) but is smeared 0.5 m upwards. Report §6.6, Tables 5–6 | — |
| 6 | **Bronze and stone**, inside the multi-height discussion | The 11/19 night stack; the LiDAR's `rfl` channel | `src/fusion/stack.py` (per-take power along the statue's azimuth, LiDAR range per part, `LidarTrace.channel`) | **Done.** At every height the radar's statue peak (31–35 dB) is at the bronze's range, even when its plane crosses the stone. The stone shows only from the two lowest heights, 14 dB, 18 dB below the bronze. LiDAR reflectivity the other way round: stone 23, bronze 4. The two takes without a statue seed are a CFAR effect (an object 0.6 m beyond the statue raises the threshold by 8–10 dB), not height or material. Report Figure 16 | What the 1.3 m object beside the statue was (not in the first take) |

## Hopes

| Hope | Status |
|---|---|
| Elevation from the phase between the two array rows | Not started (`experiments/x9-elevation` is a stub, no data of its own). The 11/19 stack, now registered, is the test set: the same statue with the radar's plane at seven known heights (0.83–1.49 m) |
| Backprojection across heights in place of stacking | Not started; item 3's registration exists now. Item 3 shows that without elevation, heights add no vertical information |
| A smooth metal panel against rough stone | No data |

## Schedule (from the proposal)

| Dates | Plan | State on 11/20 |
|---|---|---|
| Nov 20–22 | Multi-height capture, by day and one set at night | Night set done (11/19). No day set: multi-height is night only (decided 11/20) |
| Nov 23–30 | No captures. Stacking, night orbit reconstruction, material statistics | All three done (11/20) |
| Dec 1–6 | Multi-height dense depth; single against multi-height, CFAR against GRT, day against night | All done (11/20) |
| Dec 7–14 | Figures, 3D renders, report and video | Report draft exists; it is 16 pages in Overleaf against a target of about 10 |
