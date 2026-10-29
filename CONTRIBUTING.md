# Contributing (for Yining)

Thank you for helping. The main pipeline is Po-Lin's course work, so the rules keep authorship clear.

## Workflow
1. `git checkout main && git pull`, then one branch per topic: `yining/<X-id>-<short-name>`
   (e.g. `yining/x5-accumulation`, `yining/360-recon`).
2. Code goes in `src/ext/<X-id>/`; results go in `experiments/<X-id>/` with a `README.md`
   (question, method, data used, numbers, figures, one-line conclusion).
3. Import the main pipeline from `src/`, do not edit it. If something there needs to change,
   say so in the PR description and Po-Lin decides.
4. Open a pull request into `main`; Po-Lin reviews and merges. Keep PRs small.
5. Each figure in `experiments/` lists the command and commit that produced it.

## Data
Captures are not in git. Po-Lin shares a copy of `data/raw/`; keep the same folder names.
Use `notes/shots.csv` to find which trace is which shot. Camera frames contain passers-by:
do not publish frames without anonymizing faces and plates.

## Topics
| ID | Topic |
|---|---|
| 360 | 360° walk-around reconstruction (GHC loop, spin in place): poses from LiDAR SLAM or SfM, fused radar + camera cloud, web viewer |
| X5 | Multi-frame accumulation: density vs number of frames, with pose alignment |
| X7 | Confidence map for dense depth, calibrated against LiDAR |
| X9 | Is the radar elevation trustworthy? Check against LiDAR |
| M6-eval | Evaluation scripts: dense depth vs mono depth vs LiDAR, by distance bin |
