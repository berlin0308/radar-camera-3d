# Radar–camera 3D camera (15-663 final project)

Classical FMCW radar (TI AWR1843 + DCA1000 on the WiSE Lab red-rover rig) plus a fisheye camera,
turned into a 3D camera: raw ADC → range / Doppler / angle → 3D points → camera-guided dense depth,
evaluated against LiDAR.

**[Experiment reports →](EXPERIMENTS.md)** every experiment with its numbers, figures and what is still pending.

Author: Po-Lin Chen. Contributor: Yining (extensions, 3D reconstruction; acknowledged in the report).

## Layout
| Path | What |
|---|---|
| `shoot.sh`, `sync.sh`, `validate.sh`, `check_valid.py` | rig-day recording and data checks |
| `AGENDA_session2.md` | rig-day procedure and shot list for the next sessions |
| `notes/shots.csv` | which trace is which shot |
| `notes/fig/` | figures used in the Notebook deck |
| `src/radar/` | own radar chain: `adc.py` → `range_doppler.py` → `angle.py` → `cfar.py` → `pointcloud.py`; `python -m src.radar <trace>` runs it on one frame |
| `tests/` | each stage against `xwr.rsp` on the same frame (`python -m pytest tests/ -q`) |
| `src/ext/<X-id>/` | extensions (see CONTRIBUTING.md) |
| `figures/` | report figures, named by trace and frame |
| `EXPERIMENTS.md`, `experiments/<id>/` | index of all experiments; one report per experiment (template: `experiments/_TEMPLATE.md`) |
| `data/raw/<trace>/` | captures, **not in git** |

## Setup
```bash
uv venv -p 3.12 && source .venv/bin/activate
uv pip install "roverd[video] @ git+https://github.com/RadarML/red-rover.git#subdirectory=format" xwr tqdm matplotlib scipy pyyaml pytest
```
Without `uv` (e.g. on nunga, where the system Python is PEP 668 managed): `python3 -m venv .venv && .venv/bin/pip install ...` with the same packages.

Put the captures under `data/raw/` with their original trace names (e.g. `data/raw/2026-10-21.15-58-19/`); symlinks are fine (on nunga they point into `/data/christineh/DATASET/radarhd/`).

```bash
python -m pytest tests/ -q                                                   # stages vs xwr.rsp
python -m src.radar data/raw/2026-10-29.15-49-40 --time 720 --accumulate 20  # point cloud of one second
```
