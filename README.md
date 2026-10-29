# Radar–camera 3D camera (15-663 final project)

Classical FMCW radar (TI AWR1843 + DCA1000 on the WiSE Lab red-rover rig) plus a fisheye camera,
turned into a 3D camera: raw ADC → range / Doppler / angle → 3D points → camera-guided dense depth,
evaluated against LiDAR.

Author: Po-Lin Chen. Contributor: Yining (extensions, 3D reconstruction; acknowledged in the report).

## Layout
| Path | What |
|---|---|
| `shoot.sh`, `sync.sh`, `validate.sh`, `check_valid.py` | rig-day recording and data checks |
| `AGENDA_session2.md` | rig-day procedure and shot list for the next sessions |
| `notes/shots.csv` | which trace is which shot |
| `notes/fig/` | figures used in the Notebook deck |
| `src/` | main pipeline (Po-Lin) |
| `src/ext/<X-id>/` | extensions (see CONTRIBUTING.md) |
| `experiments/<X-id>/` | results of each extension: figures + README |
| `data/raw/<trace>/` | captures, **not in git** |

## Setup
```bash
uv venv -p 3.12 && source .venv/bin/activate
uv pip install "roverd[video] @ git+https://github.com/RadarML/red-rover.git#subdirectory=format" xwr tqdm
```
Put the captures under `data/raw/` with their original trace names (e.g. `data/raw/2026-10-21.15-58-19/`).
