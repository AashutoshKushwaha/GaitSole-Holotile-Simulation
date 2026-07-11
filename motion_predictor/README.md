x# Legs-only motion predictor

Observes a short history of lower-limb skeleton kinematics and predicts, at
**minimum latency**, the next few frames of:

- **next pose** — lower-limb joint-angle residuals + pelvis orientation/height + root horizontal velocity
- **foot force** — ground-reaction force `Fx, Fy, Fz` per foot (body-weight units)
- **foot moment** — free moment `Mz` + centre of pressure `COPx, COPz` per foot

It's a lightweight all-MLP model (siMLPe-style) → sub-millisecond inference on CPU.

## Why these variables
- **Joint angles, not 3D points** — low-dimensional, subject/scale invariant, and identical to OpenSim `.sto` coordinates (so a prediction replays on your skeleton).
- **Root as velocity, never absolute position** — absolute `pelvis_tx/tz` grows unbounded and wrecks generalization.
- **Predict residuals (the change), integrate** — smooth, no jump at the observe→predict boundary.

Exact variable lists live in `config.py` (one place). Per-frame input = 26 numbers; outputs split into pose (12) + root velocity (2) + kinetics (12).

## Install
```bash
pip install -r requirements.txt        # torch + numpy
```

## Validate the pipeline now (synthetic gait, no download)
```bash
python train.py                 # trains on synthetic gait, saves runs/predictor.pt
python infer_stream.py          # streaming demo + latency benchmark
```
Reference result on a laptop CPU: **median ~0.9 ms / prediction (~1100 fps capacity)**, next-frame right-knee error ~0.7°, vertical GRF error ~0.03 BW on held-out synthetic. (Synthetic data only proves the machinery — real accuracy comes from real data below.)

## Train for real on a GPU (Lightning.ai)
This model genuinely benefits from a GPU (unlike the OpenSim Moco work):
```bash
python train.py --data <dir_of_trials> --device cuda --epochs 100
```
You must implement `load_real(path)` in `train.py` — it returns a list of
**raw-trial dicts** in the same format as `data.make_synthetic_trial()`:
each dict has key `time` plus every column in `config.RAW_COLS`, as 1-D numpy
arrays resampled to `config.FPS` (100 Hz).

`RAW_COLS` = the 12 pose columns + `pelvis_tx, pelvis_tz` + the 12 force/moment columns.

### Getting real data into that format
**Option A — Schreiber & Moissenet 2019** (open, figshare; markers + force plates)
1. Scale an OpenSim model to each subject; run **Inverse Kinematics** on the markers → joint-angle `.sto` (gives the 12 pose columns + pelvis tx/tz/ty).
2. Take **force / COP / free-moment** straight from the force-plate analog channels (assign to right/left foot). Normalize force by body weight.
3. Resample both to 100 Hz, package into the dict. (You already have OpenSim, so the IK step is in your wheelhouse.)

**Option B — Camargo et al. 2021 (Georgia Tech EPIC lab)** (open)
Already provides processed lower-limb **joint angles AND GRF/kinetics** across level/ramp/stair walking → skips the IK step; just map their column names to `RAW_COLS`.

## Files
| file | role |
|---|---|
| `config.py` | all variables, dims, hyperparameters, paths |
| `data.py` | synthetic gait, featurization, windowing, normalization |
| `model.py` | `MotionPredictor` — MLP trunk + pose/rootvel/kinetics heads |
| `train.py` | training loop (synthetic by default; `--data` for real) |
| `infer_stream.py` | real-time streaming inference + latency benchmark |

## Roadmap (agreed: add as we go)
- **Per-region foot force** (heel/midfoot/forefoot/toe) — extend `FORCE_COLS` + train on your OpenSim contact-sim data (force plates only give per-foot).
- **Whole-body linear/angular momentum** output head.
- **Longer horizon** (configurable; currently next ~50 ms).
- **Live input source** (OpenCap video / IMU / mocap stream) feeding `infer_stream.py`.
