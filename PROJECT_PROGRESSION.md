# GaitSole → HoloTile: Project Progression Report

**Author:** Aashutosh Kushwaha
**Date:** 15 June 2026
**Scope:** Regional foot-sole force simulation, low-latency gait prediction, and an omnidirectional-floor (HoloTile) control simulation.

---

## 1. Executive Summary

This project builds an end-to-end pipeline that goes from **physics-based gait
simulation with detailed plantar (foot-sole) loading** → **a fast learned predictor
of motion and foot loading** → **a control simulation of a HoloTile omnidirectional
floor** for VR rehabilitation and robot/vehicle terrain training.

It has three connected parts, built in sequence:

1. **GaitSole simulation pipeline (OpenSim + Moco)** — simulates 2D and 3D
   running/walking gait and reads **per-region** ground-reaction force (heel /
   midfoot / forefoot / toe per foot) instead of a single whole-foot force. Each
   contact region behaves like an independent load-cell.
2. **Motion predictor (PyTorch)** — a lightweight model that observes a short
   history of lower-limb kinematics and predicts, at **sub-millisecond latency**,
   the next pose plus per-foot force/moment. Trained first on synthetic gait, then
   validated on **real treadmill data (Camargo et al. 2021)**.
3. **HoloTile simulation (MuJoCo)** — a physics simulation of Disney's HoloTile
   patented omnidirectional floor (US20180217662A1) that **fuses live-sensor and
   ML-predicted velocity** to keep a walking person centered on the floor.

The common thread: detailed plantar-load gait simulation feeds a real-time learned
predictor, which in turn drives a closed-loop floor-control simulation.

---

## 2. Motivation

Standard running-biomechanics models report **one** ground-reaction-force (GRF)
vector per foot. Many questions in footwear design, injury research, and orthotics
depend on **where under the foot** the load sits (heel strike vs. forefoot push-off).

- The **4-region contact model** exposes that spatial breakdown directly from a
  dynamically-consistent simulation.
- The **ML predictor** makes those quantities available in real time without
  re-solving an expensive optimal-control problem.
- The **HoloTile sim** demonstrates a downstream application: using anticipatory
  motion prediction to control a physical floor before sensor latency causes drift.

---

## 3. Timeline of Work

| Phase | Dates (2026) | What happened |
|-------|--------------|---------------|
| **0 — Environment & first models** | 19 May | OpenSim 4.5 + Moco + Python bindings set up; first scratch models. |
| **1 — 2D gait pipeline** | 19–29 May | Built and validated the 2D 4-region foot pipeline (scripts 01–08). |
| **2 — 3D model + predictive Moco** | 29–30 May | Built 3D Rajagopal2016 model with 4-region feet + fingers; ran the real 3D predictive solve (script 11). |
| **3 — Pivot to ML predictor** | 30 May | Paused the expensive 3D Moco solves; began the low-latency learned predictor (`motion_predictor/`). |
| **4 — Real data + GPU training** | 5–6 Jun | Processed Camargo et al. 2021 dataset; set up GPU server access; trained & evaluated the predictor. |
| **5 — HoloTile simulation** | 6 Jun → present | Built MuJoCo HoloTile floor; milestones M1–M5 complete (physics proof → velocity fusion). |

---

## 4. Phase 1 — The 2D Gait Simulation Pipeline

**Tooling:** OpenSim 4.5, Moco (optimal control), Python bindings (run via the
dedicated conda interpreter `E:/conda/envs/opensim_env/python.exe`).

The scripts are numbered and run in order. The **key idea** is to split the foot's
single contact region into **four independent contact regions** (heel, midfoot,
forefoot, toe) so that GRF can be read per region, like a grid of load-cells.

| # | Script | What it does | Output |
|---|--------|--------------|--------|
| 01 | `01_inspect_2d_gait.py` | Enumerate joints / coordinates / muscles / contacts of the shipped 2D model. | console inventory |
| 02 | `02_forward_sim_per_region_forces.py` | Forward-simulate and read each contact sphere's force vector. | `fwd_forces_forces.sto` |
| 03 | `03_upgrade_to_4_regions.py` | **Build `2D_gait_4regions.osim`** — splits front contact into midfoot + forefoot + toe. | `2D_gait_4regions.osim` |
| 04 | `04_read_force_sto.py` | Load ForceReporter `.sto`, plot per-region vertical force. | `per_region_GRF_right.png/.csv` |
| 05 | `05_predictive_running_moco.py` | Moco predictive **2D running** half-stride with anti-symmetric periodicity (~10–30 min solve). | `run_solution.sto` |
| 06 | `06_extract_grf_from_solution.py` | Replay the saved solution to extract per-region GRF without re-solving. | `run_GRF.sto` |
| 07 | `07_walk_driven_per_region_grf.py` | Drive the 4-region model with reference **walking** kinematics → per-region GRF (fast, reliable). | `walk_GRF.sto` |
| 08 | `08_tile_strides.py` | Mirror the half-stride to a full stride and tile into N strides. | `run_solution_Nstrides.sto`, `run_GRF_Nstrides.sto` |

**Data-processing flow (2D):**
`2D_gait.osim` → split contacts (03) → predictive solve (05) → extract per-region GRF (06) → tile into a multi-stride signal (08).

**Generated model:** `2D_gait_4regions.osim` (2D lower-limb model, 4 contact regions per foot).

---

## 5. Phase 2 — The 3D Model and Full Predictive Solve

| # | Script | What it does | Output |
|---|--------|--------------|--------|
| 09 | `09_build_rajagopal_arms_fingers.py` | **Build `Rajagopal2016_4regions_fingers.osim`** — full 3D body with 4-region feet + articulated fingers (14 single-DOF flex joints per hand). | `Rajagopal2016_4regions_fingers.osim` |
| 10 | `10_inspect_and_test_rajagopal.py` | Inspect the 3D model; generate a 2-second prescribed-motion preview (arms/fingers/hip) for GUI playback. | `rajagopal_arms_preview.sto` |
| 11 | `11_predictive_running_3d_moco.py` | Moco predictive **3D running** half-stride — the full physics solve (welds lower fingers/wrist for tractability; long runtime). | `run3d_solution.sto`, `run3d_solve_log.txt` |

**Cross-platform note:** `scripts_lightning/11_*` is a path-independent variant of the
3D solve (detects a `PROJECT_ROOT` env var) so the heavy solve can run unchanged on a
Linux cloud GPU instance as well as on Windows. The core solve logic is identical.

**Generated models / preprocessing:**
- `Rajagopal2016_4regions_fingers.osim` — the full articulated 3D model.
- `_rajagopal_for_moco.osim` — a Moco-tractable variant with lower-finger / wrist /
  forearm-pronation joints welded.

**Why this phase was paused:** the 3D optimal-control solve is expensive (minutes to
hours per solve), which motivated a fast **learned surrogate** — Phase 3.

---

## 6. Phase 3 & 4 — The Motion Predictor (ML)

**Goal:** observe a short window of lower-limb kinematics and predict the next pose
**plus per-foot force/moment** at real-time latency, so the costly optimal-control
solve does not have to run online.

### 6.1 Model architecture (`motion_predictor/model.py`)

- **Input:** a 26-dimensional per-frame lower-limb feature vector
  (8 leg joint angles + 3 pelvis orientation + 1 pelvis height + 2 root velocities
  + 12 velocities), over a **30-frame @ 100 Hz** history (0.3 s) → 780-D flattened.
- **Trunk:** all-MLP (siMLPe-style), 3 dense layers of 256 units with GELU,
  **~365k parameters** — deliberately small for CPU inference.
- **Three output heads** (predict the next 5 frames):
  1. **Pose** — next-frame joint-angle residuals (predict *changes*, then integrate).
  2. **Root velocity** — pelvis horizontal velocity (`tx`, `tz`).
  3. **Kinetics** — per-foot GRF (`Fx, Fy, Fz`), free moment (`Mz`), centre of
     pressure (`COPx, COPz`).

### 6.2 Data pipeline (`motion_predictor/data.py`)

1. **Synthetic gait generator** — plausible structured walking (6 s @ 100 Hz,
   randomized stride frequency/speed, heel→toe COP progression, mild sensor noise).
   Used to validate the full machinery end-to-end before any real data.
2. **Featurize** — compute per-frame velocities via finite difference; stack pose +
   root-velocity + pose-velocity into the 26-D feature.
3. **Window** — sliding window: 30-frame history → 5-frame-ahead targets
   (pose as residuals, kinetics absolute).
4. **Normalize** — per-channel z-score; stats saved to `runs/norm_stats.npz` and
   inverted at inference so predictions come back in physical units.

### 6.3 Training (`motion_predictor/train.py`)

- **Optimizer:** AdamW (lr 1e-3, weight-decay 1e-5), batch 256, 40 epochs default,
  80/20 train/val split.
- **Loss:** weighted multi-head MSE — `1.0·pose + 1.0·rootvel + 1.5·kinetics`
  (kinetics up-weighted because force is the harder, higher-value target).
- **Checkpoint:** best-val model → `runs/predictor.pt`.

### 6.4 Real data: Camargo et al. 2021 (Phase 4)

Moved from synthetic to **real treadmill gait** (subject AB14):

- `camargo_table_to_csv.m` — MATLAB-Online converter (`.mat` table objects can't be
  read by SciPy/pymatreader) producing CSV of 200 Hz inverse-kinematics + 1000 Hz
  force-plate data, with body-weight-normalized GRF and lab-frame COP.
- `camargo.py` — Python loader resampling IK (200 Hz) and force plate (1000 Hz) to a
  common **100 Hz** grid, converting angles to radians and computing the free
  vertical moment about COP.
- Raw archives: `camargo_convert.zip` (69 MB, `.mat`), `camargo_csv.zip` (77 MB, CSV).

### 6.5 GPU server training

- `install_gpu_key.ps1` configured passwordless SSH (Ed25519) to a 3× GTX 1080 Ti
  server (`172.20.160.50`, CUDA 11.6). A `~/venvs/motion` env with `torch 1.13.1+cu116`
  was created and the `motion_predictor/` tree synced; a GPU smoke test passed.
- The cross-platform `scripts_lightning/` variant lets the heavy work run on Linux GPU.

### 6.6 Inference & latency (`motion_predictor/infer_stream.py`)

- Simulates real-time deployment: frames arrive sequentially, a rolling 30-frame
  window predicts the next 5 frames each tick; timed with `perf_counter` after warm-up.
- **Result: ~0.9 ms/prediction (~1100 fps) on a laptop CPU** — far inside a 100 Hz
  control budget.

---

## 7. Motion-Predictor Results

**Validated on held-out Camargo 2021 windows (next-frame MAE):**

| Quantity | Right | Left |
|----------|-------|------|
| Hip flexion | 0.082° | 0.081° |
| Hip adduction | 0.057° | 0.062° |
| **Knee angle** (primary) | **0.130°** | 0.106° |
| Ankle angle | 0.094° | 0.090° |
| **Vertical GRF** (primary) | **0.027 BW** | 0.025 BW |
| AP / ML GRF | 0.007–0.010 BW | — |
| Free moment Mz | 0.005–0.012 BW·m | — |
| COP (AP / lateral) | 0.016 m / 0.059 m | — |

Pelvis tilt/list/rotation MAE ≈ 0.026–0.041°; pelvis height MAE ≈ 0.13 mm.

**Result artifacts (`motion_predictor/runs/`):**

| File | Size | Meaning |
|------|------|---------|
| `predictor.pt` | 1.46 MB | trained model checkpoint |
| `norm_stats.npz` | 2.4 KB | normalization stats |
| `eval_metrics.csv` | ~1 KB | per-channel next-frame MAE (table above) |
| `eval_curves.png` | 352 KB | predicted vs actual: knee/hip/ankle, GRF, COP |
| `eval_horizon.png` | 64 KB | error vs lead-time across the 5-frame horizon |
| `camargo_actual.mot` / `camargo_predicted.mot` | 137 KB each | ground-truth vs predicted joint angles as OpenSim motion |
| `AB14.osim` | 625 KB | subject skeleton used for visualization |
| `skeleton_compare.mp4` / `_50ms.mp4` | ~3 MB each | side-by-side real vs predicted skeleton (full / 50 ms horizon) |
| `opensim_sync.mp4` | 3.5 MB | OpenSim model driven by real vs predicted kinematics |

---

## 8. Phase 5 — The HoloTile Simulation (MuJoCo)

**Goal:** simulate Disney's HoloTile patented omnidirectional floor
(US20180217662A1) — a grid of tilted, spinning disk assemblies that can drive a
person in any direction while keeping them centered — and demonstrate that
**fusing live-sensor velocity with ML-predicted velocity** beats either alone.

### 8.1 Architecture

- **Geometry:** 0.30 m square tiles (configurable to 8×8 floors), 5×5 = 25 disks per
  tile; each disk is a sphere spinning on a **35° tilted axis** (patent range 15–60°),
  so the spin produces lateral surface velocity with no vertical pumping.
- **Two contact models:**
  1. **Production floor** — frictionless tile pads with a **moving-surface
     (belt-drive) friction model** (`F = −BELT_K·slip`, saturated at `µ·N`). Fast,
     scales to full floors, numerically stable at high spin.
  2. **Physical-disk floor** — real spinning disk bodies with true friction;
     proof-of-concept, stable only at low spin.
- **Per-tile control:** azimuth servo (orientation, slew-limited) + spin servo
  (velocity, rate-limited). Surface-velocity command:
  **`v_surface = w_live · v_live + w_model · v_model`**, where `v_live` is a delayed,
  noisy synthetic sensor and `v_model` is the anticipatory ML-predictor estimate.
- **Predictor bridge** — streams the trained `motion_predictor` checkpoint; median
  inference **0.3–0.4 ms** on CPU.

### 8.2 Milestones

| Milestone | Title | Status | Key script(s) | Result |
|-----------|-------|--------|---------------|--------|
| **M1** | Physics proof | ✅ Done | `check_m1.py` | Puck heading tracks commanded azimuth; spin reversal flips heading ~180°. Verdict **PASS**. |
| **M2** | Production floor + visual overlay | ✅ Done | `run_demo.py`, `disk_overlay.py`, `mjcf_builder.py` | 8×8 frictionless pads with disk overlay; scripted azimuth/spin → `holotile_m2.mp4`. |
| **M3** | Predictor-driven 3D skeleton | ✅ Done | `run_walk.py`, `predictor_bridge.py`, `skeleton3d.py` | Trained predictor streams gait; 3D skeleton stands/walks on spinning floor → `holotile_walk.mp4`. |
| **M4** | Floor centering control | ✅ Done | `run_control.py`, `floor_controller.py` | Two-foot stance/swing; controller keeps walker centered on a 7×7 floor → `holotile_control.mp4`. |
| **M5** | Velocity fusion (live + model) | ✅ Done | `fusion.py`, `sensors.py`, `intended_velocity.py` | Weighted blend; sweep shows **best `w_model ≈ 0.5–0.75`** minimizes turn drift. |
| **M6** | Per-foot force / COP mapper | 🔲 Scoped | (referenced in `predictor_bridge.py`) | Not yet implemented. |
| **M7** | Real-time sensor I/O | 🔲 Scoped | — | Live camera/IMU/lidar feed; currently a synthetic sensor stub. |

### 8.3 HoloTile results

- **M1 physics proof:** at spin = 24 rad/s, commanded azimuth 0/90/180/270° produces
  matching puck heading; reverse spin flips heading ~180°. Controllable & reversible.
- **M4 centering:** 10 s walk at 0.9 m/s — controller **ON** keeps pelvis drift
  ≈ 0.10–0.15 m; **OFF** it drifts 0.5–1.0 m off the floor.
- **M5 fusion sweep** (walk + sharp 90° turn): live-only (`w_model=0`) lags the turn
  (turn-max drift ≈ 0.25 m); the fused blend at **`w_model ≈ 0.5–0.75`** gives the
  best turn response (turn-max drift ≈ 0.14–0.15 m). **The fused estimate beats
  either source alone** — the project's central thesis.

**Result artifacts (`holotile_sim/output/`):**

| File | Size | Meaning |
|------|------|---------|
| `holotile_control.mp4` | 7.4 MB | walker stays centered as tiles re-orient/spin (M4) |
| `holotile_m2.mp4` | 1.9 MB | production floor, disks rotating under scripted command (M2) |
| `holotile_walk.mp4` | 994 KB | 3D skeleton walking on spinning-disk floor (M3) |
| `rigid_spin_demo.mp4` | 2.7 MB | real spinning disks, low-speed literal physics |
| `control_trajectory.png` | 26 KB | pelvis path: centered (ON) vs drifting (OFF) |
| `fusion_sweep.png` | 45 KB | drift vs `w_model` curve — optimal blend |
| `control_frame.png`, `m2_frame.png`, `walk_frame.png`, `rigid_spin_frame.png` | — | single rendered frames |

---

## 9. Consolidated File Inventory

**Generated models (`.osim`):**
- `2D_gait_4regions.osim` — 2D model, 4 contact regions per foot (script 03)
- `Rajagopal2016_4regions_fingers.osim` — full 3D model, 4-region feet + fingers (script 09)
- `_rajagopal_for_moco.osim` — Moco-tractable welded variant (script 11)
- `motion_predictor/runs/AB14.osim` — Camargo subject skeleton for visualization

**Simulation results (`output/`):**
`run_solution.sto`, `run_GRF.sto`, `run_solution_Nstrides.sto`, `run_GRF_Nstrides.sto`,
`walk_GRF.sto`, `fwd_forces_forces.sto`, `per_region_GRF_right.csv/.png`,
`run3d_solution.sto`, `run3d_solve_log.txt`, `rajagopal_arms_preview.sto`.

**ML predictor (`motion_predictor/`):** code (`config/data/model/train/infer_stream/evaluate.py`),
real-data tooling (`camargo_table_to_csv.m`, `camargo.py`), and `runs/` results (Section 7).

**HoloTile (`holotile_sim/`):** simulation code (`mjcf_builder.py`, `sim_world.py`,
`floor_controller.py`, `fusion.py`, `sensors.py`, `predictor_bridge.py`, `skeleton3d.py`,
run scripts) + `output/` videos and plots (Section 8.3). Patent reference under `Holotile/`.

**Datasets:** `camargo_convert.zip` (69 MB), `camargo_csv.zip` (77 MB) — Camargo et al. 2021.

---

## 10. Status and Future Work

**Done:**
- 2D 4-region foot pipeline (scripts 01–08), validated.
- 3D articulated model + full predictive 3D Moco solve (scripts 09–11).
- ML motion predictor trained and **validated on real Camargo data** (sub-degree pose
  MAE, ~0.03 BW GRF MAE) at **~0.9 ms/prediction**.
- HoloTile MuJoCo sim, milestones M1–M5, including the live+model velocity-fusion result.

**Next:**
- **Per-region foot force in the predictor** (heel/midfoot/forefoot/toe) trained on the
  OpenSim 4-region contact data — closing the loop between the simulation and the
  predictor (HoloTile M6 depends on this).
- Real-time sensor I/O for HoloTile (M7): live camera / IMU / mocap stream.
- Whole-body momentum output head; configurable longer prediction horizon.

**Caveats:** paths are currently hard-coded to an `E:/OpenSim/...` layout; finger
articulation is simplified to flex/extend hinges; the synthetic-data results only prove
the machinery — the real accuracy numbers come from the Camargo validation.

---

## 11. Key References

- Rajagopal et al. (2016) — full-body musculoskeletal model.
- Camargo et al. (2021) — multimodal lower-limb treadmill gait dataset (training/validation).
- Schreiber & Moissenet (2019) — reference gait dataset (candidate for future training).
- Disney HoloTile patent **US20180217662A1** — omnidirectional-floor mechanism.
- Included PDFs: `28 Submission.pdf`, `fcomp-01-00012.pdf`.
</content>
</invoke>
