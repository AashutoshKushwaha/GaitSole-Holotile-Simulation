"""
Central configuration for the legs-only motion predictor.

The model OBSERVES a short history of lower-limb skeleton kinematics and
PREDICTS, for the next few frames:
   * the next pose (joint-angle residuals + root horizontal velocity), and
   * the foot ground-reaction FORCE and MOMENT (the "force & momentum from the
     foot" the project needs).

Representation rationale (see project notes):
   * Joint ANGLES (not 3D positions) -> low-dimensional, subject/scale
     invariant, and directly compatible with OpenSim .sto coordinates.
   * Root (pelvis) horizontal motion is encoded as VELOCITY, never absolute
     position (absolute tx/tz grows without bound and destroys generalization).
   * Outputs are next-frame RESIDUALS for pose (predict the change, integrate)
     -> smooth, avoids the discontinuity jump at the observe->predict boundary.

Everything downstream (data, model, train, infer) imports these names, so the
variable set is defined in exactly one place. Adding the 4 foot regions or body
momentum later = extend the lists here + the matching head in model.py.
"""

import os

# ---------------------------------------------------------------------------
# Variable definitions  (the heart of the spec)
# ---------------------------------------------------------------------------
SIDES = ["r", "l"]

# Lower-limb joint angles we observe & predict (sagittal-dominant + frontal hip).
LEG_JOINTS = ["hip_flexion", "hip_adduction", "knee_angle", "ankle_angle"]
LEG_ANGLE_COLS = [f"{j}_{s}" for s in SIDES for j in LEG_JOINTS]      # 8

# Pelvis orientation + height (bounded, safe to use as absolute values).
PELVIS_ORIENT_COLS = ["pelvis_tilt", "pelvis_list", "pelvis_rotation"]  # 3
PELVIS_HEIGHT_COLS = ["pelvis_ty"]                                      # 1

# The full "pose" we predict residuals for.
POSE_COLS = LEG_ANGLE_COLS + PELVIS_ORIENT_COLS + PELVIS_HEIGHT_COLS    # 12

# Raw horizontal root position, present in a trial only so we can DIFFERENTIATE
# it into velocity. Never fed/predicted as an absolute value.
ROOT_POS_COLS = ["pelvis_tx", "pelvis_tz"]                              # 2
ROOT_VEL_COLS = ["pelvis_tx_vel", "pelvis_tz_vel"]                      # 2 (derived)

# Kinetic targets from the foot: force (Fx,Fy,Fz) and moment (free moment Mz +
# centre of pressure x/z) per foot. Forces in body-weight (BW); moments BW*m;
# COP in metres. Per-FOOT here (what force plates give); per-REGION (heel/
# midfoot/forefoot/toe) is a future extension once trained on OpenSim sim data.
FORCE_COLS = [f"grf_{a}_{s}" for s in SIDES for a in ["x", "y", "z"]]   # 6
MOMENT_COLS = [f"{m}_{s}" for s in SIDES for m in ["mz", "cop_x", "cop_z"]]  # 6
KINETIC_COLS = FORCE_COLS + MOMENT_COLS                                # 12

# Columns a raw per-trial table MUST contain (besides 'time').
RAW_COLS = POSE_COLS + ROOT_POS_COLS + KINETIC_COLS

# ---------------------------------------------------------------------------
# Derived dimensions
# ---------------------------------------------------------------------------
# Per-frame INPUT feature vector = [pose, root velocity, pose velocity].
IN_DIM = len(POSE_COLS) + len(ROOT_VEL_COLS) + len(POSE_COLS)          # 26
# Per-frame OUTPUT groups.
OUT_POSE_DIM = len(POSE_COLS)        # 12  (residuals)
OUT_ROOTVEL_DIM = len(ROOT_VEL_COLS) # 2   (direct, already a rate)
OUT_KIN_DIM = len(KINETIC_COLS)      # 12  (force + moment)

# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------
FPS = 100.0          # all trials resampled to this rate
T_IN = 30            # observation window length (frames) -> 0.30 s of history
H_OUT = 5            # prediction horizon (frames) -> 0.05 s ahead (min-latency)

HIDDEN = 256
N_LAYERS = 3
DROPOUT = 0.0

LR = 1e-3
WEIGHT_DECAY = 1e-5
EPOCHS = 40
BATCH = 256
VAL_FRACTION = 0.2
SEED = 0

# Loss weights across the three output heads (kinetics up-weighted a touch
# because foot force/moment is a primary deliverable).
W_POSE = 1.0
W_ROOTVEL = 1.0
W_KIN = 5.0      # up-weighted: force is the weak channel

# ---------------------------------------------------------------------------
# Paths (auto-detected; portable laptop <-> Lightning)
# ---------------------------------------------------------------------------
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(PROJECT_DIR, "data")
RUNS_DIR = os.path.join(PROJECT_DIR, "runs")
CKPT_PATH = os.path.join(RUNS_DIR, "predictor.pt")
STATS_PATH = os.path.join(RUNS_DIR, "norm_stats.npz")
