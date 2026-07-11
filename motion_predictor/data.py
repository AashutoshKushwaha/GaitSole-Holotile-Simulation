"""
Data layer for the legs-only motion predictor.

Provides:
  * a documented INTERMEDIATE TRIAL FORMAT (a dict of 1-D numpy arrays, one per
    column in config.RAW_COLS, plus 'time'), so real data and synthetic data
    flow through identical code;
  * make_synthetic_trial(): plausible structured gait (so the full
    train -> infer pipeline can be validated with NO download, on CPU);
  * featurize(): turns a raw trial into per-frame input features + targets,
    encoding the representation choices in config.py (root as velocity, pose
    velocities appended, kinetics passed through);
  * GaitWindows: a torch Dataset of sliding (history -> future) windows;
  * normalization-stat computation / application (saved so inference matches).

REAL DATA (later): write a small adapter that returns the same dict for one
trial. Two recommended sources for legs + GRF:
  * Schreiber & Moissenet 2019 (open, figshare): markers + force plates. Joint
    ANGLES come from running OpenSim Inverse Kinematics on the markers (you
    already have OpenSim); FORCE/COP/free-moment come straight from the force
    plate analog channels. See README.
  * Camargo et al. 2021 (Georgia Tech EPIC lab, open): already provides
    processed lower-limb joint ANGLES and GRF/kinetics -> skips the IK step.
"""

import numpy as np

import config as C


# ===========================================================================
# Synthetic gait (for pipeline validation without any download)
# ===========================================================================
def make_synthetic_trial(duration_s=6.0, fps=None, rng=None):
    """Return a raw-trial dict of structured, gait-like signals. Not
    biomechanically exact -- just rich/learnable enough to validate the whole
    pipeline (clear left/right phasing, stance-phase GRF, heel->toe COP)."""
    fps = fps or C.FPS
    rng = rng or np.random.default_rng()
    n = int(duration_s * fps)
    t = np.arange(n) / fps

    # Per-trial variation so the model can't just memorise one cycle.
    stride_hz = 0.9 * rng.uniform(0.85, 1.15)          # full strides / second
    speed = 1.3 * rng.uniform(0.8, 1.25)               # m/s forward
    w = 2 * np.pi * stride_hz
    d = {}

    def ang(amp_deg, phase, harm2=0.0):
        a = np.deg2rad(amp_deg)
        return a * (np.sin(w * t + phase) + harm2 * np.sin(2 * w * t + phase))

    # Legs: left lags right by half a stride (pi).
    for s, ph in (("r", 0.0), ("l", np.pi)):
        d[f"hip_flexion_{s}"] = ang(30, ph) + np.deg2rad(5)
        d[f"hip_adduction_{s}"] = ang(6, ph + 0.4)
        # Knee: small stance flex + big swing flex -> rectified-ish double bump.
        knee = np.deg2rad(20) * (1 - np.cos(w * t + ph)) + np.deg2rad(25) * np.clip(
            np.sin(w * t + ph + 0.6), 0, None) ** 2
        d[f"knee_angle_{s}"] = knee
        d[f"ankle_angle_{s}"] = ang(12, ph + 1.0, harm2=0.3)

    # Pelvis: small orientation oscillations, vertical bob at 2x stride.
    d["pelvis_tilt"] = ang(3, 0.2)
    d["pelvis_list"] = ang(4, np.pi / 2)
    d["pelvis_rotation"] = ang(5, 0.0)
    d["pelvis_ty"] = 0.94 + 0.02 * np.cos(2 * w * t)
    # Root horizontal: forward progression + small lateral sway.
    d["pelvis_tx"] = speed * t
    d["pelvis_tz"] = 0.02 * np.sin(w * t)

    # Foot GRF: each foot in stance ~60% of its cycle, half-cycle offset.
    def stance_window(ph):
        # 1 during stance, 0 during swing, smooth edges.
        phase = (w * t + ph) % (2 * np.pi)
        # stance from 0..1.2pi (~60%)
        s_on = np.clip(np.sin(phase / 1.2), 0, None)
        return (phase < 1.2 * np.pi) * s_on

    for s, ph in (("r", 0.0), ("l", np.pi)):
        st = stance_window(ph)
        # vertical: double-hump ~1.1 BW during stance
        vy = st * (1.0 + 0.15 * np.cos(2 * w * t + ph)) * 1.1
        d[f"grf_y_{s}"] = vy
        d[f"grf_x_{s}"] = st * 0.2 * np.sin(w * t + ph)          # AP push/brake
        d[f"grf_z_{s}"] = st * 0.05 * np.cos(w * t + ph)         # ML
        # COP travels heel(-0.05) -> toe(+0.15) across stance; mz small free moment
        prog = np.clip((((w * t + ph) % (2 * np.pi)) / (1.2 * np.pi)), 0, 1)
        d[f"cop_x_{s}"] = np.where(st > 0, -0.05 + 0.20 * prog, 0.0)
        d[f"cop_z_{s}"] = np.where(st > 0, 0.02 * (1 if s == "r" else -1), 0.0)
        d[f"mz_{s}"] = st * 0.02 * np.sin(w * t + ph)

    # Light sensor noise.
    for k in C.RAW_COLS:
        scale = 0.01 if not k.startswith("grf") else 0.02
        d[k] = d[k] + rng.normal(0, scale, size=n)

    d["time"] = t
    return d


def make_synthetic_dataset(n_trials=40, seed=C.SEED):
    rng = np.random.default_rng(seed)
    return [make_synthetic_trial(rng=rng) for _ in range(n_trials)]


# ===========================================================================
# Featurization
# ===========================================================================
def featurize(trial):
    """Raw-trial dict -> arrays used for windowing.

    Returns dict with:
      in_feat : [N, IN_DIM]  per-frame input features
      pose    : [N, 12]      absolute pose (for residual targets / integration)
      rootvel : [N, 2]       root horizontal velocity (target, m/s)
      kin     : [N, 12]      force + moment targets
    """
    pose = np.stack([trial[c] for c in C.POSE_COLS], axis=1)           # [N,12]
    rootpos = np.stack([trial[c] for c in C.ROOT_POS_COLS], axis=1)    # [N,2]
    kin = np.stack([trial[c] for c in C.KINETIC_COLS], axis=1)         # [N,12]

    # Velocities by finite difference (first frame replicated to keep length).
    def vel(x):
        v = np.diff(x, axis=0) * C.FPS
        return np.concatenate([v[:1], v], axis=0)

    rootvel = vel(rootpos)                                             # [N,2]
    pose_vel = vel(pose)                                              # [N,12]

    in_feat = np.concatenate([pose, rootvel, pose_vel], axis=1)        # [N,26]
    return {"in_feat": in_feat, "pose": pose, "rootvel": rootvel, "kin": kin}


def build_windows(trials):
    """Slide a (T_IN history -> H_OUT future) window over every trial.

    Targets:
      pose_resid[h] = pose[t+h] - pose[t-1]   (predict change from last obs)
      rootvel[h]    = rootvel[t+h]
      kin[h]        = kin[t+h]
    Returns numpy arrays:
      X        [M, T_IN, IN_DIM]
      Y_pose   [M, H_OUT, 12]
      Y_rvel   [M, H_OUT, 2]
      Y_kin    [M, H_OUT, 12]
    """
    X, Yp, Yr, Yk = [], [], [], []
    for tr in trials:
        f = featurize(tr)
        inf, pose, rvel, kin = f["in_feat"], f["pose"], f["rootvel"], f["kin"]
        N = inf.shape[0]
        for t in range(C.T_IN, N - C.H_OUT + 1):
            X.append(inf[t - C.T_IN:t])
            last = pose[t - 1]
            Yp.append(pose[t:t + C.H_OUT] - last)
            Yr.append(rvel[t:t + C.H_OUT])
            Yk.append(kin[t:t + C.H_OUT])
    return (np.asarray(X, np.float32), np.asarray(Yp, np.float32),
            np.asarray(Yr, np.float32), np.asarray(Yk, np.float32))


# ===========================================================================
# Normalization (z-score per channel; saved for inference)
# ===========================================================================
def fit_stats(X, Yp, Yr, Yk):
    def ms(a):
        flat = a.reshape(-1, a.shape[-1])
        return flat.mean(0), flat.std(0) + 1e-6
    s = {}
    s["x_mean"], s["x_std"] = ms(X)
    s["p_mean"], s["p_std"] = ms(Yp)
    s["r_mean"], s["r_std"] = ms(Yr)
    s["k_mean"], s["k_std"] = ms(Yk)
    return {k: v.astype(np.float32) for k, v in s.items()}


def apply_stats(arr, mean, std):
    return (arr - mean) / std


def invert_stats(arr, mean, std):
    return arr * std + mean


def save_stats(path, stats):
    np.savez(path, **stats)


def load_stats(path):
    d = np.load(path)
    return {k: d[k] for k in d.files}
