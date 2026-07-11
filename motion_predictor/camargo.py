"""
Camargo et al. 2021 dataset adapter for the legs-only motion predictor.

Reads the CSVs exported from the dataset's MATLAB v7 *table* .mat files
(see camargo_table_to_csv.m -- the raw .mat tables are unreadable by any
pure-Python lib, so we convert them to CSV with MATLAB Online once).

Produces the SAME raw-trial dict format as data.make_synthetic_trial():
a dict with key 'time' plus every column in config.RAW_COLS, as 1-D float
arrays resampled to config.FPS (100 Hz).

Mapping from the real data (verified against AB14 treadmill trials):
  * IK  @200Hz: OpenSim coordinate names in DEGREES. Angles -> radians;
    pelvis_ty / pelvis_tx / pelvis_tz are lengths (metres), left as-is.
  * FP  @1000Hz: per-foot Treadmill_{R,L}_{vx,vy,vz}=force(N),
    {px,py,pz}=COP(m), {moment_x,y,z}=moment about plate origin (N*m).
    Vertical axis is Y (matches config: grf_y vertical, grf_x AP, grf_z ML).
  * Forces & free moment normalized by body weight (mass*g). COP kept in m.
  * Free vertical moment about the COP:  Tz = M_y + (px*Fz - pz*Fx).
  * IK and FP share one lab clock ('Header'); both are linearly resampled
    onto a common 100 Hz grid over their overlapping interval.

Caveats (documented, not bugs):
  * Treadmill data: pelvis_tx barely changes (subject stays in place), so the
    root horizontal velocity target is ~0. Real locomotion speed lives in the
    'conditions' belt-speed channel (not ingested yet). Fine for v1.
  * COP is in lab/treadmill frame (absolute position on the belt), not
    foot-relative; per-channel z-scoring removes the offset at train time.
"""

import glob
import os

import numpy as np
import pandas as pd

import config as C

GRAVITY = 9.81  # m/s^2

# IK columns that are ANGLES -> convert degrees to radians. Lengths
# (pelvis_ty height, pelvis_tx/tz root position) stay in metres.
_ANGLE_COLS = set(C.LEG_ANGLE_COLS + C.PELVIS_ORIENT_COLS)

# Default subject these exported CSVs belong to (single-subject export for now).
DEFAULT_SUBJECT = "AB14"

_SUBJECT_INFO = "camargo_convert__SubjectInfo.csv"


def _subject_mass_kg(csv_dir, subject):
    si = pd.read_csv(os.path.join(csv_dir, _SUBJECT_INFO))
    row = si[si["Subject"] == subject]
    if row.empty:
        raise ValueError(f"Subject {subject!r} not found in {_SUBJECT_INFO}; "
                         f"available: {list(si['Subject'])}")
    return float(row["Weight"].iloc[0])  # 'Weight' column is mass in kg


def _resample(t_src, y_src, t_grid):
    return np.interp(t_grid, t_src, y_src.astype(float))


def load_trial(ik_csv, fp_csv, mass_kg):
    """One (ik, fp) CSV pair -> a raw-trial dict on the 100 Hz grid."""
    ik = pd.read_csv(ik_csv)
    fp = pd.read_csv(fp_csv)
    weight_n = mass_kg * GRAVITY

    t_ik = ik["Header"].values.astype(float)
    t_fp = fp["Header"].values.astype(float)
    t0 = max(t_ik[0], t_fp[0])
    t1 = min(t_ik[-1], t_fp[-1])
    t_grid = np.arange(t0, t1, 1.0 / C.FPS)

    d = {}

    # --- pose angles + root positions from IK ---
    for col in C.POSE_COLS + C.ROOT_POS_COLS:
        y = ik[col].values.astype(float)
        if col in _ANGLE_COLS:
            y = np.deg2rad(y)
        d[col] = _resample(t_ik, y, t_grid)

    # --- per-foot kinetics from FP ---
    for s, pref in (("r", "Treadmill_R"), ("l", "Treadmill_L")):
        fx = _resample(t_fp, fp[f"{pref}_vx"].values, t_grid)
        fy = _resample(t_fp, fp[f"{pref}_vy"].values, t_grid)
        fz = _resample(t_fp, fp[f"{pref}_vz"].values, t_grid)
        px = _resample(t_fp, fp[f"{pref}_px"].values, t_grid)
        pz = _resample(t_fp, fp[f"{pref}_pz"].values, t_grid)
        my = _resample(t_fp, fp[f"{pref}_moment_y"].values, t_grid)
        tz_free = my + (px * fz - pz * fx)   # free vertical moment about COP

        d[f"grf_x_{s}"] = fx / weight_n      # body-weight units
        d[f"grf_y_{s}"] = fy / weight_n
        d[f"grf_z_{s}"] = fz / weight_n
        d[f"cop_x_{s}"] = px                 # metres (lab frame)
        d[f"cop_z_{s}"] = pz
        d[f"mz_{s}"] = tz_free / weight_n

    # --- clean any non-finite samples (IK gaps etc.) ---
    for k in C.RAW_COLS:
        v = d[k]
        if not np.all(np.isfinite(v)):
            v = pd.Series(v).interpolate(limit_direction="both").to_numpy()
            d[k] = np.nan_to_num(v)

    d["time"] = t_grid - t_grid[0]
    return d


def load_camargo(csv_dir, subject=DEFAULT_SUBJECT):
    """Load every ik__*.csv / fp__*.csv pair in csv_dir as raw-trial dicts."""
    mass = _subject_mass_kg(csv_dir, subject)
    ik_files = sorted(glob.glob(os.path.join(csv_dir, "ik__*.csv")))
    if not ik_files:
        raise FileNotFoundError(f"No ik__*.csv files in {csv_dir}")

    trials = []
    for ik_csv in ik_files:
        fp_csv = os.path.join(os.path.dirname(ik_csv),
                              os.path.basename(ik_csv).replace("ik__", "fp__", 1))
        if not os.path.exists(fp_csv):
            print(f"  WARN: no FP match for {os.path.basename(ik_csv)}, skipping")
            continue
        trials.append(load_trial(ik_csv, fp_csv, mass))
    print(f"Camargo: loaded {len(trials)} trials for {subject} "
          f"(mass {mass:.1f} kg)")
    return trials
