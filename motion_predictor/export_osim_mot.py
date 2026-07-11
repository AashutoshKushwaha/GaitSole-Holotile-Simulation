"""
Export ACTUAL and PREDICTED joint angles as OpenSim .mot files, so you can
view a true 3D anatomical skeleton in the OpenSim GUI.

Pairs with Camargo's own bundled model (AB14/osimxml/AB14.osim) -- its
coordinates match the IK column names exactly, so the .mot loads cleanly.

Outputs to --outdir:
  * camargo_actual.mot     -- ground-truth IK (full coordinate set)
  * camargo_predicted.mot  -- same, but the 12 model-predicted DOFs
                              (leg angles + pelvis orient/height) replaced by
                              the model's prediction at --lead frames ahead.
Non-predicted DOFs (hip_rotation, subtalar, mtp, lumbar, pelvis_tx/tz) are
copied from the actual IK so the skeleton is complete; only the predicted
DOFs differ between the two files -- that difference IS the model error.

Usage:
  python export_osim_mot.py --data <csv_dir> --lead 4 --seconds 6
"""

import argparse
import glob
import os

import numpy as np
import pandas as pd
import torch

import config as C
import data as D
from model import MotionPredictor

# POSE_COLS that are angles (rad<->deg); pelvis_ty is a length (m).
_POSE_ANGLE = set(C.LEG_ANGLE_COLS + C.PELVIS_ORIENT_COLS)


def load_model(device):
    ckpt = torch.load(C.CKPT_PATH, map_location=device)
    model = MotionPredictor().to(device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, D.load_stats(C.STATS_PATH)


def _vel(x):
    v = np.diff(x, axis=0) * C.FPS
    return np.concatenate([v[:1], v], axis=0)


def write_mot(path, time, coord_names, table):
    ncols = len(coord_names) + 1
    with open(path, "w") as fh:
        fh.write(os.path.basename(path) + "\n")
        fh.write("version=1\n")
        fh.write(f"nRows={len(time)}\n")
        fh.write(f"nColumns={ncols}\n")
        fh.write("inDegrees=yes\n")
        fh.write("endheader\n")
        fh.write("time\t" + "\t".join(coord_names) + "\n")
        for i in range(len(time)):
            row = [f"{time[i]:.5f}"] + [f"{table[c][i]:.6f}" for c in coord_names]
            fh.write("\t".join(row) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="Camargo CSV dir")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--outdir", default=C.RUNS_DIR)
    ap.add_argument("--trial", type=int, default=0, help="which ik__*.csv (index)")
    ap.add_argument("--seconds", type=float, default=6.0, help="clip length; 0 = whole trial")
    ap.add_argument("--lead", type=int, default=4,
                    help="horizon index 0..H_OUT-1; predicts (lead+1)*10 ms ahead")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    device = torch.device(args.device)
    model, stats = load_model(device)
    torch.set_grad_enabled(False)
    lead = max(0, min(args.lead, C.H_OUT - 1))
    lead_ms = int(round((lead + 1) * 1000.0 / C.FPS))

    ik_files = sorted(glob.glob(os.path.join(args.data, "ik__*.csv")))
    ik = pd.read_csv(ik_files[args.trial])
    coord_names = [c for c in ik.columns if c != "Header"]   # full OpenSim coord set
    t_src = ik["Header"].values.astype(float)
    t0, t1 = t_src[0], t_src[-1]
    if args.seconds > 0:
        t1 = min(t1, t0 + args.seconds)
    t_grid = np.arange(t0, t1, 1.0 / C.FPS)
    N = len(t_grid)

    # resample every coordinate onto the 100 Hz grid (original units: deg / m)
    actual = {c: np.interp(t_grid, t_src, ik[c].values.astype(float)) for c in coord_names}

    # model input features from the predicted DOFs (angles -> rad, pelvis_ty in m)
    pose_rad = np.stack([np.deg2rad(actual[c]) if c in _POSE_ANGLE else actual[c]
                         for c in C.POSE_COLS], axis=1)
    rootpos = np.stack([actual[c] for c in C.ROOT_POS_COLS], axis=1)
    in_feat = np.concatenate([pose_rad, _vel(rootpos), _vel(pose_rad)], axis=1)
    Xn = (in_feat - stats["x_mean"]) / stats["x_std"]
    pm, ps = stats["p_mean"], stats["p_std"]

    # predicted = actual, with the predicted DOFs overwritten where available
    predicted = {c: actual[c].copy() for c in coord_names}
    for f in range(C.T_IN + lead, N):
        t = f - lead
        if t < C.T_IN or t > N - C.H_OUT:
            continue
        w = Xn[t - C.T_IN:t].astype(np.float32)
        o = model.predict_one(torch.from_numpy(w).to(device))
        resid = D.invert_stats(o["pose"].cpu().numpy(), pm, ps)[lead]
        abs_pose = pose_rad[t - 1] + resid
        for i, c in enumerate(C.POSE_COLS):
            val = np.rad2deg(abs_pose[i]) if c in _POSE_ANGLE else abs_pose[i]
            predicted[c][f] = val

    time = t_grid - t0
    a_path = os.path.join(args.outdir, "camargo_actual.mot")
    p_path = os.path.join(args.outdir, "camargo_predicted.mot")
    write_mot(a_path, time, coord_names, actual)
    write_mot(p_path, time, coord_names, predicted)
    print(f"Wrote {N} frames ({time[-1]:.1f}s), predicted {lead_ms} ms ahead:")
    print(f"  {a_path}\n  {p_path}")


if __name__ == "__main__":
    main()
