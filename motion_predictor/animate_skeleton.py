"""
Animated skeletal comparison: ACTUAL vs PREDICTED gait, for presentations.

Renders a sagittal (side-view) stick figure by forward-kinematics from the
predicted/actual joint angles, side by side:
  * LEFT  panel  -- actual (ground-truth) skeleton, black.
  * RIGHT panel  -- predicted skeleton, with a faint GRAY GHOST of the actual
                    behind it and each segment tinted green->red by its
                    instantaneous joint-angle error (so you see WHERE it's off).
Vertical ground-reaction-force arrows are drawn at each foot (actual = blue,
predicted = orange), and a live error readout sits in the title.

Both skeletons go through identical FK, so the visible gap faithfully reflects
the model's error. This is a sagittal view (hip/knee/ankle flexion); frontal
plane (hip ab/adduction) is not shown.

Usage:
  python animate_skeleton.py --data <csv_dir> --device cuda --seconds 5
Output: runs/skeleton_compare.mp4
"""

import argparse
import os

import numpy as np
import torch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import cm

import config as C
import data as D
from model import MotionPredictor

ERR_MAX_DEG = 6.0          # joint error mapped to full red at this value
GRF_SCALE = 0.22           # metres of arrow per body-weight of vertical GRF
_CMAP = matplotlib.colormaps["RdYlGn_r"]


def load_model(device):
    ckpt = torch.load(C.CKPT_PATH, map_location=device)
    model = MotionPredictor().to(device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, D.load_stats(C.STATS_PATH)


def segment_lengths(height_m):
    return dict(thigh=0.245 * height_m, shank=0.246 * height_m,
                foot=0.152 * height_m, trunk=0.30 * height_m)


def fk(row, L):
    """Forward kinematics -> dict of 2D points for trunk + both legs.
    row: dict of POSE_COLS in physical units (angles rad, pelvis_ty m)."""
    P = np.array([0.0, row["pelvis_ty"]])
    tilt = row["pelvis_tilt"]
    T = P + L["trunk"] * np.array([np.sin(tilt), np.cos(tilt)])

    out = {"trunk": (P, T)}
    for s in ("r", "l"):
        hip = row[f"hip_flexion_{s}"]
        knee = row[f"knee_angle_{s}"]          # Camargo: negative = flexion
        ank = row[f"ankle_angle_{s}"]
        th = hip                                # thigh angle from vertical (fwd +)
        K = P + L["thigh"] * np.array([np.sin(th), -np.cos(th)])
        sh = th + knee                          # shank angle from vertical
        A = K + L["shank"] * np.array([np.sin(sh), -np.cos(sh)])
        fa = sh + ank                           # foot direction (from +x axis)
        Toe = A + L["foot"] * np.array([np.cos(fa), np.sin(fa)])
        out[f"thigh_{s}"] = (P, K)
        out[f"shank_{s}"] = (K, A)
        out[f"foot_{s}"] = (A, Toe)
        out[f"ankle_{s}"] = A
    return out


def collect_frames(model, stats, trial, device, seconds, lead=0):
    xm, xs = stats["x_mean"], stats["x_std"]
    pm, ps = stats["p_mean"], stats["p_std"]
    km, ks = stats["k_mean"], stats["k_std"]
    f = D.featurize(trial)
    inf, pose, kin = f["in_feat"], f["pose"], f["kin"]
    N = inf.shape[0]
    end = min(C.T_IN + int(seconds * C.FPS), N - C.H_OUT + 1)
    gy_r, gy_l = C.KINETIC_COLS.index("grf_y_r"), C.KINETIC_COLS.index("grf_y_l")

    frames = []
    for t in range(C.T_IN, end):
        w = ((inf[t - C.T_IN:t] - xm) / xs).astype(np.float32)
        o = model.predict_one(torch.from_numpy(w).to(device))
        pr = D.invert_stats(o["pose"].cpu().numpy(), pm, ps)[lead]
        pk = D.invert_stats(o["kin"].cpu().numpy(), km, ks)[lead]
        # prediction made from the window ending at t-1 targets frame t+lead
        actual_pose = {c: pose[t + lead][i] for i, c in enumerate(C.POSE_COLS)}
        pred_pose = {c: (pose[t - 1][i] + pr[i]) for i, c in enumerate(C.POSE_COLS)}
        frames.append(dict(
            t=(t + lead) / C.FPS,
            actual_pose=actual_pose, pred_pose=pred_pose,
            grf_actual=(kin[t + lead][gy_r], kin[t + lead][gy_l]),
            grf_pred=(pk[gy_r], pk[gy_l]),
        ))
    return frames


def seg_error_deg(actual_pose, pred_pose, joint_col):
    return abs(np.rad2deg(actual_pose[joint_col] - pred_pose[joint_col]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--subject", default=None)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--outdir", default=C.RUNS_DIR)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--fps", type=int, default=50, help="playback fps (50 = 2x slow-mo)")
    ap.add_argument("--trial", type=int, default=0)
    ap.add_argument("--lead", type=int, default=0,
                    help="horizon index 0..H_OUT-1; predicts (lead+1)*10 ms ahead")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    device = torch.device(args.device)
    model, stats = load_model(device)
    torch.set_grad_enabled(False)

    import camargo
    trials = (camargo.load_camargo(args.data) if args.subject is None
              else camargo.load_camargo(args.data, args.subject))
    # subject height for limb lengths
    import pandas as pd
    si = pd.read_csv(os.path.join(args.data, "camargo_convert__SubjectInfo.csv"))
    subj = args.subject or camargo.DEFAULT_SUBJECT
    height = float(si[si["Subject"] == subj]["Height"].iloc[0])
    L = segment_lengths(height)

    lead = max(0, min(args.lead, C.H_OUT - 1))
    lead_ms = int(round((lead + 1) * 1000.0 / C.FPS))
    frames = collect_frames(model, stats, trials[args.trial], device, args.seconds, lead)
    print(f"Rendering {len(frames)} frames (subject {subj}, height {height} m), "
          f"predicting {lead_ms} ms ahead...")

    # ---- figure / artists ----
    fig, (axA, axP) = plt.subplots(1, 2, figsize=(11, 6.5))
    for ax, title in ((axA, "ACTUAL"), (axP, "PREDICTED")):
        ax.set_xlim(-0.55, 0.7); ax.set_ylim(-0.05, 1.15)
        ax.set_aspect("equal"); ax.set_title(title, fontsize=14, fontweight="bold")
        ax.axhline(0.0, color="0.7", lw=1)
        ax.set_xticks([]); ax.set_yticks([])
    axP.set_title(f"PREDICTED  (+{lead_ms} ms)", fontsize=14, fontweight="bold")

    LEGS = ["thigh_r", "shank_r", "foot_r", "thigh_l", "shank_l", "foot_l"]
    JOINT_OF = {"thigh_r": "hip_flexion_r", "shank_r": "knee_angle_r", "foot_r": "ankle_angle_r",
                "thigh_l": "hip_flexion_l", "shank_l": "knee_angle_l", "foot_l": "ankle_angle_l"}

    # actual panel artists (black)
    a_lines = {k: axA.plot([], [], "-", color="black", lw=3, solid_capstyle="round")[0]
               for k in ["trunk"] + LEGS}
    a_grf = [axA.plot([], [], "-", color="tab:blue", lw=2.5)[0] for _ in range(2)]
    # predicted panel: ghost (gray, actual) + colored predicted
    g_lines = {k: axP.plot([], [], "-", color="0.75", lw=2.5, alpha=0.7)[0]
               for k in ["trunk"] + LEGS}
    p_lines = {k: axP.plot([], [], "-", lw=3, solid_capstyle="round")[0]
               for k in ["trunk"] + LEGS}
    p_grf = [axP.plot([], [], "-", color="tab:orange", lw=2.5)[0] for _ in range(2)]

    suptitle = fig.suptitle("", fontsize=13)
    # color legend for error
    sm = cm.ScalarMappable(cmap=_CMAP, norm=plt.Normalize(0, ERR_MAX_DEG))
    cbar = fig.colorbar(sm, ax=axP, fraction=0.046, pad=0.04)
    cbar.set_label("joint angle error (deg)")

    def set_seg(line, seg):
        (x0, y0), (x1, y1) = seg
        line.set_data([x0, x1], [y0, y1])

    def draw_grf(lines, pts, grf):
        for ln, ank, g in zip(lines, pts, grf):
            x, y = ank
            ln.set_data([x, x], [y, y + max(g, 0) * GRF_SCALE])

    def update(i):
        fr = frames[i]
        kA = fk(fr["actual_pose"], L)
        kP = fk(fr["pred_pose"], L)
        # actual panel
        for k in ["trunk"] + LEGS:
            set_seg(a_lines[k], kA[k])
        draw_grf(a_grf, [kA["ankle_r"], kA["ankle_l"]], fr["grf_actual"])
        # predicted panel: ghost + colored predicted
        for k in ["trunk"] + LEGS:
            set_seg(g_lines[k], kA[k])
            set_seg(p_lines[k], kP[k])
        p_lines["trunk"].set_color("black")
        for k in LEGS:
            e = seg_error_deg(fr["actual_pose"], fr["pred_pose"], JOINT_OF[k])
            p_lines[k].set_color(_CMAP(min(e / ERR_MAX_DEG, 1.0)))
        draw_grf(p_grf, [kP["ankle_r"], kP["ankle_l"]], fr["grf_pred"])

        knee_e = seg_error_deg(fr["actual_pose"], fr["pred_pose"], "knee_angle_r")
        grf_e = abs(fr["grf_actual"][0] - fr["grf_pred"][0])
        suptitle.set_text(f"+{lead_ms} ms ahead   |   t = {fr['t']:.2f} s   |   "
                          f"R-knee err {knee_e:.2f} deg   |   R-GRF err {grf_e:.3f} BW")
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(frames), interval=1000 / args.fps)
    out = os.path.join(args.outdir, f"skeleton_compare_{lead_ms}ms.mp4")
    writer = animation.FFMpegWriter(fps=args.fps, bitrate=2400)
    anim.save(out, writer=writer)
    print(f"Saved -> {out}")


if __name__ == "__main__":
    main()
