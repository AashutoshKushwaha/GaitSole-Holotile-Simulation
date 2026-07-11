"""
Evaluate a trained motion predictor and produce viewable diagnostics.

Outputs (saved to --outdir, default runs/):
  1. eval_metrics.csv  -- per-channel next-frame MAE in physical units,
                          computed on the held-out VALIDATION windows
                          (same split as train.py, seed C.SEED) = honest number.
  2. eval_curves.png   -- predicted vs actual overlay for key joints + GRF over
                          a continuous stretch (the gait curves to eyeball).
  3. eval_horizon.png  -- error vs prediction lead-time (0..H_OUT-1 frames).

Pose is reconstructed by adding the predicted residual to the last observed
pose (matches infer_stream.py); kinetics come out directly in BW / metres.

Usage:
  python evaluate.py --data <csv_dir> --device cuda
"""

import argparse
import os

import numpy as np
import torch

import matplotlib
matplotlib.use("Agg")            # headless server -> render to file
import matplotlib.pyplot as plt

import config as C
import data as D
from model import MotionPredictor

# POSE channels that are angles (report in degrees); pelvis_ty is a length (m).
_POSE_IS_ANGLE = np.array([c not in C.PELVIS_HEIGHT_COLS for c in C.POSE_COLS])
# KINETIC channels that are forces/moments (BW / BW*m) vs COP lengths (m).
_KIN_UNIT = ["BW" if (c.startswith("grf") or c.startswith("mz")) else "m"
             for c in C.KINETIC_COLS]


def load_model(device):
    ckpt = torch.load(C.CKPT_PATH, map_location=device)
    model = MotionPredictor().to(device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, D.load_stats(C.STATS_PATH)


def _val_indices(n):
    """Reproduce train.py's random_split val indices exactly."""
    n_val = int(n * C.VAL_FRACTION)
    n_tr = n - n_val
    g = torch.Generator().manual_seed(C.SEED)
    perm = torch.randperm(n, generator=g).numpy()
    return perm[n_tr:]


def _batched_forward(model, Xn, device, bs=8192):
    outs_p, outs_k = [], []
    for i in range(0, len(Xn), bs):
        xt = torch.from_numpy(Xn[i:i + bs].astype(np.float32)).to(device)
        o = model(xt)
        outs_p.append(o["pose"].cpu().numpy())
        outs_k.append(o["kin"].cpu().numpy())
    return np.concatenate(outs_p), np.concatenate(outs_k)


def per_channel_metrics(model, stats, trials, device, outdir):
    X, Yp, Yr, Yk = D.build_windows(trials)
    vi = _val_indices(len(X))
    Xn = (X - stats["x_mean"]) / stats["x_std"]
    pred_p, pred_k = _batched_forward(model, Xn[vi], device)
    pred_p = D.invert_stats(pred_p, stats["p_mean"], stats["p_std"])  # residual, phys
    pred_k = D.invert_stats(pred_k, stats["k_mean"], stats["k_std"])  # absolute, phys
    true_p, true_k = Yp[vi], Yk[vi]                                   # phys

    # --- next-frame (h=0) per-channel MAE ---
    pe = np.abs(pred_p[:, 0, :] - true_p[:, 0, :]).mean(0)            # [12]
    ke = np.abs(pred_k[:, 0, :] - true_k[:, 0, :]).mean(0)            # [12]
    pe_disp = np.where(_POSE_IS_ANGLE, np.rad2deg(pe), pe)

    lines = ["group,channel,mae,unit"]
    print(f"\n=== Per-channel next-frame MAE  (held-out val, {len(vi)} windows) ===")
    print("  POSE")
    for c, e, ang in zip(C.POSE_COLS, pe_disp, _POSE_IS_ANGLE):
        u = "deg" if ang else "m"
        print(f"    {c:18s} {e:8.3f} {u}")
        lines.append(f"pose,{c},{e:.5f},{u}")
    print("  KINETICS")
    for c, e, u in zip(C.KINETIC_COLS, ke, _KIN_UNIT):
        print(f"    {c:18s} {e:8.4f} {u}")
        lines.append(f"kin,{c},{e:.6f},{u}")

    # headline summary
    knee = pe_disp[C.POSE_COLS.index("knee_angle_r")]
    leg_ang = pe_disp[_POSE_IS_ANGLE].mean()
    grf_idx = [i for i, c in enumerate(C.KINETIC_COLS) if c.startswith("grf")]
    grf_mae = ke[grf_idx].mean()
    print(f"\n  summary: mean leg/pelvis angle MAE {leg_ang:.2f} deg | "
          f"R-knee {knee:.2f} deg | mean GRF MAE {grf_mae:.3f} BW")

    with open(os.path.join(outdir, "eval_metrics.csv"), "w") as fh:
        fh.write("\n".join(lines) + "\n")

    # --- horizon: MAE vs lead time, averaged over channels groups ---
    H = pred_p.shape[1]
    knee_i = C.POSE_COLS.index("knee_angle_r")
    gy_i = C.KINETIC_COLS.index("grf_y_r")
    knee_h = np.rad2deg(np.abs(pred_p[:, :, knee_i] - true_p[:, :, knee_i]).mean(0))
    grf_h = np.abs(pred_k[:, :, gy_i] - true_k[:, :, gy_i]).mean(0)
    lead_ms = (np.arange(H) + 1) * (1000.0 / C.FPS)

    fig, ax1 = plt.subplots(figsize=(7, 4))
    ax1.plot(lead_ms, knee_h, "o-", color="tab:blue", label="R-knee angle")
    ax1.set_xlabel("prediction lead time (ms)")
    ax1.set_ylabel("knee angle MAE (deg)", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax1.twinx()
    ax2.plot(lead_ms, grf_h, "s-", color="tab:red", label="R-vertical GRF")
    ax2.set_ylabel("vertical GRF MAE (BW)", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")
    ax1.set_title("Error vs prediction lead time (held-out val)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "eval_horizon.png"), dpi=120)
    plt.close(fig)


def overlay_plots(model, stats, trials, device, outdir, seconds):
    xm, xs = stats["x_mean"], stats["x_std"]
    pm, ps = stats["p_mean"], stats["p_std"]
    km, ks = stats["k_mean"], stats["k_std"]

    f = D.featurize(trials[0])
    inf, pose, kin = f["in_feat"], f["pose"], f["kin"]
    N = inf.shape[0]
    end = min(C.T_IN + int(seconds * C.FPS), N - C.H_OUT + 1)

    tax, pp, tp, pk, tk = [], [], [], [], []
    for t in range(C.T_IN, end):
        w = ((inf[t - C.T_IN:t] - xm) / xs).astype(np.float32)
        o = model.predict_one(torch.from_numpy(w).to(device))
        pr = D.invert_stats(o["pose"].cpu().numpy(), pm, ps)[0]
        pkin = D.invert_stats(o["kin"].cpu().numpy(), km, ks)[0]
        pp.append(pose[t - 1] + pr); tp.append(pose[t])
        pk.append(pkin); tk.append(kin[t])
        tax.append(t / C.FPS)
    tax = np.array(tax); pp = np.array(pp); tp = np.array(tp)
    pk = np.array(pk); tk = np.array(tk)

    def pidx(c): return C.POSE_COLS.index(c)
    def kidx(c): return C.KINETIC_COLS.index(c)

    panels = [
        ("knee_angle_r", pp, tp, pidx, "deg", True, "R knee angle"),
        ("hip_flexion_r", pp, tp, pidx, "deg", True, "R hip flexion"),
        ("ankle_angle_r", pp, tp, pidx, "deg", True, "R ankle angle"),
        ("pelvis_ty", pp, tp, pidx, "m", False, "pelvis height"),
        ("grf_y_r", pk, tk, kidx, "BW", False, "R vertical GRF"),
        ("grf_y_l", pk, tk, kidx, "BW", False, "L vertical GRF"),
        ("grf_x_r", pk, tk, kidx, "BW", False, "R AP GRF"),
        ("cop_x_r", pk, tk, kidx, "m", False, "R COP (AP)"),
    ]
    fig, axes = plt.subplots(4, 2, figsize=(13, 11), sharex=True)
    for ax, (ch, P, T, idxf, unit, is_ang, title) in zip(axes.ravel(), panels):
        i = idxf(ch)
        yp = np.rad2deg(P[:, i]) if is_ang else P[:, i]
        yt = np.rad2deg(T[:, i]) if is_ang else T[:, i]
        ax.plot(tax, yt, color="k", lw=1.6, label="actual")
        ax.plot(tax, yp, color="tab:orange", lw=1.1, ls="--", label="predicted")
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc="upper right", fontsize=9)
    for ax in axes[-1, :]:
        ax.set_xlabel("time (s)")
    fig.suptitle("Next-frame prediction vs actual (real Camargo treadmill gait)",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(os.path.join(outdir, "eval_curves.png"), dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="Camargo CSV dir")
    ap.add_argument("--subject", default=None)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--outdir", default=C.RUNS_DIR)
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    device = torch.device(args.device)
    model, stats = load_model(device)
    torch.set_grad_enabled(False)

    import camargo
    trials = (camargo.load_camargo(args.data) if args.subject is None
              else camargo.load_camargo(args.data, args.subject))

    per_channel_metrics(model, stats, trials, device, args.outdir)
    overlay_plots(model, stats, trials, device, args.outdir, args.seconds)
    print(f"\nSaved: eval_metrics.csv, eval_curves.png, eval_horizon.png -> {args.outdir}")


if __name__ == "__main__":
    main()
