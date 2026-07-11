"""
Train the legs-only motion predictor.

By default trains on SYNTHETIC gait so the full pipeline runs end-to-end with
no download (great for a CPU smoke test). Point --data at a real preprocessed
dataset (same intermediate format as data.make_synthetic_trial) to train for
real -- ideally on a GPU (Lightning.ai), which this is well-suited to.

Usage:
  python train.py                         # synthetic, CPU/GPU auto
  python train.py --epochs 60 --device cuda
  python train.py --data <dir_of_trials>  # (hook your real loader in load_real)

Saves runs/predictor.pt and runs/norm_stats.npz.
"""

import argparse
import os
import time

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

import config as C
import data as D
from model import MotionPredictor, count_params


def make_loaders(trials, batch, val_fraction, seed):
    X, Yp, Yr, Yk = D.build_windows(trials)
    stats = D.fit_stats(X, Yp, Yr, Yk)

    Xn = D.apply_stats(X, stats["x_mean"], stats["x_std"])
    Ypn = D.apply_stats(Yp, stats["p_mean"], stats["p_std"])
    Yrn = D.apply_stats(Yr, stats["r_mean"], stats["r_std"])
    Ykn = D.apply_stats(Yk, stats["k_mean"], stats["k_std"])

    tensors = [torch.from_numpy(a) for a in (Xn, Ypn, Yrn, Ykn)]
    ds = TensorDataset(*tensors)

    n_val = int(len(ds) * val_fraction)
    n_tr = len(ds) - n_val
    g = torch.Generator().manual_seed(seed)
    tr, va = torch.utils.data.random_split(ds, [n_tr, n_val], generator=g)
    return (DataLoader(tr, batch_size=batch, shuffle=True),
            DataLoader(va, batch_size=batch), stats)


def step_loss(model, batch, device):
    x, yp, yr, yk = [b.to(device) for b in batch]
    out = model(x)
    mse = torch.nn.functional.mse_loss
    lp = mse(out["pose"], yp)
    lr = mse(out["rootvel"], yr)
    lk = mse(out["kin"], yk)
    total = C.W_POSE * lp + C.W_ROOTVEL * lr + C.W_KIN * lk
    return total, (lp.item(), lr.item(), lk.item())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=C.EPOCHS)
    ap.add_argument("--batch", type=int, default=C.BATCH)
    ap.add_argument("--lr", type=float, default=C.LR)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--n_trials", type=int, default=40, help="synthetic trials")
    ap.add_argument("--data", default=None, help="dir of real trials (optional)")
    args = ap.parse_args()

    torch.manual_seed(C.SEED)
    np.random.seed(C.SEED)
    os.makedirs(C.RUNS_DIR, exist_ok=True)

    if args.data:
        trials = load_real(args.data)
        print(f"Loaded {len(trials)} real trials from {args.data}")
    else:
        trials = D.make_synthetic_dataset(n_trials=args.n_trials)
        print(f"Generated {len(trials)} synthetic gait trials")

    tr_loader, va_loader, stats = make_loaders(
        trials, args.batch, C.VAL_FRACTION, C.SEED)
    D.save_stats(C.STATS_PATH, stats)

    device = torch.device(args.device)
    model = MotionPredictor().to(device)
    print(f"Model params: {count_params(model):,}  | device: {device}")
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr,
                            weight_decay=C.WEIGHT_DECAY)

    best_val = float("inf")
    for ep in range(1, args.epochs + 1):
        model.train()
        t0 = time.perf_counter()
        for batch in tr_loader:
            opt.zero_grad()
            loss, _ = step_loss(model, batch, device)
            loss.backward()
            opt.step()

        model.eval()
        vp = vr = vk = vt = 0.0
        nb = 0
        with torch.no_grad():
            for batch in va_loader:
                loss, (lp, lr, lk) = step_loss(model, batch, device)
                vt += loss.item(); vp += lp; vr += lr; vk += lk; nb += 1
        vt, vp, vr, vk = (v / max(nb, 1) for v in (vt, vp, vr, vk))
        dt = time.perf_counter() - t0
        print(f"ep {ep:3d} | val {vt:.4f} (pose {vp:.4f} rootvel {vr:.4f} "
              f"kin {vk:.4f}) | {dt:.1f}s")

        if vt < best_val:
            best_val = vt
            torch.save({"model": model.state_dict(),
                        "config": {k: getattr(C, k) for k in
                                   ("IN_DIM", "T_IN", "H_OUT", "HIDDEN",
                                    "N_LAYERS", "OUT_POSE_DIM",
                                    "OUT_ROOTVEL_DIM", "OUT_KIN_DIM")}},
                       C.CKPT_PATH)

    print(f"\nBest val loss {best_val:.4f}")
    print(f"Saved checkpoint -> {C.CKPT_PATH}")
    print(f"Saved norm stats -> {C.STATS_PATH}")
    _report_physical_error(model, va_loader, stats, device)


def _report_physical_error(model, loader, stats, device):
    """Translate normalized val error into interpretable units for one batch:
    joint angle error in degrees, vertical GRF error in body-weight."""
    model.eval()
    batch = next(iter(loader))
    x, yp, yr, yk = [b.to(device) for b in batch]
    with torch.no_grad():
        out = model(x)
    # Pose residual error -> degrees (denormalize, compare on first horizon step)
    p_pred = D.invert_stats(out["pose"].cpu().numpy(), stats["p_mean"], stats["p_std"])
    p_true = D.invert_stats(yp.cpu().numpy(), stats["p_mean"], stats["p_std"])
    knee_idx = C.POSE_COLS.index("knee_angle_r")
    knee_err_deg = np.rad2deg(np.abs(p_pred[:, 0, knee_idx] - p_true[:, 0, knee_idx])).mean()
    # Vertical GRF error -> BW
    k_pred = D.invert_stats(out["kin"].cpu().numpy(), stats["k_mean"], stats["k_std"])
    k_true = D.invert_stats(yk.cpu().numpy(), stats["k_mean"], stats["k_std"])
    gy_idx = C.KINETIC_COLS.index("grf_y_r")
    grf_err_bw = np.abs(k_pred[:, 0, gy_idx] - k_true[:, 0, gy_idx]).mean()
    print("Interpretable next-frame error (val batch):")
    print(f"   right knee angle : {knee_err_deg:.2f} deg")
    print(f"   right vertical GRF: {grf_err_bw:.3f} BW")


def load_real(path):
    """Load real preprocessed trials as raw-trial dicts (config.RAW_COLS + 'time').

    Dispatches to the Camargo et al. 2021 adapter, which reads the CSVs exported
    from the dataset's MATLAB tables (see camargo.py / camargo_table_to_csv.m).
    `path` is the directory containing ik__*.csv / fp__*.csv / SubjectInfo CSV."""
    import camargo
    return camargo.load_camargo(path)


if __name__ == "__main__":
    main()
