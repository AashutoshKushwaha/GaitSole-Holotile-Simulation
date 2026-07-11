"""
Real-time streaming inference + latency benchmark.

Simulates the deployment loop: skeleton kinematics arrive one frame at a time;
we keep a rolling T_IN-frame window and, each new frame, predict the next
H_OUT frames of (pose, root velocity, foot force, foot moment) -- then measure
how long that prediction takes. This is the "minimum latency" check.

By default it streams a held-out SYNTHETIC trial so you can run it immediately
after `python train.py`. The predicted next pose is reconstructed by adding the
predicted residual to the last observed pose; force/moment come out directly in
body-weight / metres.

Usage:
  python infer_stream.py                 # CPU
  python infer_stream.py --device cuda
"""

import argparse
import time

import numpy as np
import torch

import config as C
import data as D
from model import MotionPredictor


def load_model(device):
    ckpt = torch.load(C.CKPT_PATH, map_location=device)
    model = MotionPredictor().to(device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    stats = D.load_stats(C.STATS_PATH)
    return model, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="cpu",
                    help="cpu is the realistic deployment target for latency")
    ap.add_argument("--warmup", type=int, default=20)
    args = ap.parse_args()
    device = torch.device(args.device)

    model, stats = load_model(device)
    torch.set_grad_enabled(False)

    # Held-out trial to stream (different seed than training set).
    trial = D.make_synthetic_trial(rng=np.random.default_rng(12345))
    f = D.featurize(trial)
    in_feat, pose, rootvel, kin = f["in_feat"], f["pose"], f["rootvel"], f["kin"]
    N = in_feat.shape[0]

    xm, xs = stats["x_mean"], stats["x_std"]
    pm, ps = stats["p_mean"], stats["p_std"]
    km, ks = stats["k_mean"], stats["k_std"]

    knee_i = C.POSE_COLS.index("knee_angle_r")
    gy_i = C.KINETIC_COLS.index("grf_y_r")
    copx_i = C.KINETIC_COLS.index("cop_x_r")

    latencies = []
    knee_errs, grf_errs = [], []
    printed = 0

    print(f"Streaming {N} frames at {C.FPS:.0f} Hz "
          f"(window {C.T_IN}, horizon {C.H_OUT})  device={device}\n")

    for t in range(C.T_IN, N - C.H_OUT + 1):
        window = in_feat[t - C.T_IN:t]                      # [T_IN, IN_DIM]
        wn = ((window - xm) / xs).astype(np.float32)
        xt = torch.from_numpy(wn).to(device)

        t0 = time.perf_counter()
        out = model.predict_one(xt)                          # dict of [H_OUT, .]
        if device.type == "cuda":
            torch.cuda.synchronize()
        dt_ms = (time.perf_counter() - t0) * 1e3

        # Reconstruct predicted next-frame (h=0) pose + kinetics in real units.
        pose_resid = D.invert_stats(out["pose"].cpu().numpy(), pm, ps)[0]
        pred_pose_next = pose[t - 1] + pose_resid
        pred_kin_next = D.invert_stats(out["kin"].cpu().numpy(), km, ks)[0]

        # Ground truth for the immediate next frame.
        true_pose_next = pose[t]
        true_kin_next = kin[t]

        knee_err = np.rad2deg(abs(pred_pose_next[knee_i] - true_pose_next[knee_i]))
        grf_err = abs(pred_kin_next[gy_i] - true_kin_next[gy_i])

        if t >= C.T_IN + args.warmup:           # skip warmup for latency stats
            latencies.append(dt_ms)
            knee_errs.append(knee_err)
            grf_errs.append(grf_err)

        if printed < 5 and t >= C.T_IN + args.warmup:
            print(f"frame {t:4d} | predict next: "
                  f"R-knee {np.rad2deg(pred_pose_next[knee_i]):6.1f} deg "
                  f"(true {np.rad2deg(true_pose_next[knee_i]):6.1f}) | "
                  f"R-GRFy {pred_kin_next[gy_i]:5.2f} BW "
                  f"(true {true_kin_next[gy_i]:5.2f}) | "
                  f"R-COPx {pred_kin_next[copx_i]:+.3f} m | {dt_ms:.3f} ms")
            printed += 1

    lat = np.array(latencies)
    print("\n--- LATENCY (per single-frame prediction) ---")
    print(f"   median {np.median(lat):.3f} ms | mean {lat.mean():.3f} ms | "
          f"p95 {np.percentile(lat, 95):.3f} ms | "
          f"max {lat.max():.3f} ms  ->  ~{1000/np.median(lat):.0f} fps capacity")
    print("\n--- NEXT-FRAME ACCURACY (held-out trial) ---")
    print(f"   right knee angle  MAE: {np.mean(knee_errs):.2f} deg")
    print(f"   right vertical GRF MAE: {np.mean(grf_errs):.3f} BW")


if __name__ == "__main__":
    main()
