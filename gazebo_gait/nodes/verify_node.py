#!/usr/bin/env python3
"""
Verification (G6): predicted next-step vs the human's ACTUAL step.

Subscribes /prediction (made at frame f, predicting frame f+1) and compares each
against the Camargo ground truth at f+1: next-pose error, foot force/moment error,
and heel-strike timing (the "predicted step vs actual step" check). After
collecting N predictions it writes plots + a metrics CSV and exits.

Camargo is loaded BEFORE any ROS entity is created (pandas-after-node-creation
corrupts the rcl context -- see gait_player).

Run via scripts/run_verify.sh (needs gait_player+perception+angle_solver+predictor).
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC
import fk
from scipy.signal import find_peaks
sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC

ANKLE_R = fk.KEYPOINT_NAMES.index("ankle_r")


def foot_strikes(poses):
    """Heel-strike frames = peaks of the foot's forward (x) position (terminal
    swing), from FK of the pose -- robust, uses the strong pose signal."""
    fwd = np.array([fk.keypoints({c: p[i] for i, c in enumerate(MC.POSE_COLS)})[ANKLE_R, 0]
                    for p in poses])
    pk, _ = find_peaks(fwd, distance=int(0.5 * GC.FPS))
    return pk

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")
NP = len(MC.POSE_COLS)
NK = len(MC.KINETIC_COLS)


def heel_strikes(grf, thr=0.05):
    c = grf > thr
    return np.where((~c[:-1]) & c[1:])[0] + 1


class Verify(Node):
    def __init__(self, trial, n_collect=700):
        super().__init__("verify_node")
        self.tr = trial
        self.gt_pose = np.stack([trial[c] for c in MC.POSE_COLS], 1)   # [N,12]
        self.gt_kin = np.stack([trial[c] for c in MC.KINETIC_COLS], 1)  # [N,12]
        self.N = self.gt_pose.shape[0]
        self.n_collect = n_collect
        self.rows = []                # (idx, pred_pose12, pred_kin12, lat)
        self.create_subscription(Float32MultiArray, GC.PREDICTION_TOPIC, self.cb, 50)
        self.get_logger().info(f"verify_node: collecting {n_collect} predictions ...")

    def cb(self, msg):
        idx = int(msg.data[0])
        pred_pose = np.array(msg.data[1:1 + NP])
        pred_kin = np.array(msg.data[1 + NP:1 + NP + NK])
        lat = msg.data[-1]
        self.rows.append((idx, pred_pose, pred_kin, lat))
        if len(self.rows) >= self.n_collect:
            self.finish()
            raise SystemExit

    def finish(self):
        rows = sorted(self.rows, key=lambda r: r[0])
        idx = np.array([r[0] for r in rows])
        tgt = (idx + 1) % self.N                       # prediction is for frame f+1
        pred_pose = np.stack([r[1] for r in rows])
        pred_kin = np.stack([r[2] for r in rows])
        lat = np.array([r[3] for r in rows])
        act_pose = self.gt_pose[tgt]
        act_kin = self.gt_kin[tgt]

        ki = MC.POSE_COLS.index("knee_angle_r")
        hi = MC.POSE_COLS.index("hip_flexion_r")
        gy = MC.KINETIC_COLS.index("grf_y_r")

        knee_mae = np.rad2deg(np.mean(np.abs(pred_pose[:, ki] - act_pose[:, ki])))
        hip_mae = np.rad2deg(np.mean(np.abs(pred_pose[:, hi] - act_pose[:, hi])))
        pose_mae = np.rad2deg(np.mean(np.abs(pred_pose[:, :8] - act_pose[:, :8])))  # leg joints
        grf_mae = np.mean(np.abs(pred_kin[:, gy] - act_kin[:, gy]))
        grf_corr = np.corrcoef(pred_kin[:, gy], act_kin[:, gy])[0, 1]

        # step timing: heel strikes from foot kinematics (forward-position peaks),
        # predicted vs actual -- the "predicted step vs actual step" check
        hs_a = foot_strikes(act_pose); hs_p = foot_strikes(pred_pose)
        dt_ms = []
        for h in hs_a:
            if len(hs_p):
                dt_ms.append(abs(hs_p[np.argmin(np.abs(hs_p - h))] - h) / GC.FPS * 1000)
        hs_err = float(np.mean(dt_ms)) if dt_ms else float("nan")

        metrics = {
            "predictions": len(rows),
            "leg_pose_MAE_deg": round(pose_mae, 2),
            "knee_R_MAE_deg": round(knee_mae, 2),
            "hip_R_MAE_deg": round(hip_mae, 2),
            "vGRF_R_MAE_BW": round(grf_mae, 3),
            "vGRF_R_corr": round(float(grf_corr), 3),
            "heelstrike_timing_err_ms": round(hs_err, 1),
            "latency_median_ms": round(float(np.median(lat)), 3),
            "latency_p95_ms": round(float(np.percentile(lat, 95)), 3),
        }
        with open(os.path.join(OUT, "verify_metrics.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(metrics.keys()); w.writerow(metrics.values())

        # plot: actual vs predicted R-knee + R vertical GRF, with actual heel strikes
        t = np.arange(len(rows)) / GC.FPS
        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        ax[0].plot(t, np.rad2deg(act_pose[:, ki]), label="actual", lw=2)
        ax[0].plot(t, np.rad2deg(pred_pose[:, ki]), "--", label="predicted")
        ax[0].set_ylabel("R knee (deg)"); ax[0].legend(); ax[0].grid(alpha=.3)
        ax[0].set_title(f"Predicted vs actual  |  leg-pose MAE {pose_mae:.1f} deg  |  "
                        f"vGRF corr {grf_corr:.2f}  |  latency {np.median(lat):.1f} ms")
        ax[1].plot(t, act_kin[:, gy], label="actual", lw=2)
        ax[1].plot(t, pred_kin[:, gy], "--", label="predicted")
        for h in hs_a:
            ax[1].axvline(h / GC.FPS, color="k", alpha=.15)
        ax[1].set_ylabel("R vertical GRF (BW)"); ax[1].set_xlabel("time (s)")
        ax[1].legend(); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, "verify_curves.png"), dpi=110)

        print("=== VERIFICATION METRICS ===")
        for k, v in metrics.items():
            print(f"  {k:26s} {v}")
        print(f"wrote {OUT}/verify_curves.png + verify_metrics.csv")


def main():
    sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
    import camargo
    trial = camargo.load_camargo(os.path.join(GC.MOTION_PREDICTOR_DIR, "data", "camargo_csv"))[0]
    rclpy.init()
    node = Verify(trial)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
