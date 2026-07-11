"""
Bridge to the trained motion predictor (motion_predictor/).

Imports the predictor package, loads its checkpoint + normalization stats, and
streams a trial frame-by-frame the way the deployment loop would: keep the last
T_IN frames, predict the next H_OUT frames of (pose residual, root velocity, foot
force/moment). Reuses motion_predictor's own config/data/model -- no duplicated
feature logic. Mirrors motion_predictor/infer_stream.py.

The HoloTile sim consumes:
  * pose  -> the joint angles that pose the 3D skeleton (skeleton3d.py),
  * kin   -> per-foot force + COP (foot_force_mapper.py, M6),
  * rootvel / a derived anticipation cue -> the controller (M5).
"""

import os
import sys
import time

import numpy as np
import torch

import holotile_config as HC

# Make the predictor package importable (its modules import each other by bare
# name: `import config`, `import data`, `from model import ...`).
if HC.MOTION_PREDICTOR_DIR not in sys.path:
    sys.path.insert(0, HC.MOTION_PREDICTOR_DIR)

import config as MC          # noqa: E402  motion_predictor/config.py
import data as MD            # noqa: E402  motion_predictor/data.py
from model import MotionPredictor  # noqa: E402


class PredictorBridge:
    def __init__(self, device="cpu", ckpt=None, stats=None):
        self.MC, self.MD = MC, MD
        self.device = torch.device(device)
        ckpt = ckpt or MC.CKPT_PATH
        stats = stats or MC.STATS_PATH
        state = torch.load(ckpt, map_location=self.device)
        self.model = MotionPredictor().to(self.device)
        self.model.load_state_dict(state["model"])
        self.model.eval()
        self.stats = MD.load_stats(stats)
        torch.set_grad_enabled(False)
        # index helpers
        self.pose_cols = MC.POSE_COLS
        self.kin_cols = MC.KINETIC_COLS

    # --- trial streaming ----------------------------------------------------
    def load_trial(self, trial):
        """Featurize a raw-trial dict; ready to stream from frame T_IN."""
        f = MD.featurize(trial)
        self.in_feat = f["in_feat"]      # [N, IN_DIM]
        self.pose = f["pose"]            # [N, 12] absolute (rad, m)
        self.rootvel = f["rootvel"]      # [N, 2]
        self.kin = f["kin"]              # [N, 12]
        self.N = self.in_feat.shape[0]
        self.t = MC.T_IN
        return self.N

    def n_steps(self):
        return self.N - MC.H_OUT - MC.T_IN

    def step(self):
        """Advance one frame. Returns a dict of current truth + next-frame predictions.

        keys: pose_now[12], pred_pose_next[12], pred_rootvel[2], pred_kin[12],
              latency_ms. Pose units: rad (+ pelvis_ty m). Kin: BW / BW*m / m.
        """
        t = self.t
        xm, xs = self.stats["x_mean"], self.stats["x_std"]
        window = (self.in_feat[t - MC.T_IN:t] - xm) / xs
        xt = torch.from_numpy(window.astype(np.float32)).to(self.device)

        t0 = time.perf_counter()
        out = self.model.predict_one(xt)
        latency_ms = (time.perf_counter() - t0) * 1e3

        pose_resid = MD.invert_stats(out["pose"].cpu().numpy(),
                                     self.stats["p_mean"], self.stats["p_std"])[0]
        pred_pose_next = self.pose[t - 1] + pose_resid
        pred_rootvel = MD.invert_stats(out["rootvel"].cpu().numpy(),
                                       self.stats["r_mean"], self.stats["r_std"])[0]
        pred_kin = MD.invert_stats(out["kin"].cpu().numpy(),
                                   self.stats["k_mean"], self.stats["k_std"])[0]

        result = {
            "pose_now": self.pose[t].copy(),
            "pred_pose_next": pred_pose_next,
            "pred_rootvel": pred_rootvel,
            "pred_kin": pred_kin,
            "latency_ms": latency_ms,
        }
        self.t += 1
        if self.t >= self.N - MC.H_OUT:
            self.t = MC.T_IN     # loop the trial
        return result

    def pose_dict(self, pose_vec):
        """Map a 12-vector pose to {coord_name: value}."""
        return {c: float(v) for c, v in zip(self.pose_cols, pose_vec)}


if __name__ == "__main__":
    # Smoke test: stream a synthetic trial, report accuracy-free sanity + latency.
    br = PredictorBridge()
    trial = MD.make_synthetic_trial(rng=np.random.default_rng(7))
    n = br.load_trial(trial)
    print(f"loaded synthetic trial: {n} frames, {br.n_steps()} stream steps")
    lat = []
    for _ in range(200):
        r = br.step()
        lat.append(r["latency_ms"])
    knee_i = MC.POSE_COLS.index("knee_angle_r")
    gy_i = MC.KINETIC_COLS.index("grf_y_r")
    print(f"sample: R-knee_now={np.rad2deg(r['pose_now'][knee_i]):6.1f} deg  "
          f"pred R-knee_next={np.rad2deg(r['pred_pose_next'][knee_i]):6.1f} deg  "
          f"pred R-GRFy={r['pred_kin'][gy_i]:.2f} BW")
    print(f"latency: median {np.median(lat):.3f} ms  p95 {np.percentile(lat,95):.3f} ms")
