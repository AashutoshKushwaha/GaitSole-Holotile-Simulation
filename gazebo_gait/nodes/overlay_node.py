#!/usr/bin/env python3
"""
On-camera tracking + prediction overlay (demo visual).

Skeletons are built from the ACTOR'S OWN forward kinematics (the exact rendered
skeleton via DaeAnim.fk_world), so the camera/lidar "detection" lands on the real
joints. Per camera frame:
  * TRACKED (green) = actor joints + synthetic sensor noise,
  * PREDICTED (red) = the predictor's anticipated pose (+50 ms), retargeted onto
    the actor skeleton's leg bones so it overlays correctly.

Sync: the actor loops one Camargo cycle (T_CYCLE); each frame's sim timestamp ->
phase -> keyframe. Run with the actor world + camera bridge (scripts/run_overlay.sh).
"""
import os
import sys

import numpy as np
import cv2
import imageio.v2 as imageio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "tools"))
import gait_config as GC
import fk
import camera_project as CP
from dae_anim import DaeAnim
from make_gait_retarget import dominant_axis, rodrigues, BONE, detect_cycle, MESH_DIR

sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC
import data as MD
import camargo
from model import MotionPredictor
import torch

OUT = os.path.join(_ROOT, "output")
CUSTOM_DAE = os.path.join(MESH_DIR, "custom_gait.dae")
T_CYCLE = 1.48
# fk_world already returns grounded world coords (pelvis z~1.0, feet z~0.08); the
# trajectory offset must NOT be re-added (that pushed the skeleton ~1m too high).
TRAJ_Z = 0.0
CALIB_DZ = 0.0           # extra vertical nudge if needed
PHASE_OFF = -0.04        # calibrated: retards skeleton to match actor at mid-stride

# rig bone whose ORIGIN = each keypoint position
KP_BONE = {"pelvis": "Hips", "hip_r": "RightUpLeg", "hip_l": "LeftUpLeg",
           "knee_r": "RightLeg", "knee_l": "LeftLeg", "ankle_r": "RightFoot",
           "ankle_l": "LeftFoot", "toe_r": "RightToeBase", "toe_l": "LeftToeBase",
           "chest": "Spine1", "head": "Head"}


def joints_from_locals(d, override):
    """fk_world -> [K,3] keypoints in world (actor model frame + trajectory z)."""
    W = d.fk_world(override)
    pts = np.array([W[KP_BONE[n]][:3, 3] for n in fk.KEYPOINT_NAMES])
    pts[:, 2] += TRAJ_Z + CALIB_DZ
    return pts


class Overlay(Node):
    def __init__(self, trial, model, stats, n_frames=160):
        super().__init__("overlay_node")
        self.n_frames = n_frames; self.frames = []; self.t0 = None
        rng = np.random.default_rng(0)

        d = DaeAnim(CUSTOM_DAE)
        K = d.n_keys
        anim = {b: d.get_bone_frames(b) for b in d.out_src}       # bone -> [K,4,4]
        axes = {b: dominant_axis(d.rest[b][:3, :3], anim[b]) for b in BONE}

        i0, i1 = detect_cycle(trial)
        pose_all = np.stack([trial[c] for c in MC.POSE_COLS], 1)

        # perceived (noisy) angles over window + cycle, then predictor per frame
        lo = i0 - MC.T_IN - 1
        perceived = {}
        for t in range(lo, i1 + 1):
            kp = fk.keypoints({c: pose_all[t][j] for j, c in enumerate(MC.POSE_COLS)})
            kp = kp + rng.normal(0, GC.KP_POS_NOISE_M, kp.shape)
            perceived[t] = np.array([fk.solve_angles(kp)[c] for c in MC.POSE_COLS])
        pred_ang = {}
        for k in range(i0, i1 + 1):
            win = np.stack([perceived[t] for t in range(k - MC.T_IN, k + 1)])
            poses = win[1:]; pv = win[1:] - win[:-1]
            rv = np.zeros((MC.T_IN, len(MC.ROOT_VEL_COLS)), np.float32)
            inf = np.concatenate([poses, rv, pv], 1).astype(np.float32)
            wn = ((inf - stats["x_mean"]) / stats["x_std"]).astype(np.float32)
            out = model.predict_one(torch.from_numpy(wn))
            resid = MD.invert_stats(out["pose"].cpu().numpy(), stats["p_mean"], stats["p_std"])
            pred_ang[k] = poses[-1] + resid[-1]          # +50 ms horizon

        # per keyframe: actual (=rendered actor), tracked (+noise), predicted (retargeted)
        self.tracked = np.zeros((K, len(fk.KEYPOINT_NAMES), 3))
        self.predicted = np.zeros_like(self.tracked)
        for kf in range(K):
            base = {b: anim[b][kf] for b in anim}
            # the actor renders IN PLACE but walk.dae's Hips marches forward -> zero
            # the Hips horizontal translation so the skeleton stays on the actor.
            h = base["Hips"].copy(); h[0, 3] = 0.0; h[1, 3] = 0.0; base["Hips"] = h
            actual = joints_from_locals(d, base)
            self.tracked[kf] = actual + rng.normal(0, GC.KP_POS_NOISE_M, actual.shape)
            cam_f = int(round(i0 + (kf / (K - 1)) * (i1 - i0)))
            pa = pred_ang[min(cam_f, i1)]
            pbase = dict(base)
            for b, (col, sign) in BONE.items():
                M = np.eye(4)
                M[:3, :3] = d.rest[b][:3, :3] @ rodrigues(axes[b], sign * pa[MC.POSE_COLS.index(col)])
                M[:3, 3] = d.rest[b][:3, 3]
                pbase[b] = M
            self.predicted[kf] = joints_from_locals(d, pbase)
        self.K, self.i0, self.i1 = K, i0, i1
        self.create_subscription(Image, "/camera/image", self.cb, 10)
        self.get_logger().info(f"overlay_node: {K} keyframes, actor-FK skeletons ready")

    def draw(self, img, pts, color, r=4, t=2):
        uv, vis = CP.project_many(pts)
        for a, b in CP.BONES:
            if vis[a] and vis[b]:
                cv2.line(img, tuple(uv[a].astype(int)), tuple(uv[b].astype(int)), color, t)
        for i in range(len(pts)):
            if vis[i]:
                cv2.circle(img, tuple(uv[i].astype(int)), r, color, -1)

    def cb(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = ts
        phase = (((ts - self.t0) / T_CYCLE) + PHASE_OFF) % 1.0
        kf = min(int(round(phase * (self.K - 1))), self.K - 1)
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        try:
            img = arr.reshape(msg.height, msg.width, -1)[:, :, :3].copy()
        except ValueError:
            return

        # calibration mode: on one frame, dump the tracked skeleton at several
        # phase offsets so we can pick the one that lands on the actor.
        if os.environ.get("GAIT_CALIB") and len(self.frames) == 30:
            for i, off in enumerate(np.linspace(-0.12, 0.04, 7)):
                im = img.copy()
                kfc = min(int(round((((ts - self.t0) / T_CYCLE + off) % 1.0) * (self.K - 1))), self.K - 1)
                self.draw(im, self.tracked[kfc], (0, 220, 0))
                cv2.putText(im, f"PHASE_OFF={off:+.3f}", (10, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                imageio.imwrite(os.path.join(OUT, f"calib_{i}.png"), im)
            self.get_logger().info("wrote calib_0..6.png")
            raise SystemExit

        self.draw(img, self.tracked[kf], (0, 220, 0))
        self.draw(img, self.predicted[kf], (255, 40, 40))
        cv2.putText(img, "green = tracked (camera/lidar)   red = predicted +50ms",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 2)
        self.frames.append(img)
        if len(self.frames) >= self.n_frames:
            imageio.mimsave(os.path.join(OUT, "overlay.mp4"), self.frames, fps=30, macro_block_size=None)
            self.get_logger().info(f"wrote {OUT}/overlay.mp4 ({len(self.frames)} frames)")
            raise SystemExit


def main():
    trial = camargo.load_camargo(os.path.join(GC.MOTION_PREDICTOR_DIR, "data", "camargo_csv"))[0]
    torch.set_grad_enabled(False)
    st = torch.load(MC.CKPT_PATH, map_location="cpu")
    model = MotionPredictor(); model.load_state_dict(st["model"]); model.eval()
    stats = MD.load_stats(MC.STATS_PATH)
    rclpy.init()
    node = Overlay(trial, model, stats)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
