#!/usr/bin/env python3
"""
Predictor node (G5): low-latency next-pose + foot force/moment prediction.

Keeps a rolling T_IN-frame window of the solved joint angles, builds the
motion_predictor input feature [pose, rootvel, pose_vel] per frame (rootvel=0 on
the treadmill), normalizes, and runs MotionPredictor.predict_one each new frame.
Publishes the reconstructed next-frame pose + per-foot force/moment, and logs the
per-call latency (the "minimum latency" claim). Reuses motion_predictor's model +
stats -- no duplicated learning code.

In:  JOINT_ANGLES_TOPIC  Float32MultiArray [frame_idx, *pose12]
Out: PREDICTION_TOPIC    Float32MultiArray [frame_idx, *pred_pose12, *pred_kin12, latency_ms]
"""
import collections
import os
import sys
import time

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC

sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC
import data as MD
from model import MotionPredictor


class PredictorNode(Node):
    def __init__(self):
        super().__init__("predictor_node")
        torch.set_grad_enabled(False)
        state = torch.load(MC.CKPT_PATH, map_location="cpu")
        self.model = MotionPredictor(); self.model.load_state_dict(state["model"]); self.model.eval()
        self.stats = MD.load_stats(MC.STATS_PATH)
        self.win = collections.deque(maxlen=MC.T_IN + 1)   # poses incl. one extra for vel
        self.lat = collections.deque(maxlen=300)
        self.pub = self.create_publisher(Float32MultiArray, GC.PREDICTION_TOPIC, 10)
        self.sub = self.create_subscription(
            Float32MultiArray, GC.JOINT_ANGLES_TOPIC, self.cb, 50)
        self.get_logger().info(
            f"predictor_node: window {MC.T_IN}, horizon {MC.H_OUT}, "
            f"predicting next pose(12) + foot force/moment(12) on CPU")

    def cb(self, msg):
        idx = msg.data[0]
        pose = np.array(msg.data[1:1 + len(MC.POSE_COLS)], dtype=np.float32)
        self.win.append(pose)
        if len(self.win) <= MC.T_IN:
            return

        arr = np.stack(self.win)                          # [T_IN+1, 12]
        poses = arr[1:]                                   # [T_IN, 12]
        pose_vel = arr[1:] - arr[:-1]                     # [T_IN, 12]
        rootvel = np.zeros((MC.T_IN, len(MC.ROOT_VEL_COLS)), np.float32)
        in_feat = np.concatenate([poses, rootvel, pose_vel], axis=1)   # [T_IN, 26]
        wn = ((in_feat - self.stats["x_mean"]) / self.stats["x_std"]).astype(np.float32)

        t0 = time.perf_counter()
        out = self.model.predict_one(torch.from_numpy(wn))
        dt_ms = (time.perf_counter() - t0) * 1e3
        self.lat.append(dt_ms)

        pose_resid = MD.invert_stats(out["pose"].cpu().numpy(),
                                     self.stats["p_mean"], self.stats["p_std"])[0]
        pred_pose = poses[-1] + pose_resid                # next-frame pose
        pred_kin = MD.invert_stats(out["kin"].cpu().numpy(),
                                   self.stats["k_mean"], self.stats["k_std"])[0]

        m = Float32MultiArray()
        m.data = ([float(idx)] + [float(v) for v in pred_pose]
                  + [float(v) for v in pred_kin] + [float(dt_ms)])
        self.pub.publish(m)
        if len(self.lat) % 100 == 0:
            self.get_logger().info(
                f"latency median {np.median(self.lat):.3f} ms  p95 "
                f"{np.percentile(self.lat,95):.3f} ms ({len(self.lat)} preds)")


def main():
    rclpy.init()
    try:
        rclpy.spin(PredictorNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
