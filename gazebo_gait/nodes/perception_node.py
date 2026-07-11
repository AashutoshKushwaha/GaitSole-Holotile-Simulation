#!/usr/bin/env python3
"""
Perception node (Option A): turn the ground-truth pose into "sensed" 3D keypoints.

Subscribes the gait ground truth (joint angles), runs forward kinematics to 3D
joint positions, then applies a synthetic camera/lidar sensor model -- per-axis
Gaussian noise, a processing LATENCY (detections arrive a few frames late), and
optional dropout -- and publishes the noisy keypoints. This is the swappable box:
later (G7) a real pose-estimation model on the camera image publishes the SAME
topic. Reuses fk.keypoints so the geometry matches the rendered figure.

In:  GROUND_TRUTH_TOPIC  Float32MultiArray [frame_idx, *pose12]
Out: PERCEIVED_KP_TOPIC  Float32MultiArray [frame_idx, *(K*3 xyz)]

Run: source /opt/ros/jazzy/setup.bash && ~/venvs/gait/bin/python nodes/perception_node.py
"""
import collections
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC
import fk

sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.pub = self.create_publisher(Float32MultiArray, GC.PERCEIVED_KP_TOPIC, 10)
        self.sub = self.create_subscription(
            Float32MultiArray, GC.GROUND_TRUTH_TOPIC, self.on_gt, 50)
        self.rng = np.random.default_rng(0)
        self.lag = max(0, round(GC.KP_LATENCY_S * GC.FPS))   # frames of latency
        self.buf = collections.deque(maxlen=self.lag + 1)
        self.get_logger().info(
            f"perception_node: {len(fk.KEYPOINT_NAMES)} keypoints, "
            f"noise {GC.KP_POS_NOISE_M*1000:.0f}mm, latency {self.lag} frames "
            f"({GC.KP_LATENCY_S*1000:.0f}ms), dropout {GC.KP_DROPOUT_PROB:.2f}")

    def on_gt(self, msg):
        frame_idx = msg.data[0]
        pose = {c: float(v) for c, v in zip(MC.POSE_COLS, msg.data[1:1 + len(MC.POSE_COLS)])}
        kp = fk.keypoints(pose)                                   # [K,3] clean

        # sensor model: Gaussian position noise (camera+lidar jitter)
        kp = kp + self.rng.normal(0, GC.KP_POS_NOISE_M, kp.shape)
        # optional dropout: a missed detection holds the last good value (NaN-free)
        if GC.KP_DROPOUT_PROB > 0:
            drop = self.rng.random(kp.shape[0]) < GC.KP_DROPOUT_PROB
            kp[drop] = self.last_kp[drop] if hasattr(self, "last_kp") else kp[drop]
        self.last_kp = kp

        # latency: emit the detection from `lag` frames ago
        self.buf.append((frame_idx, kp))
        if len(self.buf) <= self.lag:
            return
        out_idx, out_kp = self.buf[0]
        m = Float32MultiArray()
        m.data = [float(out_idx)] + [float(v) for v in out_kp.reshape(-1)]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
