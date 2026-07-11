#!/usr/bin/env python3
"""Grab a few /perceived_keypoints messages, decode, and sanity-print them."""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC
import fk


class Check(Node):
    def __init__(self):
        super().__init__("check_perception")
        self.n = 0
        self.create_subscription(Float32MultiArray, GC.PERCEIVED_KP_TOPIC, self.cb, 10)
        self.get_logger().info("waiting for /perceived_keypoints ...")

    def cb(self, msg):
        idx = msg.data[0]
        kp = np.array(msg.data[1:]).reshape(-1, 3)
        d = dict(zip(fk.KEYPOINT_NAMES, kp))
        self.get_logger().info(
            f"frame {idx:.0f} | pelvis z={d['pelvis'][2]:.3f} "
            f"ankle_r z={d['ankle_r'][2]:.3f} toe_r z={d['toe_r'][2]:.3f} "
            f"head z={d['head'][2]:.3f} | {kp.shape[0]} kps")
        self.n += 1
        if self.n >= 4:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Check())
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
