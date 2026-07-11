#!/usr/bin/env python3
"""Grab a few /prediction messages and print decoded predictions + latency."""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC
sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC


class Check(Node):
    def __init__(self):
        super().__init__("check_prediction")
        self.n = 0
        self.ki = MC.POSE_COLS.index("knee_angle_r")
        self.gy = MC.KINETIC_COLS.index("grf_y_r")
        self.create_subscription(Float32MultiArray, GC.PREDICTION_TOPIC, self.cb, 10)
        self.get_logger().info("waiting for /prediction ...")

    def cb(self, msg):
        idx = msg.data[0]
        pose = np.array(msg.data[1:1 + len(MC.POSE_COLS)])
        kin = np.array(msg.data[1 + len(MC.POSE_COLS):1 + len(MC.POSE_COLS) + len(MC.KINETIC_COLS)])
        lat = msg.data[-1]
        self.get_logger().info(
            f"frame {idx:.0f} | pred R-knee {np.rad2deg(pose[self.ki]):6.1f} deg | "
            f"pred R-GRFy {kin[self.gy]:5.2f} BW | latency {lat:.3f} ms")
        self.n += 1
        if self.n >= 5:
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
