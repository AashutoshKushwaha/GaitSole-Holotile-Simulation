#!/usr/bin/env python3
"""
Angle solver (G4): perceived 3D keypoints -> 12 joint angles for the predictor.

Subscribes the noisy keypoints, runs fk.solve_angles (exact inverse of the
keypoint FK), and republishes the recovered pose in motion_predictor POSE_COLS
order. This is the "skeleton tracking -> kinematics" stage; with sensor noise the
recovered angles are noisy, which is exactly what we want to feed the predictor.

In:  PERCEIVED_KP_TOPIC  Float32MultiArray [frame_idx, *(K*3)]
Out: JOINT_ANGLES_TOPIC  Float32MultiArray [frame_idx, *pose12]  (POSE_COLS order)
"""
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


class AngleSolver(Node):
    def __init__(self):
        super().__init__("angle_solver_node")
        self.pub = self.create_publisher(Float32MultiArray, GC.JOINT_ANGLES_TOPIC, 10)
        self.sub = self.create_subscription(
            Float32MultiArray, GC.PERCEIVED_KP_TOPIC, self.cb, 50)
        self.get_logger().info("angle_solver_node: keypoints -> 12 joint angles")

    def cb(self, msg):
        idx = msg.data[0]
        kp = np.array(msg.data[1:]).reshape(-1, 3)
        ang = fk.solve_angles(kp)
        m = Float32MultiArray()
        m.data = [float(idx)] + [float(ang[c]) for c in MC.POSE_COLS]
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(AngleSolver())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
