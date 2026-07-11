#!/usr/bin/env python3
"""
Gait player: the single source of truth for the walking motion.

Loads a real Camargo treadmill trial (via motion_predictor/camargo.py) and, on a
100 Hz timer:
  * drives the Gazebo humanoid's 8 leg joints  -> std_msgs/Float64 on
    /human/cmd/<joint>  (ros_gz_bridge forwards these to the gz controllers), and
  * publishes the ground-truth pose (12 joint angles, motion_predictor POSE_COLS
    order) -> std_msgs/Float32MultiArray on GROUND_TRUTH_TOPIC, layout
    [frame_idx, *pose12]. Everything downstream (perception, verify) keys off this.

This owns the motion, so ground truth never has to be read back from Gazebo
(which also dodges the gz-sim actor pose-topic segfault).

Run:  source /opt/ros/jazzy/setup.bash && ~/venvs/gait/bin/python nodes/gait_player.py
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gait_config as GC  # noqa: E402  gazebo_gait/gait_config.py
import fk                 # noqa: E402  gazebo_gait/fk.py (pelvis grounding)

sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC   # noqa: E402  motion_predictor/config.py (bare name required by camargo/data)
import camargo        # noqa: E402

CAMARGO_CSV = os.path.join(GC.MOTION_PREDICTOR_DIR, "data", "camargo_csv")

# The 8 driven leg joints (must match build_human_sdf.py joint names).
LEG_JOINTS = [f"{j}_{s}" for s in ("r", "l")
              for j in ("hip_flexion", "hip_adduction", "knee_angle", "ankle_angle")]

# OpenSim angle -> gz joint command:  cmd = SIGN * angle + OFFSET (radians).
# Signs make the SDF axes anatomical: thigh swings forward at +flexion, knee folds
# backward, adduction pulls toward midline (mirror L/R). Verified by side render.
CALIB = {
    "hip_flexion_r": (-1.0, 0.0), "hip_flexion_l": (-1.0, 0.0),
    "hip_adduction_r": (-1.0, 0.0), "hip_adduction_l": (1.0, 0.0),
    "knee_angle_r": (-1.0, 0.0), "knee_angle_l": (-1.0, 0.0),
    "ankle_angle_r": (-1.0, 0.0), "ankle_angle_l": (-1.0, 0.0),
}


class GaitPlayer(Node):
    def __init__(self):
        super().__init__("gait_player")
        # create ROS entities BEFORE loading data: importing/using pandas after
        # node creation can invalidate the rcl context (pandas 3.x interaction).
        self.cmd_pubs = {j: self.create_publisher(Float64, f"/human/cmd/{j}", 10)
                         for j in LEG_JOINTS}
        self.pz_pub = self.create_publisher(Float64, "/human/cmd/pelvis_ty", 10)
        self.gt_pub = self.create_publisher(Float32MultiArray, GC.GROUND_TRUTH_TOPIC, 10)

        trials = camargo.load_camargo(CAMARGO_CSV)
        self.tr = trials[0]
        self.pose = np.stack([self.tr[c] for c in MC.POSE_COLS], axis=1)  # [N,12]
        self.N = self.pose.shape[0]
        self.i = 0

        self.timer = self.create_timer(1.0 / GC.FPS, self.tick)
        self.get_logger().info(
            f"gait_player: {self.N} frames @ {GC.FPS:.0f} Hz "
            f"({self.N / GC.FPS:.1f}s), driving {len(LEG_JOINTS)} joints")

    def tick(self):
        pose = self.pose[self.i]                       # 12 angles (rad)
        pose_by_name = dict(zip(MC.POSE_COLS, pose))

        cmd = {}
        for j in LEG_JOINTS:
            sign, off = CALIB[j]
            cmd[j] = float(sign * pose_by_name[j] + off)
            msg = Float64()
            msg.data = cmd[j]
            self.cmd_pubs[j].publish(msg)

        # drive pelvis height so the lower foot stays on the floor (grounding + bob)
        pz = Float64()
        pz.data = float(fk.pelvis_height(cmd))
        self.pz_pub.publish(pz)

        gt = Float32MultiArray()
        gt.data = [float(self.i)] + [float(v) for v in pose]
        self.gt_pub.publish(gt)

        self.i = (self.i + 1) % self.N


def main():
    rclpy.init()
    node = GaitPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
