"""
Shared configuration for the Gazebo gait-perception -> predictor -> verify loop.

The pipeline (one ROS 2 Jazzy graph in WSL, Gazebo Harmonic):

    Gazebo (walking human + camera + lidar)
       |  ground-truth link poses          camera image + lidar cloud
       v                                     |  (bridged + recorded for the
    [perception_node]  <-- Option A          |   future real-vision swap, G7)
       |  PERCEIVED_KP_TOPIC  (3D keypoints + synthetic noise & latency)
       v
    [angle_solver_node]  keypoints -> 12 joint angles + root, resampled to 100 Hz
       |  JOINT_ANGLES_TOPIC  (matches motion_predictor POSE_COLS order)
       v
    [predictor_node]  rolling T_IN window -> predict next pose + foot force/moment
       |  PREDICTION_TOPIC  (+ measured latency)
       v
    [verify_node]  predicted next step  vs  the human's actual step

Run recipe (proven in G1):
    source /opt/ros/jazzy/setup.bash && ~/venvs/gait/bin/python <node>.py
The venv at ~/venvs/gait was built with --system-site-packages so it sees ROS's
rclpy + system numpy, plus its own CPU torch (2.12). predictor.pt loads here and
runs at ~0.28 ms median (~3500 fps).
"""

import os

# --- filesystem (Windows paths as seen from WSL) ----------------------------
# This package lives on the Windows drive; WSL reads it under /mnt/e.
PKG_DIR = os.path.dirname(os.path.abspath(__file__))                 # native
MOTION_PREDICTOR_DIR = "/mnt/e/OpenSim/motion_predictor"             # WSL view
GAZEBO_GAIT_WSL = "/mnt/e/OpenSim/gazebo_gait"                       # WSL view
OUTPUT_DIR = os.path.join(PKG_DIR, "output")

# --- ROS topic contract (the seams between nodes) ---------------------------
PERCEIVED_KP_TOPIC = "/perceived_keypoints"   # std_msgs/Float32MultiArray: 3D kp
JOINT_ANGLES_TOPIC = "/joint_angles"          # std_msgs/Float32MultiArray: pose+root
PREDICTION_TOPIC   = "/prediction"            # std_msgs/Float32MultiArray: pred bundle
GROUND_TRUTH_TOPIC = "/gait_ground_truth"     # std_msgs/Float32MultiArray: true angles
CAMERA_TOPIC = "/camera/image"                # sensor_msgs/Image
LIDAR_TOPIC  = "/lidar/points"                # sensor_msgs/PointCloud2

# --- timing -----------------------------------------------------------------
FPS = 100.0          # pipeline rate; matches motion_predictor.config.FPS

# --- synthetic-sensor model (Option A: noise + latency on ground truth) -----
# Mirrors the HoloTile "live sensors = noise+latency on true state" decision.
KP_POS_NOISE_M   = 0.0
KP_LATENCY_S     = 0.0
KP_DROPOUT_PROB  = 0.0     # per-frame chance a keypoint is dropped (0 = off)
