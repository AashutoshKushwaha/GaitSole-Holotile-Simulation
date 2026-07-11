# gazebo_gait

Gazebo simulation loop that tests the trained legs-only motion predictor
(`../motion_predictor/`) under live perception: a walking human is observed by a
simulated camera + lidar, the observations become 3D joint angles, the predictor
anticipates the next pose + foot force/moment at low latency, and the prediction
is verified against the human's actual steps.

**Project overview and result screenshots:** [../README.md](../README.md)

Example outputs (also copied to `../docs/assets/` for the main README):

| File | Description |
|------|-------------|
| `output/verify_curves.png` | Predicted vs actual step verification |
| `output/ov_*.png` | Perception overlay frames |
| `output/ref_*.png` | Reference gait frames |
| `output/final_*.png` | Final composite renders |

## Stack (all in WSL Ubuntu 24.04)
- Gazebo Harmonic (gz-sim 8) + ROS 2 Jazzy + `ros_gz_bridge` / `ros_gz_image`.
- Python: venv `~/venvs/gait` (`--system-site-packages`) = ROS rclpy + numpy + CPU torch 2.12.
- No colcon: nodes are plain Python scripts using std_msgs/sensor_msgs only (no
  custom .msg to compile). Run with the recipe below.

## Run recipe
```bash
source /opt/ros/jazzy/setup.bash
~/venvs/gait/bin/python nodes/<node>.py
```

## Pipeline & milestones
| Stage | Node | Status |
|---|---|---|
| G1 | env + scaffold (torch+rclpy proven, predictor 0.28 ms) | done |
| G2 | Gazebo world: walking human + camera + lidar | in progress |
| G3 | `perception_node`: ground-truth tap + synthetic noise/latency; record real cam/lidar | |
| G4 | `angle_solver_node`: keypoints -> 12 joint angles + root @ 100 Hz | |
| G5 | `predictor_node`: rolling window -> predict, low latency | |
| G6 | `verify_node`: predicted step vs actual step, metrics + plots | |
| G7 | real markerless vision swap (same `/perceived_keypoints` topic) | later |

See `gait_config.py` for the topic contract and sensor-noise parameters.

## Repository layout

```
gazebo_gait/
├── nodes/
│   ├── perception_node.py      # G3: noisy keypoints / sensor tap
│   ├── angle_solver_node.py    # G4: keypoints → joint angles
│   ├── predictor_node.py       # G5: rolling-window inference
│   ├── verify_node.py          # G6: metrics + plots
│   ├── gait_player.py          # Articulated walk in world
│   └── overlay_node.py         # Debug overlays
├── worlds/                     # gait_world.sdf, walking_world.sdf, …
├── meshes/                     # Limb OBJ meshes for articulated model
├── scripts/                    # run_pipeline.sh, run_verify.sh, …
├── tools/                      # Retargeting, mesh extract, frame export
├── gait_config.py              # Topics, noise, timing
├── fk.py                       # Forward kinematics helpers
└── output/                     # PNG results + logs (logs gitignored)
```

## Known stack constraint
gz-sim 8 segfaults when subscribing to `/world/*/dynamic_pose/info` while a
Gazebo `actor` is present (upstream bug gazebosim/gz-sim#2880). Ground truth is
therefore taken from a controllable articulated model, not from actor bone poses.
