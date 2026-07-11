#!/usr/bin/env bash
# Tracking+prediction overlay demo: actor world + camera bridge + overlay_node
# -> output/overlay.mp4 (tracked green + predicted red skeleton on the feed).
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT
gz sim -s -r -v1 worlds/gait_world.sdf > output/gz_overlay.log 2>&1 & PIDS+=($!)
sleep 9
ros2 run ros_gz_bridge parameter_bridge \
  camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r camera:=/camera/image \
  > output/bridge_overlay.log 2>&1 & PIDS+=($!)
sleep 4
~/venvs/gait/bin/python nodes/overlay_node.py 2>&1 | tail -4
echo "OVERLAY_DONE"
