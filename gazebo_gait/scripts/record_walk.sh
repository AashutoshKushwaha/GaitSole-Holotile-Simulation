#!/usr/bin/env bash
# Render a world headless + bridge camera + record an MP4 and a few stills.
# Usage: bash scripts/record_walk.sh <world.sdf> [name] [n_frames]
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
WORLD=${1:?world path}
NAME=${2:-walk}
NF=${3:-150}
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

gz sim -s -r -v1 "$WORLD" > output/gz_render.log 2>&1 &
PIDS+=($!)
sleep 9
ros2 run ros_gz_bridge parameter_bridge \
  camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r camera:=/camera/image \
  > output/bridge_render.log 2>&1 &
PIDS+=($!)
sleep 4
# stills across the cycle for frame-by-frame calibration checks
~/venvs/gait/bin/python nodes/snap_camera.py 6 0.28 2>&1 | tail -2
# the video
~/venvs/gait/bin/python nodes/record_camera.py "$NF" "$NAME" 30 2>&1 | tail -2
echo "RECORD_DONE"
