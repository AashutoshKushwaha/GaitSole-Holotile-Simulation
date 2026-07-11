#!/usr/bin/env bash
# Render an actor world (self-animating) headless + camera bridge + record MP4.
# Usage: bash scripts/record_actor.sh <world.sdf> [name] [n_frames]
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
WORLD=${1:?world}; NAME=${2:-walk_actor}; NF=${3:-150}
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT
gz sim -s -r -v1 "$WORLD" > output/gz_actor.log 2>&1 & PIDS+=($!)
sleep 9
ros2 run ros_gz_bridge parameter_bridge \
  camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r camera:=/camera/image \
  > output/bridge_actor.log 2>&1 & PIDS+=($!)
sleep 4
~/venvs/gait/bin/python nodes/record_camera.py "$NF" "$NAME" 30 2>&1 | tail -2
echo "RECORD_ACTOR_DONE"
