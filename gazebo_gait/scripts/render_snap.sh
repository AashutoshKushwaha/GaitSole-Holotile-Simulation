#!/usr/bin/env bash
# Render a world headless, bridge the camera, snap N frames to output/.
# Usage: bash scripts/render_snap.sh <world.sdf> [warmup_s] [n_snaps]
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
WORLD=${1:?world path}
WARM=${2:-10}
N=${3:-2}
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

gz sim -s -r -v1 "$WORLD" > output/gz_render.log 2>&1 &
PIDS+=($!)
sleep "$WARM"
ros2 run ros_gz_bridge parameter_bridge \
  camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r camera:=/camera/image \
  > output/bridge_render.log 2>&1 &
PIDS+=($!)
sleep 4
~/venvs/gait/bin/python nodes/snap_camera.py "$N" 1.0 2>&1 | tail -4
echo "--- gz errors (if any) ---"; grep -iE "err|warn|skel|fail" output/gz_render.log | grep -ivE "fuel|LocalCache" | tail -8
echo "RENDER_SNAP_DONE"
