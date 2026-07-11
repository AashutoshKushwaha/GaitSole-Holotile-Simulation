#!/usr/bin/env bash
# Bring up the walking demo: gz server + ros_gz bridge + gait player, let it walk,
# snap a few camera frames, then tear everything down.
# Usage: bash scripts/run_walk.sh [run_seconds] [n_snaps]
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait

RUN_S=${1:-10}
N_SNAPS=${2:-3}
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

echo "[1/4] gz server"
gz sim -s -r -v1 worlds/walking_world.sdf > output/gz_walk.log 2>&1 &
PIDS+=($!)
sleep 7

echo "[2/4] ros_gz bridge"
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/mnt/e/OpenSim/gazebo_gait/launch/bridge.yaml \
  > output/bridge.log 2>&1 &
PIDS+=($!)
sleep 4

echo "[3/4] gait player"
~/venvs/gait/bin/python nodes/gait_player.py > output/gait_player.log 2>&1 &
PIDS+=($!)
sleep "$RUN_S"

echo "[4/4] snapshots"
~/venvs/gait/bin/python nodes/snap_camera.py "$N_SNAPS" 0.35 2>&1 | tail -8
~/venvs/gait/bin/python nodes/record_camera.py 150 walk_capsule 30 2>&1 | tail -2

echo "--- gait_player log tail ---"; tail -3 output/gait_player.log
echo "--- bridge log tail ---";      tail -3 output/bridge.log
echo "RUN_WALK_DONE"
