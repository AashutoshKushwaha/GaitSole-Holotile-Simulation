#!/usr/bin/env bash
# Download the Fuel human actor + confirm a skinned actor renders. gz + camera
# bridge + snapshots. First run downloads the mesh (needs internet), so allow time.
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
WARM=${1:-25}
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

gz sim -s -r -v2 worlds/actor_probe.sdf > output/gz_actor.log 2>&1 &
PIDS+=($!)
echo "warming up / downloading actor (${WARM}s)..."
sleep "$WARM"

ros2 run ros_gz_bridge parameter_bridge \
  camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r camera:=/camera/image \
  > output/bridge_actor.log 2>&1 &
PIDS+=($!)
sleep 4

~/venvs/gait/bin/python nodes/snap_camera.py 3 1.0 2>&1 | tail -6
echo "--- gz log: actor/skin/download lines ---"
grep -iE "actor|walk.dae|skel|skin|download|fetch|fuel|Err" output/gz_actor.log | tail -15
echo "ACTOR_PROBE_DONE"
