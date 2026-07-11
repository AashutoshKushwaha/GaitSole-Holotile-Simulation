#!/usr/bin/env bash
# Probe the running walking_world: list key topics, command a few joints, read back.
source /opt/ros/jazzy/setup.bash

echo "=== command + state + sensor topics ==="
gz topic -l 2>/dev/null | grep -iE "human/cmd|joint_state|^/camera|^/lidar" | sort

echo "=== publish targets: hip_flexion_r=0.6, knee_angle_r=-0.7 ==="
gz topic -t /human/cmd/hip_flexion_r -m gz.msgs.Double -p "data: 0.6" 2>/dev/null
gz topic -t /human/cmd/knee_angle_r  -m gz.msgs.Double -p "data: -0.7" 2>/dev/null
sleep 2.0

echo "=== joint_state readback (one message) ==="
timeout 3 gz topic -e -t /world/walking/model/human/joint_state -n 1 2>/dev/null \
  | grep -E "name:|position:" | head -40

echo "=== camera image header ==="
timeout 3 gz topic -i -t /camera 2>/dev/null | head -4
echo "PROBE_DONE"
