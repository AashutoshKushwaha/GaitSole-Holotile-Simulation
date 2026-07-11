#!/usr/bin/env bash
# Confirm JointPositionController loaded and tracks a command. Dumps raw
# joint_state structure and checks the /human/cmd topic before/after publish.
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait

gz sim -s -r -v1 worlds/walking_world.sdf > output/gz_walk.log 2>&1 &
GZ_PID=$!
sleep 8

echo "=== controller plugin load lines in log ==="
grep -iE "JointPositionController|position.controller|Loaded plugin|Error|err" output/gz_walk.log | head -20

echo "=== all /human topics ==="
gz topic -l 2>/dev/null | grep -i human | sort

echo "=== raw joint_state (one msg, first 60 lines) ==="
timeout 3 gz topic -e -t /world/walking/model/human/joint_state -n 1 2>/dev/null | head -60

echo "=== command hip_flexion_r=0.6 then read its joint block ==="
gz topic -t /human/cmd/hip_flexion_r -m gz.msgs.Double -p "data: 0.6" 2>/dev/null
gz topic -t /human/cmd/knee_angle_r  -m gz.msgs.Double -p "data: -0.7" 2>/dev/null
sleep 2.5
echo "--- hip_flexion_r block (expect axis1 position ~0.6) ---"
timeout 3 gz topic -e -t /world/walking/model/human/joint_state -n 1 2>/dev/null \
  | grep -A24 'name: "hip_flexion_r"' | grep -E "name:|position:" | head -4
echo "--- knee_angle_r block (expect axis1 position ~-0.7) ---"
timeout 3 gz topic -e -t /world/walking/model/human/joint_state -n 1 2>/dev/null \
  | grep -A24 'name: "knee_angle_r"' | grep -E "name:|position:" | head -4

kill $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null
echo "TRACK_DONE"
