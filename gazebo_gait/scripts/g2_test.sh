#!/usr/bin/env bash
# Self-contained G2 check: launch the walking world headless, command a few leg
# joints, confirm the JointPositionController tracks them, confirm camera/lidar
# publish, then shut down. Run: bash scripts/g2_test.sh
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait

gz sim -s -r -v1 worlds/walking_world.sdf > output/gz_walk.log 2>&1 &
GZ_PID=$!
echo "gz server pid=$GZ_PID, warming up..."
sleep 8

echo "=== key topics ==="
gz topic -l 2>/dev/null | grep -iE "human/cmd|joint_state|^/camera$|^/lidar" | sort

echo "=== command hip_flexion_r=0.6, knee_angle_r=-0.7, ankle_angle_r=0.2 ==="
gz topic -t /human/cmd/hip_flexion_r -m gz.msgs.Double -p "data: 0.6"  2>/dev/null
gz topic -t /human/cmd/knee_angle_r  -m gz.msgs.Double -p "data: -0.7" 2>/dev/null
gz topic -t /human/cmd/ankle_angle_r -m gz.msgs.Double -p "data: 0.2"  2>/dev/null
sleep 2.5

echo "=== joint_state readback (name/position pairs) ==="
timeout 3 gz topic -e -t /world/walking/model/human/joint_state -n 1 2>/dev/null \
  | grep -E "name:|position:" | head -48

echo "=== camera publishing? ==="
timeout 4 gz topic -e -t /camera -n 1 2>/dev/null | grep -E "width:|height:|step:" | head -3
echo "=== lidar publishing? ==="
timeout 4 gz topic -e -t /lidar/points -n 1 2>/dev/null | grep -E "width:|height:|row_step:" | head -3

kill $GZ_PID 2>/dev/null
wait $GZ_PID 2>/dev/null
echo "G2_TEST_DONE"
