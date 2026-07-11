#!/usr/bin/env bash
# Full pipeline + verification: gait_player -> perception -> angle_solver ->
# predictor -> verify_node (writes output/verify_curves.png + verify_metrics.csv).
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT
for n in gait_player perception_node angle_solver_node predictor_node; do
  ~/venvs/gait/bin/python nodes/$n.py > output/$n.log 2>&1 &
  PIDS+=($!)
done
sleep 6
~/venvs/gait/bin/python nodes/verify_node.py 2>&1 | tail -16
echo "VERIFY_DONE"
