#!/usr/bin/env bash
# Run the full data pipeline (no Gazebo needed): gait_player -> perception ->
# angle_solver -> predictor, then sample /prediction. Verifies G3-G5 end to end.
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT
for n in gait_player perception_node angle_solver_node predictor_node; do
  ~/venvs/gait/bin/python nodes/$n.py > output/$n.log 2>&1 &
  PIDS+=($!)
done
sleep 10
~/venvs/gait/bin/python nodes/check_prediction.py 2>&1 | tail -7
echo "--- predictor latency summary ---"; grep -i latency output/predictor_node.log | tail -2
echo "PIPELINE_DONE"
