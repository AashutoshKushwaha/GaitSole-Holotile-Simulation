#!/usr/bin/env bash
# One-command launcher for the smooth walking-human simulation (Gazebo GUI).
# Run from a WSL Ubuntu terminal:  bash /mnt/e/OpenSim/gazebo_gait/scripts/view.sh
source /opt/ros/jazzy/setup.bash
cd /mnt/e/OpenSim/gazebo_gait
# Force the GUI (Qt) onto X11, which is the reliable WSLg socket (Wayland can
# show a blank/no window under WSLg).
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0
echo "Launching Gazebo GUI... (first start takes 10-20s; a window should appear)"
echo "If no window appears, close this and tell Claude - we'll use the video instead."
exec gz sim -r worlds/gait_world.sdf
