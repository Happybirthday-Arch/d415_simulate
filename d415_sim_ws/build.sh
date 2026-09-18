#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gz_camera_macros realsense2_gz_description bridge_d415_sim
