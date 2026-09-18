#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
  echo '请先运行 ./build.sh' >&2
  exit 1
fi
source install/setup.bash
mkdir -p runtime/ros_logs
export ROS_LOG_DIR="$PWD/runtime/ros_logs"
# Separate Gazebo transport partition; all child processes inherit it.
export IGN_PARTITION="${IGN_PARTITION:-bridge_d415}"
exec ros2 launch bridge_d415_sim camera.launch.py "$@"
