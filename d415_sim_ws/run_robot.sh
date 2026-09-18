#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
if [ ! -f install/bridge_robot_sim/share/bridge_robot_sim/package.xml ]; then
  echo '请先运行 ./build_robot.sh' >&2
  exit 1
fi
source install/setup.bash
mkdir -p runtime/ros_logs
export ROS_LOG_DIR="$PWD/runtime/ros_logs"
export IGN_PARTITION="${IGN_PARTITION:-bridge_robot_camera}"
exec ros2 launch bridge_robot_sim robot_camera_bridge.launch.py "$@"
