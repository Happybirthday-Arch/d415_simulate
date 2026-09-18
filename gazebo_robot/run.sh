#!/usr/bin/env bash
set -euo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
world_name="${1:-bridge_robot}"
if [[ $# -gt 0 ]]; then shift; fi
world_file="$robot_root/worlds/$world_name.sdf"
if [[ ! -f "$world_file" ]]; then
  echo "找不到世界文件：$world_file" >&2
  exit 1
fi
if [[ ! -f "$robot_root/build/libMagneticRobot.so" ]]; then
  echo "请先运行 $robot_root/build.sh" >&2
  exit 1
fi
export MAGNETIC_ROBOT_RESOURCE_ROOT="$robot_root"
export IGN_GAZEBO_RESOURCE_PATH="$robot_root/models:$robot_root/../gazebo_bridge/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$robot_root/build${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
exec ign gazebo -r -v 2 "$world_file" "$@"
