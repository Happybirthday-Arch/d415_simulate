#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
source install/setup.bash
session="${1:-}"
if [ -z "$session" ]; then
  session=$(python3 - <<'PY'
from pathlib import Path
paths=sorted(p.parent for p in Path('recordings').glob('*/run_config.yaml') if (p.parent/'bag/metadata.yaml').is_file())
if not paths: raise SystemExit('没有已结束的录制。')
print(paths[-1].resolve())
PY
)
else
  shift
fi
export ROS_LOG_DIR="$PWD/runtime/ros_logs"
exec ros2 launch bridge_d415_sim replay.launch.py "session:=$session" "$@"
