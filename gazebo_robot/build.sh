#!/usr/bin/env bash
set -euo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$robot_root/scripts/generate_model.py"
cmake -S "$robot_root" -B "$robot_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$robot_root/build" -j2
ctest --test-dir "$robot_root/build" --output-on-failure
