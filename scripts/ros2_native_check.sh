#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$HOME/xos/setup.bash"
command -v colcon >/dev/null || { echo '缺少 colcon，未执行 ROS 2 构建' >&2; exit 2; }
mkdir -p "$APP_DIR/artifacts"
export PYTHONNOUSERSITE=1
export LD_LIBRARY_PATH="$APP_DIR/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
colcon --log-base "$APP_DIR/artifacts/ros2_logs" build --base-paths "$APP_DIR" \
  --packages-select tg3_pico_dualarm --build-base "$APP_DIR/artifacts/ros2_build" \
  --install-base "$APP_DIR/artifacts/ros2_install"
source "$APP_DIR/artifacts/ros2_install/setup.bash"
"$APP_DIR/.venv/bin/python" -s "$APP_DIR/scripts/lab_check.py" --ros
