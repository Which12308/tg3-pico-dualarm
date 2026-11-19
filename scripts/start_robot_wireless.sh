#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_HOME=/home/ubuntu
if [[ ! -f "$ROBOT_HOME/xos/setup.bash" ]]; then
  echo '缺少 ~/xos/setup.bash，未启动机器人接收程序' >&2
  exit 2
fi
source "$ROBOT_HOME/xos/setup.bash"
MANAGED_XARM=0
for argument in "$@"; do
  if [[ "$argument" == '--xarm' ]]; then
    if [[ ! -f "$ROBOT_HOME/XARM/install/setup.bash" ]]; then
      echo '未找到 ~/XARM/install/setup.bash；需先安装与本机匹配的厂家 XARM，不会自动切换原厂系统。' >&2
      exit 2
    fi
    source "$ROBOT_HOME/XARM/install/setup.bash"
    MANAGED_XARM=1
  fi
done
if [[ ! -f "$APP_DIR/.venv/tg3_environment.json" ]]; then
  echo '项目独立环境尚未完成原生验收；请从电脑启动器部署' >&2
  exit 2
fi
export PYTHONNOUSERSITE=1
export PYTHONPATH="$APP_DIR/src:${PYTHONPATH:-}"
if [[ "$MANAGED_XARM" == 1 ]]; then
  if [[ "$EUID" != 0 ]]; then
    echo 'XARM 接管需要操作台通过 sudo 启动，密码仅通过本次 SSH 标准输入传递。' >&2
    exit 2
  fi
  exec /usr/bin/python3 -s -m tg3_pico_dualarm.supervised_station \
    --config "$APP_DIR/config" --record "$APP_DIR/recordings" --ros "$@"
fi
export LD_LIBRARY_PATH="$APP_DIR/.venv/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
exec "$APP_DIR/.venv/bin/python" -s -m tg3_pico_dualarm.station --key-stdin \
  --config "$APP_DIR/config" --record "$APP_DIR/recordings" --ros "$@"
