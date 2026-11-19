#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$APP_DIR/.venv/bin/python" ]]; then
  bash "$APP_DIR/setup_linux.sh"
fi
exec "$APP_DIR/.venv/bin/python" -s -m tg3_pico_dualarm serve --config "$APP_DIR/config" --record "$APP_DIR/recordings" "$@"
