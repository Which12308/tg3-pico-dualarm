#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$APP_DIR/scripts/robot_environment.py"
