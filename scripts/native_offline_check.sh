#!/usr/bin/env bash
set -eo pipefail
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${TG3_IK_PYTHON:-$APP_DIR/.venv/bin/python}"
export PYTHONPATH="$APP_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" -s -m tg3_pico_dualarm selftest
"$PYTHON" -s "$APP_DIR/scripts/run_tests.py"
