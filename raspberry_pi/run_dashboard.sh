#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON="${PYTHON:-.venv/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  echo "Python not found: $PYTHON" >&2
  echo "Run: python3 -m venv .venv && .venv/bin/python -m pip install -e \".[dev]\"" >&2
  exit 1
fi

exec "$PYTHON" -m tello_control.dashboard \
  --host "${DASHBOARD_HOST:-0.0.0.0}" \
  --port "${DASHBOARD_PORT:-8000}" \
  --telemetry-host "${TELEMETRY_HOST:-0.0.0.0}" \
  --telemetry-port "${TELEMETRY_PORT:-5005}" \
  --open-browser \
  --browser-display "${BROWSER_DISPLAY:-:0}" \
  "$@"
