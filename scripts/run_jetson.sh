#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "${ROOT}"

HEADLESS=0
for arg in "$@"; do
  if [ "${arg}" = "--headless" ]; then
    HEADLESS=1
    break
  fi
done

if [ -n "${PYTHON:-}" ]; then
  PY="${PYTHON}"
elif [ "${HEADLESS}" = "0" ] && [ -x "/home/jetson/yubin/.venv/bin/python3" ]; then
  PY="/home/jetson/yubin/.venv/bin/python3"
elif [ -x "${ROOT}/.venv/bin/python3" ]; then
  PY="${ROOT}/.venv/bin/python3"
else
  PY="python3"
fi

echo "[run_jetson] python=${PY}"
exec "${PY}" jetson/jetson_node.py "$@"
