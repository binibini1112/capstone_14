#!/usr/bin/env bash
set -euo pipefail

HOST="${ULTRA96_HOST:-192.168.3.1}"
USER="${ULTRA96_USER:-xilinx}"
REMOTE_ROOT="${ULTRA_REMOTE_ROOT:-${ULTRA_YUBIN_V1_REMOTE_ROOT:-${ULTRA_CHAN_REMOTE_ROOT:-/home/xilinx/ultra_yubin_v1}}}"
SERVICE="${ULTRA_SERVICE:-${ULTRA_YUBIN_V1_SERVICE:-${ULTRA_CHAN_SERVICE:-ultra-yubin-v1.service}}}"
PORT="${ULTRA_YUBIN_V1_PORT:-${ULTRA_CHAN_PORT:-${ULTRA_YUBIN_PORT:-5016}}}"
BASE="${ULTRA_YUBIN_V1_BASE:-${ULTRA_CHAN_BASE:-${ULTRA_YUBIN_BASE:-0xA0000000}}}"
PAN_ID="${PAN_ID:-1}"
TILT_ID="${TILT_ID:-2}"
SERIAL="${ULTRA_YUBIN_V1_SERIAL:-${ULTRA_CHAN_SERIAL:-${ULTRA_YUBIN_SERIAL:-auto}}}"
BAUD="${ULTRA_YUBIN_V1_BAUD:-${ULTRA_CHAN_BAUD:-${ULTRA_YUBIN_BAUD:-57600}}}"
PROFILE_ACCEL="${ULTRA_YUBIN_V1_PROFILE_ACCEL:-${ULTRA_CHAN_PROFILE_ACCEL:-${ULTRA_YUBIN_PROFILE_ACCEL:-130}}}"
PROFILE_VELOCITY="${ULTRA_YUBIN_V1_PROFILE_VELOCITY:-${ULTRA_CHAN_PROFILE_VELOCITY:-${ULTRA_YUBIN_PROFILE_VELOCITY:-430}}}"
LASER_ENABLE="${ULTRA_YUBIN_V1_LASER_ENABLE:-${ULTRA_CHAN_LASER_ENABLE:-1}}"
LASER_ID="${ULTRA_YUBIN_V1_LASER_ID:-${ULTRA_CHAN_LASER_ID:-3}}"
LASER_CENTER="${ULTRA_YUBIN_V1_LASER_CENTER:-${ULTRA_CHAN_LASER_CENTER:-2048}}"
LASER_OFFSET_MM="${ULTRA_YUBIN_V1_CAMERA_LASER_OFFSET_MM:-${ULTRA_CHAN_CAMERA_LASER_OFFSET_MM:-38}}"
LASER_DISTANCE_MM="${ULTRA_YUBIN_V1_LASER_DISTANCE_MM:-${ULTRA_CHAN_LASER_DISTANCE_MM:-${ULTRA_YUBIN_LASER_DISTANCE_MM:-1000}}}"
LASER_VERTICAL_FOV_DEG="${ULTRA_YUBIN_V1_LASER_VERTICAL_FOV_DEG:-${ULTRA_CHAN_LASER_VERTICAL_FOV_DEG:-${ULTRA_YUBIN_LASER_VERTICAL_FOV_DEG:-43}}}"
# Laser is mounted 38 mm above the 0-degree optical axis, so the
# default parallax correction points the laser downward.
LASER_SIGN="${ULTRA_YUBIN_V1_LASER_SIGN:-${ULTRA_CHAN_LASER_SIGN:-${ULTRA_YUBIN_LASER_SIGN:--1}}}"
CENTER_FILE="${ULTRA_YUBIN_V1_CENTER_FILE:-${ULTRA_CHAN_CENTER_FILE:-${ULTRA_YUBIN_CENTER_FILE:-/home/xilinx/ultra_yubin_v1/front_center.env}}}"
DRY_RUN="${ULTRA_YUBIN_V1_DRY_RUN:-${ULTRA_CHAN_DRY_RUN:-${ULTRA_YUBIN_DRY_RUN:-auto}}}"
NO_PL="${ULTRA_YUBIN_V1_NO_PL:-${ULTRA_CHAN_NO_PL:-${ULTRA_YUBIN_NO_PL:-0}}}"
RESTART="${ULTRA_YUBIN_V1_RESTART:-${ULTRA_CHAN_RESTART:-${ULTRA_YUBIN_RESTART:-0}}}"
SKIP_CHECK="${ULTRA_YUBIN_V1_SKIP_CHECK:-${ULTRA_CHAN_SKIP_CHECK:-${ULTRA_YUBIN_SKIP_CHECK:-0}}}"
SKIP_PL_LOAD="${ULTRA_YUBIN_V1_SKIP_PL_LOAD:-${ULTRA_CHAN_SKIP_PL_LOAD:-${ULTRA_YUBIN_SKIP_PL_LOAD:-0}}}"
SKIP_PL_INIT="${ULTRA_YUBIN_V1_SKIP_PL_INIT:-${ULTRA_CHAN_SKIP_PL_INIT:-${ULTRA_YUBIN_SKIP_PL_INIT:-0}}}"
SKIP_DXL_INIT="${ULTRA_YUBIN_V1_SKIP_DXL_INIT:-${ULTRA_CHAN_SKIP_DXL_INIT:-${ULTRA_YUBIN_SKIP_DXL_INIT:-0}}}"
LAZY_PL_OPEN="${ULTRA_YUBIN_V1_LAZY_PL_OPEN:-${ULTRA_CHAN_LAZY_PL_OPEN:-${ULTRA_YUBIN_LAZY_PL_OPEN:-1}}}"
TRACK_DIRECT_PS="${ULTRA_YUBIN_V1_TRACK_DIRECT_PS:-${ULTRA_CHAN_TRACK_DIRECT_PS:-0}}"
TRACK_PL_SHADOW="${ULTRA_YUBIN_V1_TRACK_PL_SHADOW:-${ULTRA_CHAN_TRACK_PL_SHADOW:-0}}"
TRACK_PAN_LIMIT="${ULTRA_YUBIN_V1_TRACK_PAN_LIMIT:-${ULTRA_CHAN_TRACK_PAN_LIMIT:-900}}"
TRACK_TILT_LIMIT="${ULTRA_YUBIN_V1_TRACK_TILT_LIMIT:-${ULTRA_CHAN_TRACK_TILT_LIMIT:-240}}"
SUDO_PASS="${ULTRA96_SUDO_PASSWORD:-xilinx}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BIT="${ROOT}/bitstream/ultra_yubin_v1.bit"
HWH="${ROOT}/bitstream/ultra_yubin_v1.hwh"
BRIDGE="${ROOT}/hardware/pl_goal_compute/ps_app/pl_udp_usb_dxl_bridge.c"

test -s "${BIT}" || { echo "missing ${BIT}" >&2; exit 2; }
test -s "${BRIDGE}" || { echo "missing ${BRIDGE}" >&2; exit 2; }

echo "[ultra-yubin-v1] target=${USER}@${HOST} remote=${REMOTE_ROOT} port=${PORT} base=${BASE}"
ssh "${USER}@${HOST}" "mkdir -p '${REMOTE_ROOT}'"

echo "[ultra-yubin-v1] stopping previous ultra_yubin_v1 service/process"
ssh "${USER}@${HOST}" "printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl disable --now '${SERVICE}' >/dev/null 2>&1 || true
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' rm -f \
  /etc/systemd/system/'${SERVICE}' \
  /etc/systemd/system/multi-user.target.wants/'${SERVICE}'
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl daemon-reload"

SERVICE_ACTIVE="$(ssh "${USER}@${HOST}" "systemctl is-active '${SERVICE}' 2>/dev/null || true" | tr -d '\r')"
if [ "${SERIAL}" = "auto" ]; then
  SERIAL="$(ssh "${USER}@${HOST}" "set -- /dev/serial/by-id/*FTDI* /dev/serial/by-id/*Serial* /dev/ttyUSB* /dev/ttyACM*; for p in \"\$@\"; do [ -e \"\$p\" ] && { echo \"\$p\"; exit 0; }; done; echo /dev/ttyUSB0" | tr -d '\r')"
fi
SERIAL_PRESENT="$(ssh "${USER}@${HOST}" "test -e '${SERIAL}' && echo 1 || echo 0" | tr -d '\r')"

EFFECTIVE_DRY_RUN="${DRY_RUN}"
if [ "${DRY_RUN}" = "auto" ]; then
  if [ "${SERIAL_PRESENT}" = "1" ]; then
    EFFECTIVE_DRY_RUN="0"
  else
    EFFECTIVE_DRY_RUN="1"
  fi
fi

EFFECTIVE_NO_PL="${NO_PL}"
if [ "${NO_PL}" = "auto" ]; then
  EFFECTIVE_NO_PL="${EFFECTIVE_DRY_RUN}"
fi

echo "[ultra-yubin-v1] service=${SERVICE_ACTIVE:-inactive} serial_present=${SERIAL_PRESENT} dry=${EFFECTIVE_DRY_RUN} no_pl=${EFFECTIVE_NO_PL} restart=${RESTART}"
echo "[ultra-yubin-v1] skip_pl_load=${SKIP_PL_LOAD} skip_pl_init=${SKIP_PL_INIT} skip_dxl_init=${SKIP_DXL_INIT} lazy_pl_open=${LAZY_PL_OPEN} track_direct_ps=${TRACK_DIRECT_PS} track_pl_shadow=${TRACK_PL_SHADOW} track_pan_limit=${TRACK_PAN_LIMIT} track_tilt_limit=${TRACK_TILT_LIMIT}"
if [ "${SERVICE_ACTIVE}" = "active" ] && [ "${RESTART}" != "1" ]; then
  echo "[ultra-yubin-v1] service already active; updating files without restart."
  echo "[ultra-yubin-v1] set ULTRA_YUBIN_V1_RESTART=1 to force service restart."
fi

scp "${BIT}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin_v1.bit"
if [ -s "${HWH}" ]; then
  scp "${HWH}" "${USER}@${HOST}:${REMOTE_ROOT}/ultra_yubin_v1.hwh"
fi
scp "${BRIDGE}" "${USER}@${HOST}:${REMOTE_ROOT}/pl_udp_usb_dxl_bridge.c"

ssh "${USER}@${HOST}" "cd '${REMOTE_ROOT}' && gcc -O2 -Wall -o pl_udp_usb_dxl_bridge pl_udp_usb_dxl_bridge.c"

dry_arg=""
if [ "${EFFECTIVE_DRY_RUN}" = "1" ]; then
  dry_arg=" --dry-run"
fi
no_pl_arg=""
if [ "${EFFECTIVE_NO_PL}" = "1" ]; then
  no_pl_arg=" --no-pl"
fi
skip_pl_init_arg=""
if [ "${SKIP_PL_INIT}" = "1" ]; then
  skip_pl_init_arg=" --skip-pl-init"
fi
skip_dxl_init_arg=""
if [ "${SKIP_DXL_INIT}" = "1" ]; then
  skip_dxl_init_arg=" --skip-dxl-init"
fi
lazy_pl_open_arg=""
if [ "${LAZY_PL_OPEN}" = "1" ]; then
  lazy_pl_open_arg=" --lazy-pl-open"
fi
laser_arg=""
if [ "${LASER_ENABLE}" = "0" ]; then
  laser_arg=" --laser-disable"
fi
track_direct_arg=""
if [ "${TRACK_DIRECT_PS}" = "1" ]; then
  track_direct_arg=" --track-direct-ps"
fi
track_shadow_arg=""
if [ "${TRACK_PL_SHADOW}" = "1" ]; then
  track_shadow_arg=" --track-pl-shadow"
fi

ssh "${USER}@${HOST}" "printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl disable --now '${SERVICE}' >/dev/null 2>&1 || true
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' rm -f '/etc/systemd/system/${SERVICE}'
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' systemctl daemon-reload
for pid in \$(pidof pl_udp_usb_dxl_bridge 2>/dev/null || true); do
  ppid=\$(awk '/^PPid:/ {print \$2}' /proc/\"\$pid\"/status 2>/dev/null || true)
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' kill -9 \"\$pid\" 2>/dev/null || true
  if [ -n \"\$ppid\" ] && [ \"\$ppid\" != '1' ]; then
    printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' kill -9 \"\$ppid\" 2>/dev/null || true
  fi
done
if [ '${SKIP_PL_LOAD}' = '1' ]; then
  echo 'skip-pl-load mode: keeping current FPGA image'
elif [ '${EFFECTIVE_NO_PL}' != '1' ]; then
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' cp '${REMOTE_ROOT}/ultra_yubin_v1.bit' /lib/firmware/ultra_yubin_v1.bit
  printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' sh -c 'echo ultra_yubin_v1.bit > /sys/class/fpga_manager/fpga0/firmware'
  sleep 1
  cat /sys/class/fpga_manager/fpga0/state
else
  echo 'no-pl mode: skipping FPGA manager reload'
fi
printf '%s\n' '${SUDO_PASS}' | sudo -S -p '' /bin/sh -c 'setsid "\$0" "\$@" >/tmp/ultra_yubin_v1_manual.log 2>&1 < /dev/null &' '${REMOTE_ROOT}/pl_udp_usb_dxl_bridge' --base '${BASE}' --port '${PORT}' --serial '${SERIAL}' --baud '${BAUD}' --pan-id '${PAN_ID}' --tilt-id '${TILT_ID}' --profile-accel '${PROFILE_ACCEL}' --profile-velocity '${PROFILE_VELOCITY}' --track-pan-limit '${TRACK_PAN_LIMIT}' --track-tilt-limit '${TRACK_TILT_LIMIT}' --laser-id '${LASER_ID}' --laser-center '${LASER_CENTER}' --laser-offset-mm '${LASER_OFFSET_MM}' --laser-distance-mm '${LASER_DISTANCE_MM}' --laser-vertical-fov-deg '${LASER_VERTICAL_FOV_DEG}' --laser-sign '${LASER_SIGN}' --center-file '${CENTER_FILE}'${laser_arg}${dry_arg}${no_pl_arg}${skip_pl_init_arg}${skip_dxl_init_arg}${lazy_pl_open_arg}${track_direct_arg}${track_shadow_arg}
sleep 1
pidof pl_udp_usb_dxl_bridge || { cat /tmp/ultra_yubin_v1_manual.log; exit 1; }"

if [ "${SKIP_CHECK}" = "1" ]; then
  echo "[ultra-yubin-v1] skipping bring-up check because ULTRA_YUBIN_V1_SKIP_CHECK=1"
else
  cd "${ROOT}"
  python3 tools/pl_bringup_check.py --host "${HOST}" --port "${PORT}"
fi
