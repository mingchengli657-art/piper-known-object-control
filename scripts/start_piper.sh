#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
set -u

ENABLE_CONTROL=false
if [[ "${1:-}" == "--enable-control" ]]; then
  ENABLE_CONTROL=true
  shift
fi

CAN_PORT="${CAN_PORT:-can0}"
if ! ip link show "${CAN_PORT}" >/dev/null 2>&1; then
  echo "CAN interface ${CAN_PORT} does not exist or is not available." >&2
  echo "Activate it with the agx_arm_ros/scripts/can_activate.sh helper first." >&2
  exit 2
fi
if ! ip link show "${CAN_PORT}" | grep -q "UP"; then
  echo "CAN interface ${CAN_PORT} is not UP." >&2
  exit 2
fi

MIN_CAN_TX_QUEUE_LEN="${MIN_CAN_TX_QUEUE_LEN:-1000}"
CURRENT_CAN_TX_QUEUE_LEN="$(
  ip -details link show "${CAN_PORT}" \
    | grep -oP 'qlen \K\d+' \
    | head -n 1 \
    || true
)"
if [[ -z "${CURRENT_CAN_TX_QUEUE_LEN}" ]] \
  || (( CURRENT_CAN_TX_QUEUE_LEN < MIN_CAN_TX_QUEUE_LEN )); then
  echo "CAN interface ${CAN_PORT} txqueuelen=${CURRENT_CAN_TX_QUEUE_LEN:-unknown}; expected at least ${MIN_CAN_TX_QUEUE_LEN}." >&2
  echo "Run the updated can_activate.sh before starting Piper." >&2
  exit 2
fi

exec ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:="${CAN_PORT}" \
  arm_type:=piper_x \
  fw_version:=v189 \
  auto_enable:="${ENABLE_CONTROL}" \
  control_enabled:="${ENABLE_CONTROL}" \
  speed_percent:="${SPEED_PERCENT:-10}" \
  effector_type:=agx_gripper \
  "$@"
