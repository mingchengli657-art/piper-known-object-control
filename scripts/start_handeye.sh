#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
set -u

exec "${PIPER_ROS_PYTHON:-python3}" "${ROOT_DIR}/handeye_pose_bridge.py" \
  --config "${1:-${ROOT_DIR}/config/online_handeye.yaml}"
