#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
set -u

exec "${PIPER_ROS_PYTHON:-python3}" "${ROOT_DIR}/grasp_controller.py" \
  --config "${ROOT_DIR}/config/object_002_grasp.yaml" \
  "$@"
