#!/usr/bin/env bash
# Source ROS Humble and the local AGX arm overlay without assuming the caller's cwd.
set -eo pipefail

PIPER_CONTROL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"

DEPENDENCIES_SH="${ROBOT_DEPENDENCIES_FILE:-}"
if [[ -n "${DEPENDENCIES_SH}" && -f "${DEPENDENCIES_SH}" ]]; then
  # shellcheck disable=SC1090
  source "${DEPENDENCIES_SH}"
fi

if [[ -n "${AGX_ARM_SETUP:-}" ]]; then
  AGX_SETUP_FILE="${AGX_ARM_SETUP}"
elif [[ -n "${AGX_ARM_WS:-}" ]]; then
  AGX_SETUP_FILE="${AGX_ARM_WS}/install/setup.bash"
else
  echo "Set AGX_ARM_WS or AGX_ARM_SETUP to your installed AGX workspace." >&2
  return 2 2>/dev/null || exit 2
fi
if [[ ! -f "${AGX_SETUP_FILE}" ]]; then
  echo "AGX arm overlay not found: ${AGX_SETUP_FILE}" >&2
  echo "Set AGX_ARM_WS=/path/to/agx_arm_ws or AGX_ARM_SETUP=/path/to/setup.bash" >&2
  return 2 2>/dev/null || exit 2
fi
# shellcheck disable=SC1090
source "${AGX_SETUP_FILE}"
