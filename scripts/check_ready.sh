#!/usr/bin/env bash
# Read-only preflight for a selected object/model. This script never publishes
# an arm or gripper command. Defaults preserve the original object_003 trial.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
set -u

CAN_PORT="${CAN_PORT:-can0}"
OBJECT_ID="${OBJECT_ID:-object_003}"
EXPECTED_MODEL_ID="${EXPECTED_MODEL_ID:-model_plane_cleaned_fixed}"
if ! ip link show "${CAN_PORT}" | grep -q "UP"; then
  echo "FAIL: ${CAN_PORT} is missing or DOWN" >&2
  exit 1
fi
echo "OK: ${CAN_PORT} is UP"

check_sample() {
  local topic="$1"
  local expected_type="$2"
  local actual_type
  actual_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual_type}" != "${expected_type}" ]]; then
    echo "FAIL: ${topic} type=${actual_type:-missing}, expected=${expected_type}" >&2
    return 1
  fi
  if ! timeout 5s ros2 topic echo "${topic}" --once >/dev/null; then
    echo "FAIL: no fresh sample from ${topic}" >&2
    return 1
  fi
  echo "OK: ${topic} (${expected_type}) is live"
}

check_sample /feedback/tcp_pose geometry_msgs/msg/PoseStamped
check_sample /feedback/joint_states sensor_msgs/msg/JointState
check_sample /feedback/arm_status agx_arm_msgs/msg/AgxArmStatus
check_sample /feedback/gripper_status agx_arm_msgs/msg/GripperStatus
check_sample /foundationpose/object_pose_camera geometry_msgs/msg/PoseStamped
check_sample /foundationpose/object_pose_base geometry_msgs/msg/PoseStamped

arm_message="$(timeout 5s ros2 topic echo /feedback/arm_status --once 2>/dev/null || true)"
if [[ "${arm_message}" != *"ctrl_mode: 1"* && "${arm_message}" != *"ctrl_mode: 2"* ]]; then
  echo "FAIL: Piper v189 ctrl_mode must be 1 or 2 for seamless switching" >&2
  exit 1
fi
if [[ "${arm_message}" != *"arm_status: 0"* || "${arm_message}" != *"err_status: 0"* ]]; then
  echo "FAIL: Piper reports a nonzero arm/error status" >&2
  exit 1
fi
if [[ "${arm_message}" != *"teach_status: 0"* && "${arm_message}" != *"teach_status: 2"* ]]; then
  echo "FAIL: drag teaching is active; teach_status must be 0 or 2" >&2
  exit 1
fi
echo "OK: Piper reports an allowed seamless mode with zero arm/error status"

# Recovery is event-driven and may remain silent while tracking is healthy.
# Its interface must exist, but a fresh message is not required here.
recovery_type="$(ros2 topic type /foundationpose/recovery_required 2>/dev/null || true)"
if [[ "${recovery_type}" != "std_msgs/msg/Bool" ]]; then
  echo "FAIL: /foundationpose/recovery_required type=${recovery_type:-missing}, expected=std_msgs/msg/Bool" >&2
  exit 1
fi
echo "OK: /foundationpose/recovery_required interface exists (event-driven)"

model_message="$(timeout 5s ros2 topic echo /foundationpose/object_pose_json --once 2>/dev/null || true)"
if [[ "${model_message}" != *"${EXPECTED_MODEL_ID}"* ]]; then
  echo "FAIL: FoundationPose is not publishing ${OBJECT_ID} ${EXPECTED_MODEL_ID}" >&2
  exit 1
fi
echo "OK: FoundationPose model identity is ${EXPECTED_MODEL_ID} (${OBJECT_ID})"

move_j_type="$(ros2 topic type /control/move_j 2>/dev/null || true)"
if [[ "${move_j_type}" != "sensor_msgs/msg/JointState" ]]; then
  echo "FAIL: /control/move_j type=${move_j_type:-missing}, expected=sensor_msgs/msg/JointState" >&2
  exit 1
fi
echo "OK: /control/move_j joint-space command interface exists"
echo "READY: ${OBJECT_ID} dry-run may be started"
