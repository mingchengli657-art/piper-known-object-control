#!/usr/bin/env python3
"""One-shot, explicitly armed grasp controller for a known FoundationPose model.

The controller waits for a stable base-frame object pose, freezes it, opens
the AGX gripper, moves to the previously validated camera-standoff target,
closes until either the requested width or stable contact is observed, and
then performs one linear +Z lift.  It never enables the robot or opens the
driver control gate; ``--execute`` is required before any command is sent.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from agx_arm_msgs.msg import AgxArmStatus, GripperStatus

from grasp_geometry import (
    as_transform,
    average_poses,
    lifted_target,
    make_camera_standoff_target,
    pose_reached,
    poses_stable,
    rotation_error_deg,
    translation_error_m,
    workspace_contains,
)
from piper_kinematics import GraspMotionPlan, PiperKinematics
from motion_settle import MotionSettler
from calibration_config import load_grasp_configuration
from vendor.piper_ros2_control.gripper_execution import (
    close_fallback_due,
    GripperExecution,
    GripperExecutionState,
    GripperFeedback,
    OpeningResponseMonitor,
)
from vendor.piper_ros2_control.gripper_startup import (
    GripperStartupFeedback,
    GripperStartupPreflight,
    GripperStartupState,
)


def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    """Convert a ROS pose to a homogeneous transform."""
    position = msg.pose.position
    quaternion = msg.pose.orientation
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    ).as_matrix()
    transform[:3, 3] = [position.x, position.y, position.z]
    return as_transform(transform)


def matrix_to_pose(transform: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    """Convert a homogeneous transform to a ROS pose."""
    transform = as_transform(transform)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(
        float, transform[:3, 3]
    )
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ) = map(float, quaternion)
    return msg


def stamp_ns(msg: PoseStamped) -> int:
    """Return a PoseStamped timestamp in nanoseconds."""
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def gripper_healthy(msg: GripperStatus) -> bool:
    """Interpret the AGX gripper driver fault bits conservatively."""
    return (
        bool(msg.driver_enable_status)
        and not bool(msg.voltage_too_low)
        and not bool(msg.motor_overheating)
        and not bool(msg.driver_overcurrent)
        and not bool(msg.driver_overheating)
        and not bool(msg.sensor_status)
        and not bool(msg.driver_error_status)
    )


class KnownObjectGrasp(Node):
    """Bounded one-shot state machine; successful completion holds the object."""

    GRIPPER_PREFLIGHT = "gripper_preflight"
    GRIPPER_NORMALIZING = "gripper_normalizing"
    WAIT_STABLE = "wait_stable"
    PLAN_READY = "plan_ready"
    OPENING = "opening"
    PREGRASPING = "pregrasping"
    APPROACHING = "approaching"
    CLOSING = "closing"
    LIFTING = "lifting"
    HOLDING = "holding"
    PREFLIGHT_ONLY = "preflight_only"
    DRY_RUN = "dry_run"
    ABORTED = "aborted"

    def __init__(
        self,
        args: argparse.Namespace,
        config: dict,
        tcp_T_camera: np.ndarray,
    ) -> None:
        super().__init__("piper_known_object_grasp")
        self.args = args
        self.config = config
        self.object_cfg = dict(config["object"])
        self.motion_cfg = dict(config["motion"])
        self.gripper_cfg = dict(config["gripper"])
        self.allow_unverified_close_fallback = bool(
            self.gripper_cfg.get("allow_unverified_close_fallback", False)
            or bool(getattr(args, "allow_unverified_close_fallback", False))
        )
        self.gripper_calibration = dict(config["_gripper_calibration"])
        self.safety_cfg = dict(config["safety"])
        self.robot_cfg = dict(config["robot"])
        self.planning_cfg = dict(config["planning"])
        self.topics = dict(config["topics"])
        self.base_frame = str(config.get("base_frame", "base_link"))
        self.tcp_T_camera = as_transform(tcp_T_camera)

        self.object_history: list[np.ndarray] = []
        self.tcp_history: list[np.ndarray] = []
        self.latest_tcp: PoseStamped | None = None
        self.latest_joints: JointState | None = None
        self.latest_arm_status: AgxArmStatus | None = None
        self.latest_gripper: GripperStatus | None = None
        self.latest_model_id = ""
        self.latest_model_stamp_ns = 0
        self.recovery_required = False
        self.last_tcp_rx = 0.0
        self.last_joint_rx = 0.0
        self.last_arm_rx = 0.0
        self.last_gripper_rx = 0.0
        self.last_object_rx = 0.0
        self.last_wait_log = 0.0

        self.frozen_object: np.ndarray | None = None
        self.pregrasp_target: np.ndarray | None = None
        self.grasp_target: np.ndarray | None = None
        self.lift_target: np.ndarray | None = None
        self.motion_plan: GraspMotionPlan | None = None
        self.state = self.GRIPPER_PREFLIGHT
        self.state_since = time.monotonic()
        self.close_contact_since: float | None = None
        self.close_contact_width: float | None = None
        self.close_contact_reached = False
        self.close_force_since: float | None = None
        self.close_start_width: float | None = None
        self.close_completion_reason = ""
        self.open_limit_since: float | None = None
        self.open_limit_width: float | None = None
        self.open_limit_reached = False
        self.opening_response: OpeningResponseMonitor | None = None
        self.startup_command_active = False
        self.startup_ready = False
        self.motion_start_tcp: np.ndarray | None = None
        self.motion_republished = False

        urdf_root = Path(
            get_package_share_directory(str(self.robot_cfg["urdf_package"]))
        )
        urdf_path = urdf_root / str(self.robot_cfg["urdf_relative_path"])
        self.kinematics = PiperKinematics.from_urdf(
            urdf_path, list(self.robot_cfg["joint_names"])
        )
        self.motion_settler = MotionSettler(
            required_samples=int(self.motion_cfg.get("settle_required_samples", 6)),
            translation_limit_m=float(
                self.motion_cfg.get("settle_translation_m", 0.0015)
            ),
            rotation_limit_deg=float(
                self.motion_cfg.get("settle_rotation_deg", 0.5)
            ),
            dwell_s=float(self.motion_cfg.get("settle_dwell_s", 0.15)),
        )

        command_limits = self.gripper_calibration["command_limits_m"]
        feedback_limits = self.gripper_calibration["feedback_limits_m"]
        startup_cfg = self.gripper_calibration["startup"]
        self.opening_response_cfg = dict(
            self.gripper_calibration["opening_response"]
        )
        homing_cfg = self.gripper_calibration["homing"]
        transport_cfg = dict(
            self.gripper_calibration.get("command_transport", {})
        )
        self.gripper = GripperExecution(
            feedback_timeout_sec=float(self.gripper_cfg["feedback_timeout_s"]),
            command_timeout_sec=float(self.gripper_cfg["command_timeout_s"]),
            opening_limit=(
                float(command_limits["min"]),
                float(command_limits["max"]),
            ),
            feedback_opening_limit=(
                float(feedback_limits["min"]),
                float(feedback_limits["max"]),
            ),
            force_limit=(0.0, 5.0),
            target_tolerance_m=float(self.gripper_cfg["target_tolerance_m"]),
            max_speed_mps=float(self.gripper_cfg["speed_mps"]),
            max_acceleration_mps2=float(self.gripper_cfg["acceleration_mps2"]),
            command_retry_interval_sec=float(
                transport_cfg.get("retry_interval_s", 0.50)
            ),
            max_command_attempts=int(transport_cfg.get("max_attempts", 4)),
        )
        self.gripper_startup = GripperStartupPreflight(
            feedback_limit_m=(
                float(feedback_limits["min"]),
                float(feedback_limits["max"]),
            ),
            safe_width_m=float(startup_cfg["safe_width_m"]),
            feedback_timeout_s=float(self.gripper_cfg["feedback_timeout_s"]),
            timeout_s=float(startup_cfg["timeout_s"]),
            target_tolerance_m=float(startup_cfg["target_tolerance_m"]),
            minimum_response_m=float(startup_cfg["minimum_response_m"]),
            direction_tolerance_m=float(startup_cfg["direction_tolerance_m"]),
            settle_dwell_s=float(startup_cfg["settle_dwell_s"]),
            homing_required=bool(homing_cfg["required"]),
        )
        configured_open = float(self.gripper_cfg["open_width_m"])
        command_min = float(command_limits["min"])
        command_max = float(command_limits["max"])
        physical_max = float(
            self.gripper_calibration["calibrated_physical_max_m"]
        )
        normal_open = float(self.gripper_calibration["normal_open_width_m"])
        if physical_max < command_min or physical_max > command_max:
            raise ValueError(
                "calibrated_physical_max_m must be inside the driver command range"
            )
        if normal_open < command_min or normal_open > physical_max:
            raise ValueError(
                "calibrated normal_open_width_m must be inside the physical range"
            )
        if configured_open < command_min or configured_open > normal_open:
            raise ValueError(
                f"configured open_width_m={configured_open:.4f}m is outside "
                f"the calibrated daily-open range [{command_min:.4f}, "
                f"{normal_open:.4f}]m"
            )
        plateau_minimum = float(self.gripper_cfg["open_limit_min_width_m"])
        if plateau_minimum < command_min or plateau_minimum > physical_max:
            raise ValueError(
                "open_limit_min_width_m must be inside the calibrated physical range"
            )

        self.move_p_pub = self.create_publisher(
            PoseStamped, self.topics["move_p"], 1
        )
        self.move_l_pub = self.create_publisher(
            PoseStamped, self.topics["move_l"], 1
        )
        self.move_j_pub = self.create_publisher(
            JointState, self.topics["move_j"], 1
        )
        self.joint_command_pub = self.create_publisher(
            JointState, self.topics["joint_command"], 1
        )
        result_qos = QoSProfile(depth=1)
        result_qos.reliability = ReliabilityPolicy.RELIABLE
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.frozen_object_pub = self.create_publisher(
            PoseStamped, self.topics["frozen_object"], result_qos
        )
        self.grasp_target_pub = self.create_publisher(
            PoseStamped, self.topics["grasp_target"], result_qos
        )
        self.pregrasp_joint_pub = self.create_publisher(
            JointState, self.topics["pregrasp_joint_target"], result_qos
        )
        self.status_pub = self.create_publisher(
            String, self.topics["status"], result_qos
        )

        self.create_subscription(
            PoseStamped, self.topics["tcp_feedback"], self._tcp_callback, 20
        )
        self.create_subscription(
            JointState, self.topics["joint_feedback"], self._joint_callback, 20
        )
        self.create_subscription(
            PoseStamped, self.topics["object_pose_base"], self._object_callback, 20
        )
        self.create_subscription(
            AgxArmStatus, self.topics["arm_status"], self._arm_callback, 20
        )
        self.create_subscription(
            GripperStatus, self.topics["gripper_status"], self._gripper_callback, 20
        )
        self.create_subscription(
            String, self.topics["object_pose_json"], self._model_callback, 10
        )
        self.create_subscription(
            Bool, self.topics["recovery"], self._recovery_callback, 10
        )
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"known object={self.object_cfg['id']}, "
            f"expected model={self.object_cfg['foundationpose_model_id']}, "
            f"standoff={float(self.motion_cfg['camera_standoff_m']):.3f}m, "
            f"lift={float(self.motion_cfg['lift_m']):.3f}m, execute={args.execute}"
        )
        if not bool(homing_cfg["required"]):
            self.get_logger().warning(
                "calibrated v189 profile treats homing_status as advisory: "
                "zero and 0-100 mm range were verified across a power cycle"
            )
        if args.execute:
            self.get_logger().warning(
                "EXECUTION ARMED: this run may command the arm and gripper. "
                "The node still never enables Piper or opens /control_enable."
            )
        else:
            self.get_logger().warning(
                "DRY RUN: visualization targets only; no arm or gripper "
                "commands will be published."
            )
        self._publish_status("started")

    def _publish_status(self, detail: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "object_id": str(self.object_cfg["id"]),
                "model_id": self.latest_model_id,
                "state": self.state,
                "execute": bool(self.args.execute),
                "detail": detail,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

    def _set_state(self, state: str, detail: str) -> None:
        self.state = state
        self.state_since = time.monotonic()
        if state == self.OPENING:
            self.open_limit_since = None
            self.open_limit_width = None
            self.open_limit_reached = False
            start_width = (
                float(self.latest_gripper.width)
                if self.latest_gripper is not None
                else float(self.gripper_calibration["startup"]["safe_width_m"])
            )
            self.opening_response = OpeningResponseMonitor(
                start_width_m=start_width,
                target_width_m=float(self.gripper_cfg["open_width_m"]),
                minimum_travel_m=float(
                    self.opening_response_cfg["minimum_travel_m"]
                ),
                minimum_step_m=float(
                    self.opening_response_cfg["minimum_step_m"]
                ),
                required_samples=int(
                    self.opening_response_cfg["required_samples"]
                ),
                direction_tolerance_m=float(
                    self.opening_response_cfg["direction_tolerance_m"]
                ),
                target_tolerance_m=float(self.gripper_cfg["target_tolerance_m"]),
            )
        elif state not in (self.WAIT_STABLE,):
            self.open_limit_since = None
            self.open_limit_width = None
            if state != self.GRIPPER_NORMALIZING:
                self.opening_response = None
        if state != self.CLOSING:
            self.close_contact_since = None
            self.close_contact_width = None
            self.close_force_since = None
        if state == self.CLOSING:
            self.close_contact_reached = False
            self.close_completion_reason = ""
            self.close_start_width = (
                float(self.latest_gripper.width)
                if self.latest_gripper is not None
                else None
            )
        self._publish_status(detail)

    def _abort(self, reason: str) -> None:
        if self.state == self.ABORTED:
            return
        self.get_logger().error(f"GRASP ABORTED: {reason}")
        self._set_state(self.ABORTED, reason)

    def _tcp_callback(self, msg: PoseStamped) -> None:
        try:
            transform = pose_to_matrix(msg)
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid TCP pose: {exc}")
            return
        self.latest_tcp = msg
        self.last_tcp_rx = time.monotonic()
        self.tcp_history.append(transform)
        sample_count = int(self.motion_cfg["stationary_tcp_frames"])
        self.tcp_history = self.tcp_history[-sample_count:]

    def _object_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warning(
                f"ignored object pose in {msg.header.frame_id!r}; "
                f"expected {self.base_frame!r}"
            )
            return
        if self.recovery_required or self.state != self.WAIT_STABLE:
            return
        try:
            transform = pose_to_matrix(msg)
        except ValueError as exc:
            self.get_logger().warning(f"ignored invalid object pose: {exc}")
            return
        self.last_object_rx = time.monotonic()
        self.object_history.append(transform)
        sample_count = int(self.motion_cfg["stable_frames"])
        self.object_history = self.object_history[-sample_count:]

    def _joint_callback(self, msg: JointState) -> None:
        expected = set(self.kinematics.names)
        if not expected.issubset(set(msg.name)):
            return
        try:
            values = [float(msg.position[msg.name.index(name)]) for name in self.kinematics.names]
        except (IndexError, TypeError, ValueError):
            return
        if not np.isfinite(values).all():
            return
        self.latest_joints = msg
        self.last_joint_rx = time.monotonic()

    def _arm_callback(self, msg: AgxArmStatus) -> None:
        self.latest_arm_status = msg
        self.last_arm_rx = time.monotonic()

    def _gripper_callback(self, msg: GripperStatus) -> None:
        now = time.monotonic()
        self.latest_gripper = msg
        self.last_gripper_rx = now
        intent = self.gripper.observe(
            GripperFeedback(
                opening_m=float(msg.width),
                force_n=float(msg.force),
                healthy=gripper_healthy(msg),
                stamp=now,
            )
        )
        startup_decision = self.gripper_startup.observe(
            GripperStartupFeedback(
                opening_m=float(msg.width),
                force_n=float(msg.force),
                healthy=gripper_healthy(msg),
                homed=bool(msg.homing_status),
                stamp=now,
            ),
            now,
        )
        if (
            startup_decision.error
            and self.state == self.GRIPPER_NORMALIZING
        ):
            self._abort(startup_decision.error)
            return
        if startup_decision.action == "ready":
            self.startup_ready = True
        if intent.error and self.state in (self.OPENING, self.CLOSING):
            self._abort(intent.error)
            return
        if self.state == self.OPENING:
            width = float(msg.width)
            if self.opening_response is not None:
                response = self.opening_response.observe(width)
                if response.error:
                    self._abort(response.error)
                    return
            minimum = float(self.gripper_cfg["open_limit_min_width_m"])
            tolerance = float(
                self.gripper_cfg["open_limit_width_tolerance_m"]
            )
            response_seen = (
                self.opening_response is None
                or self.opening_response.response_seen
            )
            if (
                response_seen
                and gripper_healthy(msg)
                and math.isfinite(width)
                and width >= minimum
            ):
                if self.open_limit_since is None:
                    self.open_limit_since = now
                    self.open_limit_width = width
                elif (
                    self.open_limit_width is None
                    or abs(width - self.open_limit_width) > tolerance
                ):
                    self.open_limit_since = now
                    self.open_limit_width = width
                elif now - self.open_limit_since >= float(
                    self.gripper_cfg["open_limit_dwell_s"]
                ):
                    self.open_limit_reached = True
            else:
                self.open_limit_since = None
                self.open_limit_width = None
            return
        if self.state != self.CLOSING:
            return
        width = float(msg.width)
        force = abs(float(msg.force))
        requested = float(self.gripper_cfg["close_width_m"])
        width_tolerance = float(self.gripper_cfg["contact_width_tolerance_m"])
        contact_force = float(self.gripper_cfg["contact_force"])
        healthy_sample = (
            gripper_healthy(msg)
            and math.isfinite(width)
            and math.isfinite(force)
        )

        # A sustained force threshold is one independent completion route.
        if healthy_sample and force >= contact_force:
            if self.close_force_since is None:
                self.close_force_since = now
            elif now - self.close_force_since >= float(
                self.gripper_cfg["contact_dwell_s"]
            ):
                self.close_contact_reached = True
                self.close_completion_reason = "force threshold"
        else:
            self.close_force_since = None

        # A second route detects a real closing motion followed by a stable
        # width above the requested target (the object physically blocked it).
        close_travel = (
            self.close_start_width - width
            if self.close_start_width is not None
            else 0.0
        )
        if (
            healthy_sample
            and close_travel >= float(self.gripper_cfg["minimum_close_travel_m"])
            and width > requested + self.gripper.target_tolerance_m
        ):
            if self.close_contact_since is None:
                self.close_contact_since = now
                self.close_contact_width = width
            elif (
                self.close_contact_width is not None
                and abs(width - self.close_contact_width) <= width_tolerance
                and now - self.close_contact_since
                >= float(self.gripper_cfg["contact_dwell_s"])
            ):
                self.close_contact_reached = True
                if not self.close_completion_reason:
                    self.close_completion_reason = "stable width plateau"
        else:
            self.close_contact_since = None
            self.close_contact_width = None

    def _model_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.latest_model_id = str(payload.get("model_id", ""))
            self.latest_model_stamp_ns = (
                int(payload.get("stamp_sec", 0)) * 1_000_000_000
                + int(payload.get("stamp_nanosec", 0))
            )
        except (json.JSONDecodeError, TypeError, ValueError):
            self.get_logger().warning("ignored malformed FoundationPose JSON")

    def _recovery_callback(self, msg: Bool) -> None:
        was_required = self.recovery_required
        self.recovery_required = bool(msg.data)
        if (
            self.state == self.WAIT_STABLE
            and self.recovery_required
            and not was_required
        ):
            self.object_history.clear()
        if self.recovery_required and not was_required:
            self.get_logger().warning(
                "FoundationPose recovery is active; target acquisition paused"
            )

    def _feedback_ready(self, now: float) -> tuple[bool, str]:
        timeout = float(self.safety_cfg["feedback_timeout_s"])
        # The gripper executor performs its own freshness check when arm() is
        # called.  Use the stricter of the two limits here so this gate cannot
        # pass a sample that the executor rejects a few microseconds later.
        gripper_timeout = min(
            timeout, float(self.gripper_cfg["feedback_timeout_s"])
        )
        checks = (
            (self.latest_tcp, self.last_tcp_rx, "TCP feedback", timeout),
            (self.latest_joints, self.last_joint_rx, "joint feedback", timeout),
            (self.latest_arm_status, self.last_arm_rx, "arm status", timeout),
            (
                self.latest_gripper,
                self.last_gripper_rx,
                "gripper feedback",
                gripper_timeout,
            ),
        )
        for value, received, name, freshness_limit in checks:
            if value is None:
                return False, f"waiting for {name}"
            age = now - received
            if age > freshness_limit:
                return False, (
                    f"{name} is stale ({age:.3f}s > "
                    f"{freshness_limit:.3f}s)"
                )
        if int(self.latest_arm_status.err_status) != 0:
            return False, f"arm error status={int(self.latest_arm_status.err_status)}"
        if int(self.latest_arm_status.arm_status) != 0:
            return False, self._arm_status_detail("arm status")
        if int(self.latest_arm_status.ctrl_mode) not in (1, 2):
            return False, (
                f"arm ctrl_mode={int(self.latest_arm_status.ctrl_mode)}, "
                "expected Piper v189 seamless mode 1 or 2"
            )
        if int(self.latest_arm_status.teach_status) not in (0, 2):
            return False, (
                f"teach_status={int(self.latest_arm_status.teach_status)}; "
                "exit drag teaching before grasping (expected 0 or 2)"
            )
        if any(bool(value) for value in self.latest_arm_status.joint_angle_limit):
            return False, self._arm_status_detail("joint angle limit")
        if any(
            bool(value)
            for value in self.latest_arm_status.communication_status_joint
        ):
            return False, "one or more joints report a communication fault"
        if not gripper_healthy(self.latest_gripper):
            return False, "gripper reports disabled or a driver fault"
        return True, "ready"

    def _feedback_refreshed_after(self, threshold: float) -> tuple[bool, str]:
        """Require every safety-critical stream to advance after a long plan."""
        checks = (
            (self.last_tcp_rx, "TCP feedback"),
            (self.last_joint_rx, "joint feedback"),
            (self.last_arm_rx, "arm status"),
            (self.last_gripper_rx, "gripper feedback"),
        )
        for received, name in checks:
            if received <= threshold:
                return False, f"waiting for a new {name} sample after IK preflight"
        return True, "ready"

    def _current_joint_vector(self) -> np.ndarray:
        if self.latest_joints is None:
            raise ValueError("joint feedback is unavailable")
        positions = {
            name: float(self.latest_joints.position[index])
            for index, name in enumerate(self.latest_joints.name)
            if index < len(self.latest_joints.position)
        }
        missing = [name for name in self.kinematics.names if name not in positions]
        if missing:
            raise ValueError(f"joint feedback is missing {','.join(missing)}")
        values = np.asarray([positions[name] for name in self.kinematics.names])
        if not np.isfinite(values).all():
            raise ValueError("joint feedback contains non-finite values")
        return values

    def _arm_status_detail(self, prefix: str) -> str:
        """Describe controller status and identify per-joint fault flags."""
        if self.latest_arm_status is None:
            return f"{prefix}: no arm status available"
        limit_joints = [
            f"J{index}"
            for index, value in enumerate(
                self.latest_arm_status.joint_angle_limit, start=1
            )
            if bool(value)
        ]
        communication_joints = [
            f"J{index}"
            for index, value in enumerate(
                self.latest_arm_status.communication_status_joint, start=1
            )
            if bool(value)
        ]
        details = [
            f"arm_status={int(self.latest_arm_status.arm_status)}",
            f"err_status={int(self.latest_arm_status.err_status)}",
        ]
        if limit_joints:
            details.append(f"angle_limit={','.join(limit_joints)}")
        if communication_joints:
            details.append(f"communication_fault={','.join(communication_joints)}")
        return f"{prefix}: " + "; ".join(details)

    def _model_ready(self) -> tuple[bool, str]:
        expected = str(self.object_cfg["foundationpose_model_id"])
        if not self.latest_model_id:
            return False, "waiting for FoundationPose model identity"
        if self.latest_model_id != expected:
            return False, f"model mismatch: got {self.latest_model_id!r}, expected {expected!r}"
        return True, "ready"

    def _vision_fresh(self) -> tuple[bool, str]:
        if self.recovery_required:
            return False, "FoundationPose recovery is active"
        if len(self.object_history) < int(self.motion_cfg["stable_frames"]):
            return False, "collecting stable object poses"
        if time.monotonic() - self.last_object_rx > float(
            self.safety_cfg["object_feedback_timeout_s"]
        ):
            return False, "base-frame object pose is stale"
        if self.latest_model_stamp_ns <= 0:
            return False, "FoundationPose model timestamp is missing"
        age_s = (
            self.get_clock().now().nanoseconds - self.latest_model_stamp_ns
        ) * 1e-9
        if age_s < -0.1 or age_s > float(self.safety_cfg["max_image_age_s"]):
            return False, f"FoundationPose image age is {age_s:.3f}s"
        return True, "ready"

    def _stable_target_available(self) -> tuple[bool, str]:
        model_ok, model_reason = self._model_ready()
        if not model_ok:
            return False, model_reason
        vision_ok, vision_reason = self._vision_fresh()
        if not vision_ok:
            return False, vision_reason
        if not poses_stable(
            self.object_history,
            translation_limit_m=float(self.motion_cfg["stable_translation_m"]),
            rotation_limit_deg=float(self.motion_cfg["stable_rotation_deg"]),
        ):
            return False, "object pose window is not stable"
        if len(self.tcp_history) < int(self.motion_cfg["stationary_tcp_frames"]):
            return False, "collecting stationary TCP feedback"
        if not poses_stable(
            self.tcp_history,
            translation_limit_m=float(self.motion_cfg["stationary_tcp_translation_m"]),
            rotation_limit_deg=float(self.motion_cfg["stationary_tcp_rotation_deg"]),
        ):
            return False, "robot must remain stationary while freezing the target"
        return True, "ready"

    def _freeze_targets(self) -> None:
        self.frozen_object = average_poses(self.object_history)
        current_tcp = pose_to_matrix(self.latest_tcp)
        self.pregrasp_target = make_camera_standoff_target(
            current_tcp,
            self.frozen_object,
            self.tcp_T_camera,
            float(self.motion_cfg["pregrasp_standoff_m"]),
        )
        self.grasp_target = make_camera_standoff_target(
            current_tcp,
            self.frozen_object,
            self.tcp_T_camera,
            float(self.motion_cfg["camera_standoff_m"]),
        )
        self.lift_target = lifted_target(
            self.grasp_target, float(self.motion_cfg["lift_m"])
        )
        bounds = dict(self.safety_cfg["workspace_m"])
        if not workspace_contains(self.pregrasp_target, bounds):
            raise ValueError(
                "pre-grasp target is outside workspace: "
                f"{self.pregrasp_target[:3, 3].tolist()}"
            )
        if not workspace_contains(self.grasp_target, bounds):
            raise ValueError(
                f"grasp target is outside workspace: {self.grasp_target[:3, 3].tolist()}"
            )
        if not workspace_contains(self.lift_target, bounds):
            raise ValueError(
                f"lift target is outside workspace: {self.lift_target[:3, 3].tolist()}"
            )
        initial_move = translation_error_m(current_tcp, self.pregrasp_target)
        if initial_move > float(self.safety_cfg["max_initial_move_m"]):
            raise ValueError(
                f"initial move {initial_move:.3f}m exceeds configured limit"
            )

        self.motion_plan = self.kinematics.plan_grasp_motion(
            self._current_joint_vector(),
            self.pregrasp_target,
            self.grasp_target,
            self.lift_target,
            self.planning_cfg,
            bounds,
        )

        stamp = self.get_clock().now().to_msg()
        self.frozen_object_pub.publish(
            matrix_to_pose(self.frozen_object, self.base_frame, stamp)
        )
        self.grasp_target_pub.publish(
            matrix_to_pose(self.grasp_target, self.base_frame, stamp)
        )
        self.pregrasp_joint_pub.publish(
            self._joint_command_message(self.motion_plan.pregrasp_joints)
        )
        self.get_logger().info(
            "target frozen: object base xyz="
            f"{np.round(self.frozen_object[:3, 3], 4).tolist()}, "
            f"pregrasp tcp xyz={np.round(self.pregrasp_target[:3, 3], 4).tolist()}, "
            f"grasp tcp xyz={np.round(self.grasp_target[:3, 3], 4).tolist()}, "
            f"lift tcp xyz={np.round(self.lift_target[:3, 3], 4).tolist()}"
        )
        self.get_logger().info(
            "IK preflight passed: pregrasp joints deg="
            f"{np.round(np.degrees(self.motion_plan.pregrasp_joints), 2).tolist()}, "
            f"minimum path margin={self.motion_plan.minimum_margin_deg:.1f}deg, "
            f"MoveJ minimum TCP z={self.motion_plan.minimum_tcp_z_m:.3f}m"
        )

    def _publish_gripper(self, opening_m: float, effort: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(opening_m)]
        msg.effort = [float(effort)]
        self.joint_command_pub.publish(msg)
        self.get_logger().info(
            "published absolute gripper target: "
            f"width={float(opening_m):.4f}m force={float(effort):.3f}N"
        )

    def _begin_gripper(self, opening_m: float) -> bool:
        now = time.monotonic()
        armed = self.gripper.arm(now)
        if armed.error:
            self._abort(f"gripper safety arm rejected: {armed.error}")
            return False
        intent = self.gripper.accept(
            float(opening_m), float(self.gripper_cfg["effort"]), now
        )
        if intent.error:
            self._abort(f"gripper target rejected: {intent.error}")
            return False
        self._publish_gripper(intent.opening_m, intent.force_n)
        return True

    def _tick_gripper(self) -> bool:
        intent = self.gripper.tick(time.monotonic())
        if intent.error:
            self._abort(intent.error)
            return False
        if intent.action == "send_target":
            self._publish_gripper(intent.opening_m, intent.force_n)
        return True

    def _publish_motion(self, publisher, target: np.ndarray) -> None:
        publisher.publish(
            matrix_to_pose(target, self.base_frame, self.get_clock().now().to_msg())
        )

    def _joint_command_message(self, joints: np.ndarray) -> JointState:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.kinematics.names)
        message.position = [float(value) for value in joints]
        return message

    def _publish_joint_motion(self, joints: np.ndarray) -> None:
        self.move_j_pub.publish(self._joint_command_message(joints))

    def _start_motion(self, publisher, target: np.ndarray, state: str, detail: str) -> bool:
        if publisher.get_subscription_count() < 1:
            self._abort(f"no driver subscriber for {publisher.topic_name}")
            return False
        if self.latest_tcp is None:
            self._abort("TCP feedback unavailable before motion")
            return False
        self.motion_start_tcp = pose_to_matrix(self.latest_tcp)
        self.motion_republished = False
        self.motion_settler.reset()
        self._publish_motion(publisher, target)
        self._set_state(state, detail)
        return True

    def _start_joint_motion(
        self, joints: np.ndarray, target: np.ndarray, state: str, detail: str
    ) -> bool:
        if self.move_j_pub.get_subscription_count() < 1:
            self._abort(f"no driver subscriber for {self.move_j_pub.topic_name}")
            return False
        if self.latest_tcp is None:
            self._abort("TCP feedback unavailable before joint motion")
            return False
        self.motion_start_tcp = pose_to_matrix(self.latest_tcp)
        self.motion_republished = False
        self.motion_settler.reset()
        self._publish_joint_motion(joints)
        self._set_state(state, detail)
        return True

    def _motion_reached(
        self, target: np.ndarray, publisher, joint_target: np.ndarray | None = None
    ) -> bool:
        elapsed = time.monotonic() - self.state_since
        if elapsed > float(self.motion_cfg["timeout_s"]):
            self._abort(f"motion timeout in state {self.state}")
            return False
        if self.latest_tcp is None or (
            time.monotonic() - self.last_tcp_rx
            > float(self.safety_cfg["feedback_timeout_s"])
        ):
            self._abort("TCP feedback became stale during motion")
            return False
        if self.latest_arm_status is not None and int(self.latest_arm_status.err_status) != 0:
            self._abort(
                f"arm error during motion: {int(self.latest_arm_status.err_status)}"
            )
            return False
        if self.latest_arm_status is not None and int(self.latest_arm_status.arm_status) != 0:
            self._abort(self._arm_status_detail("abnormal arm status during motion"))
            return False
        if self.latest_arm_status is None or (
            time.monotonic() - self.last_arm_rx
            > float(self.safety_cfg["feedback_timeout_s"])
        ):
            self._abort("arm status feedback became stale during motion")
            return False
        if (
            self.latest_arm_status is not None
            and int(self.latest_arm_status.teach_status) not in (0, 2)
        ):
            self._abort(
                "drag teaching became active during motion: "
                f"teach_status={int(self.latest_arm_status.teach_status)}"
            )
            return False
        current_tcp = pose_to_matrix(self.latest_tcp)
        reached = pose_reached(
            current_tcp,
            target,
            translation_tolerance_m=float(self.motion_cfg["position_tolerance_m"]),
            rotation_tolerance_deg=float(self.motion_cfg["rotation_tolerance_deg"]),
        )
        if reached and joint_target is not None:
            joint_error_deg = float(
                np.max(
                    np.abs(
                        np.degrees(self._current_joint_vector() - joint_target)
                    )
                )
            )
            reached = joint_error_deg <= float(
                self.planning_cfg["joint_position_tolerance_deg"]
            )
        if reached and elapsed >= float(self.motion_cfg["minimum_motion_s"]):
            if int(self.latest_arm_status.motion_status) != 0:
                self.motion_settler.reset()
                return False
            if self.motion_settler.observe(
                current_tcp,
                motion_status=int(self.latest_arm_status.motion_status),
                now=time.monotonic(),
            ):
                return True
        else:
            self.motion_settler.reset()

        started = self.motion_start_tcp is not None and (
            translation_error_m(current_tcp, self.motion_start_tcp)
            >= float(self.motion_cfg["start_translation_m"])
            or rotation_error_deg(current_tcp, self.motion_start_tcp)
            >= float(self.motion_cfg["start_rotation_deg"])
        )
        if (
            not started
            and not self.motion_republished
            and elapsed >= float(self.motion_cfg["republish_after_s"])
        ):
            if joint_target is None:
                self._publish_motion(publisher, target)
            else:
                self._publish_joint_motion(joint_target)
            self.motion_republished = True
            self.get_logger().warning(
                f"no TCP motion yet; republished {publisher.topic_name} once"
            )
        if not started and elapsed > float(self.motion_cfg["start_timeout_s"]):
            self._abort(
                "TCP did not start moving; the driver rejected or did not "
                "receive the target (check enable state and target reachability)"
            )
        return False

    def _start_lift(self, reason: str) -> None:
        if self._start_motion(
            self.move_l_pub, self.lift_target, self.LIFTING, reason
        ):
            self.get_logger().info(
                f"linear lift command sent: +{float(self.motion_cfg['lift_m']):.3f}m base Z"
            )

    def _tick(self) -> None:
        if self.state in (
            self.HOLDING,
            self.PREFLIGHT_ONLY,
            self.DRY_RUN,
            self.ABORTED,
        ):
            return
        now = time.monotonic()

        if self.state == self.GRIPPER_PREFLIGHT:
            feedback_ok, feedback_reason = self._feedback_ready(now)
            if not feedback_ok:
                if now - self.last_wait_log > 2.0:
                    self.get_logger().info(
                        f"waiting for gripper startup preflight: {feedback_reason}"
                    )
                    self.last_wait_log = now
                return
            decision = self.gripper_startup.begin(now)
            if decision.error:
                self._abort(f"gripper startup preflight failed: {decision.error}")
                return
            if not self.args.execute:
                self.get_logger().info(
                    "gripper startup feedback validated; dry-run will not "
                    "publish a normalization command"
                )
                self._set_state(self.WAIT_STABLE, "gripper preflight validated")
                return
            self.startup_command_active = decision.action == "normalize"
            if self.startup_command_active:
                if not self._begin_gripper(float(decision.target_m)):
                    return
                self.get_logger().info(
                    "gripper startup normalization command started: "
                    f"target={float(decision.target_m):.3f}m"
                )
            else:
                self.get_logger().info(
                    "gripper already lies within the calibrated startup "
                    "target window; verifying stable feedback"
                )
            self._set_state(
                self.GRIPPER_NORMALIZING,
                "verifying calibrated startup width and response",
            )
            return

        if self.state == self.GRIPPER_NORMALIZING:
            decision = self.gripper_startup.tick(now)
            if decision.error:
                self._abort(decision.error)
                return
            if (
                self.startup_command_active
                and self.gripper.state is GripperExecutionState.MOVING
                and not self._tick_gripper()
            ):
                return
            if self.startup_ready or (
                self.gripper_startup.state is GripperStartupState.READY
            ):
                if (
                    self.startup_command_active
                    and self.gripper.state is not GripperExecutionState.ARMED
                ):
                    return
                self.get_logger().info(
                    "gripper startup preflight passed: fresh calibrated "
                    "feedback, correct response direction, stable safe width"
                )
                if bool(getattr(self.args, "gripper_preflight_only", False)):
                    self._set_state(
                        self.PREFLIGHT_ONLY,
                        "gripper startup preflight completed; no arm motion permitted",
                    )
                    self.get_logger().info(
                        "GRIPPER PREFLIGHT ONLY COMPLETE: no arm motion "
                        "commands were published"
                    )
                else:
                    self._set_state(self.WAIT_STABLE, "gripper startup ready")
            return

        if self.state == self.WAIT_STABLE:
            feedback_ok, feedback_reason = self._feedback_ready(now)
            target_ok, target_reason = self._stable_target_available()
            if not feedback_ok or not target_ok:
                if now - self.last_wait_log > 2.0:
                    reason = feedback_reason if not feedback_ok else target_reason
                    self.get_logger().info(f"waiting: {reason}")
                    self.last_wait_log = now
                return
            try:
                self._freeze_targets()
            except (ValueError, np.linalg.LinAlgError) as exc:
                self._abort(f"unsafe or invalid target: {exc}")
                return
            if not self.args.execute:
                self.get_logger().info(
                    "DRY RUN COMPLETE: frozen grasp/lift targets published; no command sent"
                )
                self._set_state(self.DRY_RUN, "targets validated; no command sent")
                return
            # IK preflight can take several seconds in this single-threaded
            # node. Enter a separate state so queued feedback is processed
            # before arming the gripper or publishing any robot command.
            self._set_state(
                self.PLAN_READY,
                "IK path validated; refreshing feedback before execution",
            )
            self.get_logger().info(
                "IK path validated; waiting for fresh arm/joint/gripper feedback"
            )
            return

        if self.state == self.PLAN_READY:
            refreshed_ok, refreshed_reason = self._feedback_refreshed_after(
                self.state_since
            )
            if not refreshed_ok:
                if now - self.state_since > float(
                    self.motion_cfg.get(
                        "post_plan_feedback_timeout_s",
                        self.motion_cfg["start_timeout_s"],
                    )
                ):
                    self._abort(
                        "feedback did not resume after IK preflight: "
                        f"{refreshed_reason}"
                    )
                    return
                if now - self.last_wait_log > 1.0:
                    self.get_logger().info(
                        f"waiting after IK preflight: {refreshed_reason}"
                    )
                    self.last_wait_log = now
                return
            feedback_ok, feedback_reason = self._feedback_ready(now)
            if not feedback_ok:
                if now - self.last_wait_log > 1.0:
                    self.get_logger().info(
                        f"waiting after IK preflight: {feedback_reason}"
                    )
                    self.last_wait_log = now
                return
            if self._begin_gripper(float(self.gripper_cfg["open_width_m"])):
                self._set_state(self.OPENING, "opening gripper")
                self.get_logger().info("gripper opening sequence started")
            return

        if self.state == self.OPENING:
            if (
                self.open_limit_reached
                and self.gripper.state is GripperExecutionState.MOVING
            ):
                limit_result = self.gripper.complete_at_physical_limit(
                    float(self.gripper_cfg["open_limit_min_width_m"]),
                    time.monotonic(),
                )
                if limit_result.error:
                    self._abort(limit_result.error)
                    return
                self.get_logger().info(
                    "opening plateau accepted after verified feedback motion: "
                    f"{float(self.latest_gripper.width):.4f}m"
                )
            if not self._tick_gripper():
                return
            if self.gripper.state is GripperExecutionState.ARMED:
                if (
                    self.opening_response is not None
                    and self.opening_response.response_required
                    and not self.opening_response.response_seen
                ):
                    self._abort(
                        "opening target was reached without a verified "
                        "directional physical feedback response"
                    )
                    return
                pregrasp_cm = 100.0 * float(
                    self.motion_cfg["pregrasp_standoff_m"]
                )
                if self.motion_plan is None:
                    self._abort("IK motion plan disappeared before execution")
                    return
                if self._start_joint_motion(
                    self.motion_plan.pregrasp_joints,
                    self.pregrasp_target,
                    self.PREGRASPING,
                    f"MoveJ to validated {pregrasp_cm:.0f} cm pre-grasp target",
                ):
                    self.get_logger().info("validated pre-grasp MoveJ command sent")
            return

        if self.state == self.PREGRASPING:
            if self._motion_reached(
                self.pregrasp_target,
                self.move_j_pub,
                self.motion_plan.pregrasp_joints,
            ):
                pregrasp_cm = 100.0 * float(
                    self.motion_cfg["pregrasp_standoff_m"]
                )
                grasp_cm = 100.0 * float(
                    self.motion_cfg["camera_standoff_m"]
                )
                if self._start_motion(
                    self.move_l_pub,
                    self.grasp_target,
                    self.APPROACHING,
                    f"linear advance from {pregrasp_cm:.0f} cm to "
                    f"{grasp_cm:.0f} cm standoff",
                ):
                    self.get_logger().info("final linear grasp approach command sent")
            return

        if self.state == self.APPROACHING:
            if self._motion_reached(self.grasp_target, self.move_l_pub):
                if self._begin_gripper(float(self.gripper_cfg["close_width_m"])):
                    self._set_state(self.CLOSING, "closing gripper")
                    self.get_logger().info("gripper close sequence started")
            return

        if self.state == self.CLOSING:
            if self.close_contact_reached and self.latest_gripper is not None:
                self._publish_gripper(
                    float(self.gripper_cfg["close_width_m"]),
                    float(self.gripper_cfg["effort"]),
                )
                self.get_logger().info(
                    f"gripper contact accepted by {self.close_completion_reason}: "
                    f"width={float(self.latest_gripper.width):.3f}m, "
                    f"force={abs(float(self.latest_gripper.force)):.3f}N"
                )
                self._start_lift(self.close_completion_reason)
                return
            fallback_due = close_fallback_due(
                enabled=self.allow_unverified_close_fallback,
                elapsed_s=time.monotonic() - self.state_since,
                fallback_after_s=float(
                    self.gripper_cfg["close_fallback_after_s"]
                ),
            )
            if fallback_due:
                self._publish_gripper(
                    float(self.gripper_cfg["close_width_m"]),
                    float(self.gripper_cfg["effort"]),
                )
                self.get_logger().warning(
                    "TEST ONLY: gripper feedback did not prove contact; "
                    "using explicitly enabled close-command fallback"
                )
                self._start_lift("TEST ONLY timed close-command fallback")
                return
            if time.monotonic() - self.state_since >= float(
                self.gripper_cfg["close_fallback_after_s"]
            ):
                self._abort(
                    "gripper contact was not verified before the close timeout; "
                    "refusing to lift without force or width evidence"
                )
                return
            if not self._tick_gripper():
                return
            if self.gripper.state is GripperExecutionState.ARMED:
                self._start_lift("requested close width reached")
            return

        if self.state == self.LIFTING:
            if self._motion_reached(self.lift_target, self.move_l_pub):
                self._set_state(self.HOLDING, "grasp and 5 cm lift completed")
                self.get_logger().info(
                    "GRASP SUCCESS: lift target reached; holding object with gripper closed"
                )


def load_configuration(path: Path) -> tuple[dict, np.ndarray]:
    """Load the object grasp profile and its relative hand-eye configuration."""
    return load_grasp_configuration(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("config") / "object_003_grasp.yaml"),
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Permit one grasp and lift sequence; default is target validation only.",
    )
    parser.add_argument(
        "--gripper-preflight-only",
        action="store_true",
        help=(
            "Run only the calibrated gripper startup check. With --execute, "
            "the gripper may normalize to its safe width; arm motion stages "
            "are never entered."
        ),
    )
    parser.add_argument(
        "--allow-unverified-close-fallback",
        action="store_true",
        help=(
            "TEST ONLY: allow the legacy timed close fallback to lift without "
            "verified gripper contact; disabled by default."
        ),
    )
    args = parser.parse_args()
    config_path = Path(args.config).expanduser().resolve()
    config, tcp_T_camera = load_configuration(config_path)
    if args.execute and not config["_calibrations_confirmed"]:
        parser.error("--execute requires calibration_confirmed: true in both local calibration files")
    if abs(float(config["motion"]["lift_m"]) - 0.05) > 1e-9:
        parser.error("this first object_003 trial requires motion.lift_m = 0.05")
    rclpy.init()
    node = KnownObjectGrasp(args, config, tcp_T_camera)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
