#!/usr/bin/env python3
"""ROS integration smoke test that must never publish hardware commands."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from agx_arm_msgs.msg import AgxArmStatus, GripperStatus


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from grasp_controller import (  # noqa: E402
    KnownObjectGrasp,
    load_configuration,
    matrix_to_pose,
)


def make_pose(node: Node, xyz: tuple[float, float, float]) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
    msg.pose.orientation.w = 1.0
    return msg


def main() -> None:
    config, tcp_T_camera = load_configuration(
        ROOT / "config" / "object_003_grasp.yaml"
    )
    rclpy.init()
    controller = KnownObjectGrasp(
        argparse.Namespace(execute=False), config, tcp_T_camera
    )
    stimulus = Node("piper_control_dry_run_stimulus")

    tcp_pub = stimulus.create_publisher(
        PoseStamped, config["topics"]["tcp_feedback"], 10
    )
    joint_pub = stimulus.create_publisher(
        JointState, config["topics"]["joint_feedback"], 10
    )
    object_pub = stimulus.create_publisher(
        PoseStamped, config["topics"]["object_pose_base"], 10
    )
    arm_pub = stimulus.create_publisher(
        AgxArmStatus, config["topics"]["arm_status"], 10
    )
    gripper_pub = stimulus.create_publisher(
        GripperStatus, config["topics"]["gripper_status"], 10
    )
    model_pub = stimulus.create_publisher(
        String, config["topics"]["object_pose_json"], 10
    )
    recovery_pub = stimulus.create_publisher(
        Bool, config["topics"]["recovery"], 10
    )

    statuses: list[str] = []
    controls: list[str] = []
    stimulus.create_subscription(
        String,
        config["topics"]["status"],
        lambda msg: statuses.append(msg.data),
        10,
    )
    stimulus.create_subscription(
        PoseStamped,
        config["topics"]["move_p"],
        lambda _msg: controls.append("move_p"),
        10,
    )
    stimulus.create_subscription(
        PoseStamped,
        config["topics"]["move_l"],
        lambda _msg: controls.append("move_l"),
        10,
    )
    stimulus.create_subscription(
        JointState,
        config["topics"]["move_j"],
        lambda _msg: controls.append("move_j"),
        10,
    )
    stimulus.create_subscription(
        JointState,
        config["topics"]["joint_command"],
        lambda _msg: controls.append("gripper"),
        10,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(stimulus)
    current_joints = [
        0.2298598625,
        0.4161388536,
        0.0147654855,
        -0.2394591734,
        0.2666339498,
        0.0500909495,
    ]
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline and controller.state != controller.DRY_RUN:
            tcp_pub.publish(
                matrix_to_pose(
                    controller.kinematics.forward(current_joints),
                    "base_link",
                    stimulus.get_clock().now().to_msg(),
                )
            )
            joints = JointState()
            joints.header.stamp = stimulus.get_clock().now().to_msg()
            joints.name = list(controller.kinematics.names)
            joints.position = current_joints
            joint_pub.publish(joints)
            # Regression case: the Cartesian-only controller was rejected
            # here with arm_status=4 despite a legal alternate IK branch.
            object_pub.publish(make_pose(stimulus, (0.6061, 0.2055, 0.082)))

            arm = AgxArmStatus()
            arm.ctrl_mode = 1
            arm.arm_status = 0
            arm.err_status = 0
            arm.motion_status = 0
            arm_pub.publish(arm)

            gripper = GripperStatus()
            gripper.header.stamp = stimulus.get_clock().now().to_msg()
            gripper.width = 0.05
            gripper.force = 0.10
            gripper.driver_enable_status = True
            gripper.homing_status = True
            gripper_pub.publish(gripper)

            source_stamp = stimulus.get_clock().now().to_msg()
            model = String()
            model.data = json.dumps(
                {
                    "model_id": "model_plane_cleaned_fixed",
                    "stamp_sec": source_stamp.sec,
                    "stamp_nanosec": source_stamp.nanosec,
                }
            )
            model_pub.publish(model)
            recovery_pub.publish(Bool(data=False))

            for _ in range(5):
                executor.spin_once(timeout_sec=0.02)

        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)

        if controller.state != controller.DRY_RUN:
            raise RuntimeError(
                f"controller did not complete dry-run; final state={controller.state}"
            )
        if controls:
            raise RuntimeError(f"dry-run leaked hardware commands: {controls}")
        decoded = [json.loads(item) for item in statuses]
        if not any(item.get("state") == "dry_run" for item in decoded):
            raise RuntimeError("dry-run completion status was not published")
        if controller.lift_target is None or controller.grasp_target is None:
            raise RuntimeError("grasp/lift targets were not generated")
        lift_delta = float(
            controller.lift_target[2, 3] - controller.grasp_target[2, 3]
        )
        if abs(lift_delta - 0.05) > 1e-9:
            raise RuntimeError(f"expected 0.05m lift, got {lift_delta}")
        print("ROS dry-run passed")
        print("no control commands published")
        print(f"lift delta: {lift_delta:.3f} m")
    finally:
        executor.remove_node(controller)
        executor.remove_node(stimulus)
        controller.destroy_node()
        stimulus.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
