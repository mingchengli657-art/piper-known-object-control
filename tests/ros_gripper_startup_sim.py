#!/usr/bin/env python3
"""ROS integration test for startup normalization without robot hardware."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agx_arm_msgs.msg import AgxArmStatus, GripperStatus


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from grasp_controller import KnownObjectGrasp, load_configuration  # noqa: E402


def main() -> None:
    config, tcp_T_camera = load_configuration(
        ROOT / "config" / "object_005_grasp.yaml"
    )
    rclpy.init()
    controller = KnownObjectGrasp(
        argparse.Namespace(execute=True, gripper_preflight_only=True),
        config,
        tcp_T_camera,
    )
    fake = Node("fake_piper_startup_feedback")

    tcp_pub = fake.create_publisher(
        PoseStamped, config["topics"]["tcp_feedback"], 10
    )
    joint_pub = fake.create_publisher(
        JointState, config["topics"]["joint_feedback"], 10
    )
    arm_pub = fake.create_publisher(
        AgxArmStatus, config["topics"]["arm_status"], 10
    )
    gripper_pub = fake.create_publisher(
        GripperStatus, config["topics"]["gripper_status"], 10
    )

    gripper_width = 0.0997
    gripper_target = gripper_width
    gripper_commands: list[float] = []
    arm_commands: list[str] = []

    def gripper_command(msg: JointState) -> None:
        nonlocal gripper_target
        if msg.position:
            gripper_target = float(msg.position[0])
            gripper_commands.append(gripper_target)

    subscriptions = []
    subscriptions.append(fake.create_subscription(
        JointState,
        config["topics"]["joint_command"],
        gripper_command,
        10,
    ))
    subscriptions.append(fake.create_subscription(
        JointState,
        config["topics"]["move_j"],
        lambda _msg: arm_commands.append("move_j"),
        10,
    ))
    subscriptions.append(fake.create_subscription(
        PoseStamped,
        config["topics"]["move_p"],
        lambda _msg: arm_commands.append("move_p"),
        10,
    ))
    subscriptions.append(fake.create_subscription(
        PoseStamped,
        config["topics"]["move_l"],
        lambda _msg: arm_commands.append("move_l"),
        10,
    ))

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(fake)
    deadline = time.monotonic() + 6.0
    try:
        while (
            time.monotonic() < deadline
            and controller.state != controller.PREFLIGHT_ONLY
        ):
            delta = gripper_target - gripper_width
            gripper_width += max(-0.001, min(0.001, delta))

            tcp = PoseStamped()
            tcp.header.stamp = fake.get_clock().now().to_msg()
            tcp.header.frame_id = "base_link"
            tcp.pose.position.x = 0.20
            tcp.pose.position.z = 0.25
            tcp.pose.orientation.w = 1.0
            tcp_pub.publish(tcp)

            joints = JointState()
            joints.header.stamp = fake.get_clock().now().to_msg()
            joints.name = list(controller.kinematics.names)
            joints.position = [0.0] * len(joints.name)
            joint_pub.publish(joints)

            arm = AgxArmStatus()
            arm.ctrl_mode = 1
            arm.arm_status = 0
            arm.teach_status = 2
            arm.motion_status = 0
            arm.err_status = 0
            arm_pub.publish(arm)

            gripper = GripperStatus()
            gripper.header.stamp = fake.get_clock().now().to_msg()
            gripper.width = gripper_width
            gripper.force = -0.02
            gripper.driver_enable_status = True
            gripper.homing_status = False
            gripper_pub.publish(gripper)

            for _ in range(3):
                executor.spin_once(timeout_sec=0.02)

        if controller.state != controller.PREFLIGHT_ONLY:
            raise RuntimeError(
                "startup normalization did not reach PREFLIGHT_ONLY; "
                f"state={controller.state}"
            )
        if not gripper_commands:
            raise RuntimeError("startup normalization published no gripper command")
        if abs(gripper_width - 0.080) > 0.002:
            raise RuntimeError(f"unexpected normalized width {gripper_width:.4f}m")
        if arm_commands:
            raise RuntimeError(
                f"startup normalization leaked arm commands: {arm_commands}"
            )
        print("ROS gripper startup simulation passed")
        print(f"normalized width: {gripper_width:.4f} m")
        print("homing_status=false accepted only by calibrated v189 policy")
        print("no arm motion commands published")
    finally:
        executor.remove_node(controller)
        executor.remove_node(fake)
        controller.destroy_node()
        fake.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
