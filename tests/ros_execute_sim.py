#!/usr/bin/env python3
"""Full ROS state-machine simulation with no connection to robot hardware."""

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


def pose(node: Node, xyz=(0.10, 0.00, 0.20)) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
    msg.pose.orientation.w = 1.0
    return msg


def main(*, bad_feedback: bool = False) -> None:
    config, tcp_T_camera = load_configuration(
        ROOT / "config" / "object_003_grasp.yaml"
    )
    rclpy.init()
    controller = KnownObjectGrasp(
        argparse.Namespace(execute=True), config, tcp_T_camera
    )
    fake = Node("fake_piper_and_perception")

    tcp_pub = fake.create_publisher(PoseStamped, config["topics"]["tcp_feedback"], 10)
    joint_pub = fake.create_publisher(
        JointState, config["topics"]["joint_feedback"], 10
    )
    object_pub = fake.create_publisher(
        PoseStamped, config["topics"]["object_pose_base"], 10
    )
    arm_pub = fake.create_publisher(AgxArmStatus, config["topics"]["arm_status"], 10)
    gripper_pub = fake.create_publisher(
        GripperStatus, config["topics"]["gripper_status"], 10
    )
    model_pub = fake.create_publisher(
        String, config["topics"]["object_pose_json"], 10
    )
    recovery_pub = fake.create_publisher(Bool, config["topics"]["recovery"], 10)

    current_joints = [
        0.2298598625,
        0.4161388536,
        0.0147654855,
        -0.2394591734,
        0.2666339498,
        0.0500909495,
    ]
    tcp = matrix_to_pose(
        controller.kinematics.forward(current_joints),
        "base_link",
        fake.get_clock().now().to_msg(),
    )
    # The normal simulation gives the controller a moving opening feedback and
    # a stable contact plateau.  The bad-feedback wrapper below freezes the
    # same signal at the former 83.4 mm value to prove that formal mode aborts.
    gripper_width = 0.0834 if bad_feedback else 0.1000
    max_gripper_width = gripper_width
    gripper_target = gripper_width
    pose_commands: list[tuple[str, object]] = []
    gripper_commands: list[float] = []

    def motion_cb(kind: str, msg: PoseStamped) -> None:
        nonlocal tcp
        pose_commands.append((kind, msg))
        tcp = msg

    def move_j_cb(msg: JointState) -> None:
        nonlocal tcp, current_joints
        pose_commands.append(("move_j", msg))
        current_joints = list(msg.position)
        tcp = matrix_to_pose(
            controller.pregrasp_target,
            "base_link",
            fake.get_clock().now().to_msg(),
        )

    def gripper_cb(msg: JointState) -> None:
        nonlocal gripper_target
        if msg.position:
            requested = float(msg.position[0])
            gripper_target = min(requested, 0.0691) if bad_feedback else requested
            gripper_commands.append(float(msg.position[0]))

    fake.create_subscription(
        PoseStamped,
        config["topics"]["move_p"],
        lambda msg: motion_cb("move_p", msg),
        10,
    )
    fake.create_subscription(
        JointState,
        config["topics"]["move_j"],
        move_j_cb,
        10,
    )
    fake.create_subscription(
        PoseStamped,
        config["topics"]["move_l"],
        lambda msg: motion_cb("move_l", msg),
        10,
    )
    fake.create_subscription(
        JointState, config["topics"]["joint_command"], gripper_cb, 10
    )

    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(fake)
    deadline = time.monotonic() + 25.0
    try:
        while time.monotonic() < deadline and controller.state != controller.HOLDING:
            tcp.header.stamp = fake.get_clock().now().to_msg()
            tcp_pub.publish(tcp)
            joints = JointState()
            joints.header.stamp = fake.get_clock().now().to_msg()
            joints.name = list(controller.kinematics.names)
            joints.position = current_joints
            joint_pub.publish(joints)
            object_pub.publish(pose(fake, (0.6061, 0.2055, 0.082)))

            arm = AgxArmStatus()
            arm.ctrl_mode = 1
            arm.arm_status = 0
            arm.err_status = 0
            arm.motion_status = 0
            arm_pub.publish(arm)

            if not bad_feedback:
                if gripper_target <= 0.045:
                    # The object blocks the closing motion at 55 mm and
                    # produces a sustained, measurable contact force.
                    gripper_width = 0.055
                else:
                    delta = gripper_target - gripper_width
                    if abs(delta) > 0.0008:
                        # Keep at least two fresh, directional samples visible
                        # to OpeningResponseMonitor before target tolerance.
                        gripper_width += max(-0.0015, min(0.0015, delta))
            max_gripper_width = max(max_gripper_width, gripper_width)
            gripper = GripperStatus()
            gripper.header.stamp = fake.get_clock().now().to_msg()
            gripper.width = gripper_width
            gripper.force = (
                0.35 if (not bad_feedback and gripper_target <= 0.045) else 0.0
            )
            gripper.driver_enable_status = True
            gripper.homing_status = True
            gripper_pub.publish(gripper)

            source_stamp = fake.get_clock().now().to_msg()
            metadata = String()
            metadata.data = json.dumps(
                {
                    "model_id": "model_plane_cleaned_fixed",
                    "stamp_sec": source_stamp.sec,
                    "stamp_nanosec": source_stamp.nanosec,
                }
            )
            model_pub.publish(metadata)
            recovery_pub.publish(Bool(data=False))

            for _ in range(5):
                executor.spin_once(timeout_sec=0.02)

        if bad_feedback:
            if controller.state != controller.ABORTED:
                raise RuntimeError(
                    "bad-feedback simulation unexpectedly continued: "
                    f"state={controller.state}"
                )
            kinds = [kind for kind, _msg in pose_commands]
            if any(kind in ("move_j", "move_l") for kind in kinds):
                raise RuntimeError(
                    f"bad-feedback simulation sent arm motion: {kinds}"
                )
            print("ROS bad-gripper-feedback simulation passed")
            print("frozen 83.4 mm feedback was rejected before arm motion")
            return
        if controller.state != controller.HOLDING:
            raise RuntimeError(
                f"execute simulation did not reach holding; state={controller.state}"
            )
        kinds = [kind for kind, _msg in pose_commands]
        if kinds.count("move_j") < 1 or kinds.count("move_l") < 2:
            raise RuntimeError(f"missing expected motion stages: {kinds}")
        if kinds.count("move_p"):
            raise RuntimeError(f"Cartesian MoveP leaked into validated sequence: {kinds}")
        if not gripper_commands or max_gripper_width < 0.069:
            raise RuntimeError("measured physical maximum opening was not reached")
        if min(gripper_commands) > 0.041:
            raise RuntimeError("final close command was not issued before fallback")
        move_l_commands = [msg for kind, msg in pose_commands if kind == "move_l"]
        final_approach = move_l_commands[-2]
        lift_target = move_l_commands[-1]
        lift = lift_target.pose.position.z - final_approach.pose.position.z
        if abs(lift - 0.05) > 1e-9:
            raise RuntimeError(f"expected 0.05m lift, got {lift}")
        print("ROS execute simulation passed")
        print("directional opening feedback response accepted")
        print("15 cm pre-grasp -> linear 10 cm grasp -> linear 5 cm lift completed")
    finally:
        executor.remove_node(controller)
        executor.remove_node(fake)
        controller.destroy_node()
        fake.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
