#!/usr/bin/env python3
"""Receive the FoundationPose base-frame target for a robot adapter.

This node is deliberately motionless: it validates and republishes the 6D
target, and writes a small JSON snapshot for a later Piper/夹爪 controller.
The actual arm command must be implemented by the installed Piper driver and
kept behind its own explicit safety/execute gate.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class PoseReceiver(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("piper_pose_receiver")
        self.args = args
        self.latest_json: dict | None = None
        self.target_pub = self.create_publisher(PoseStamped, args.target_topic, 10)
        self.create_subscription(PoseStamped, args.pose_topic, self.pose_callback, 20)
        self.create_subscription(String, args.pose_json_topic, self.json_callback, 10)
        self.get_logger().info(
            f"listening to {args.pose_topic} (expected frame={args.base_frame}); "
            f"republishing {args.target_topic}; no robot commands are sent"
        )

    def json_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("ignored malformed FoundationPose JSON")
            return
        if not isinstance(data, dict):
            return
        self.latest_json = data

    def pose_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.args.base_frame:
            self.get_logger().warning(
                f"ignored pose in frame {msg.header.frame_id!r}; "
                f"expected {self.args.base_frame!r}"
            )
            return
        p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        norm = math.sqrt(sum(float(value) ** 2 for value in q))
        if not _finite((*p, *q)) or norm < 0.9 or norm > 1.1:
            self.get_logger().warning("ignored non-finite or non-unit target pose")
            return

        self.target_pub.publish(msg)
        payload = {
            "schema_version": 1,
            "frame_id": msg.header.frame_id,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "position_m": [float(value) for value in p],
            "quaternion_xyzw": [float(value) for value in q],
            "model_id": "" if self.latest_json is None else str(self.latest_json.get("model_id", "")),
        }
        self._atomic_write(payload)

    def _atomic_write(self, payload: dict) -> None:
        path = Path(self.args.output).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-topic", default="/foundationpose/object_pose_base")
    parser.add_argument("--pose-json-topic", default="/foundationpose/object_pose_json")
    parser.add_argument("--target-topic", default="/piper_control/target_pose")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--output", default="./runtime/piper_target.json")
    args = parser.parse_args()
    rclpy.init()
    node = PoseReceiver(args)
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
