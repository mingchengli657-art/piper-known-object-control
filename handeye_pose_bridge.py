#!/usr/bin/env python3
"""Convert a camera-frame object pose to the Piper base frame.

The node intentionally does not command the robot. It is the safe integration
stage between FoundationPose and Piper:

    T_base_object = T_base_tcp * T_tcp_camera * T_camera_object

FoundationPose is expected to publish geometry_msgs/PoseStamped on
``object_pose_camera_topic``. The Piper driver publishes its TCP pose on
``/feedback/tcp_pose``; that message has an empty frame_id in the current
driver, so the configured ``tcp_pose_assumed_frame`` is applied.
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker
from grasp_geometry import as_transform


def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    p = msg.pose.position
    q = msg.pose.orientation
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_pose(T: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = T[:3, 3]
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
    return msg


def compose_base_object(T_base_tcp: np.ndarray, T_tcp_camera: np.ndarray,
                        T_camera_object: np.ndarray) -> np.ndarray:
    return T_base_tcp @ T_tcp_camera @ T_camera_object


def stamp_ns(msg: PoseStamped) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class HandeyePoseBridge(Node):
    def __init__(self, cfg: dict):
        super().__init__("handeye_pose_bridge")
        self.cfg = cfg
        self.base_frame = str(cfg.get("base_frame", "base_link"))
        self.tcp_frame = str(cfg.get("tcp_frame", "link6"))
        self.camera_frame = str(cfg.get("camera_frame", "d405_color_optical_frame"))
        self.object_frame = str(cfg.get("object_frame", "object_estimate"))
        self.assumed_tcp_frame = str(cfg.get("tcp_pose_assumed_frame", self.base_frame))
        self.tcp_T_camera = as_transform(cfg["tcp_T_camera"])
        self.axis_length = float(cfg.get("axis_length_m", 0.08))
        self.line_width = float(cfg.get("marker_line_width_m", 0.004))
        self.latest_tcp = None
        self.latest_tcp_stamp_ns = 0
        self.tcp_history = deque(maxlen=int(cfg.get("tcp_history_size", 200)))
        self.max_tcp_time_diff_s = float(cfg.get("max_tcp_time_diff_s", 0.10))
        self.warned_no_tcp = False
        self.warned_frame = False
        self.warned_unstamped_tcp = False
        self.warned_time_mismatch = False

        self.pose_pub = self.create_publisher(
            PoseStamped, str(cfg.get("object_pose_base_topic", "/foundationpose/object_pose_base")), 10
        )
        self.marker_pub = self.create_publisher(
            Marker, str(cfg.get("axes_marker_topic", "/foundationpose/object_axes_marker")), 10
        )
        self.create_subscription(
            PoseStamped, str(cfg.get("tcp_pose_topic", "/feedback/tcp_pose")), self.tcp_callback, 20
        )
        self.create_subscription(
            PoseStamped, str(cfg.get("object_pose_camera_topic", "/foundationpose/object_pose_camera")),
            self.object_callback, 20
        )
        self.get_logger().info(
            f"hand-eye: {self.base_frame} <- {self.tcp_frame} <- {self.camera_frame}; "
            f"output={cfg.get('object_pose_base_topic', '/foundationpose/object_pose_base')}"
        )

    def tcp_callback(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.assumed_tcp_frame:
            self.get_logger().warning(
                f"/feedback/tcp_pose frame_id={msg.header.frame_id!r}; "
                f"configured assumption is {self.assumed_tcp_frame!r}"
            )
            self.warned_frame = True
            return
        if stamp_ns(msg) <= 0:
            return
        try:
            self.latest_tcp = as_transform(pose_to_matrix(msg))
        except ValueError:
            return
        self.latest_tcp_stamp_ns = stamp_ns(msg)
        if self.latest_tcp_stamp_ns > 0:
            self.tcp_history.append((self.latest_tcp_stamp_ns, self.latest_tcp.copy()))

    def tcp_for_image_stamp(self, image_stamp_ns: int):
        """Return the TCP pose nearest to an image timestamp.

        The old bridge used whichever TCP message happened to arrive last. That
        is acceptable while static, but creates a systematic base-frame error
        whenever the arm moves. A bounded timestamp match is required before
        using a pose for grasp planning.
        """
        if self.latest_tcp is None:
            return None, None
        if image_stamp_ns <= 0 or not self.tcp_history:
            if not self.warned_unstamped_tcp:
                self.get_logger().warning(
                    "TCP or image timestamp is zero; refusing an unsynchronized transform"
                )
                self.warned_unstamped_tcp = True
            return None, None
        stamp, pose = min(
            self.tcp_history,
            key=lambda item: abs(item[0] - image_stamp_ns),
        )
        dt_s = abs(stamp - image_stamp_ns) * 1e-9
        if dt_s > self.max_tcp_time_diff_s:
            if not self.warned_time_mismatch:
                self.get_logger().warning(
                    f"no TCP pose within {self.max_tcp_time_diff_s:.3f}s of image; "
                    f"nearest difference is {dt_s:.3f}s"
                )
                self.warned_time_mismatch = True
            return None, dt_s
        return pose, dt_s

    def object_callback(self, msg: PoseStamped):
        if self.latest_tcp is None:
            if not self.warned_no_tcp:
                self.get_logger().warning("waiting for TCP pose topic /feedback/tcp_pose")
                self.warned_no_tcp = True
            return
        image_stamp_ns = stamp_ns(msg)
        T_base_tcp, tcp_dt_s = self.tcp_for_image_stamp(image_stamp_ns)
        if T_base_tcp is None:
            return
        source_frame = msg.header.frame_id or self.camera_frame
        if msg.header.frame_id and msg.header.frame_id != self.camera_frame:
            self.get_logger().warning(
                f"object pose frame_id={msg.header.frame_id!r}, expected {self.camera_frame!r}"
            )
            self.warned_frame = True
            return
        T_base_object = compose_base_object(T_base_tcp, self.tcp_T_camera, pose_to_matrix(msg))
        out = matrix_to_pose(T_base_object, self.base_frame, msg.header.stamp)
        self.pose_pub.publish(out)
        self.publish_axes(T_base_object, msg.header.stamp)
        self.get_logger().debug(
            f"object camera frame={source_frame}; base xyz="
            f"[{T_base_object[0,3]:+.3f}, {T_base_object[1,3]:+.3f}, {T_base_object[2,3]:+.3f}]; "
            f"tcp-image dt={('n/a' if tcp_dt_s is None else f'{tcp_dt_s * 1000.0:.1f}ms')}"
        )

    def publish_axes(self, T: np.ndarray, stamp):
        origin = T[:3, 3]
        ends = [origin + T[:3, i] * self.axis_length for i in range(3)]
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = "foundationpose_object"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = self.line_width
        for end, color in zip(ends, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))):
            m.points.extend([
                Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
            ])
            for _ in range(2):
                c = type(m.color)()
                c.r, c.g, c.b, c.a = *color, 1.0
                m.colors.append(c)
        self.marker_pub.publish(m)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default=str(Path(__file__).parent / "config/online_handeye.yaml"))
    args = p.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    if cfg.get("calibration_confirmed") is not True:
        p.error("local hand-eye calibration must be filled in and calibration_confirmed set to true")
    rclpy.init()
    node = HandeyePoseBridge(cfg)
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
