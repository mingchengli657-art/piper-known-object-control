#!/usr/bin/env python3
"""Read-only ROS probe for the AGX gripper feedback topic.

This tool deliberately creates one subscription and no publishers.  It never
opens CAN, calls an SDK command, or writes a calibration value.  The receive
clock is recorded separately from the ROS header stamp so stale/replayed
feedback can be distinguished during a physical observation.
"""

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, TextIO


DEFAULT_TOPIC = "/feedback/gripper_status"
DEFAULT_REPORT_PERIOD_S = 1.0
DEFAULT_FREEZE_SAMPLES = 20


def sample_key(msg: Any) -> tuple[Any, ...]:
    """Return the complete payload used for repeated-sample detection."""
    return (
        float(msg.width),
        float(msg.force),
        bool(msg.voltage_too_low),
        bool(msg.motor_overheating),
        bool(msg.driver_overcurrent),
        bool(msg.driver_overheating),
        bool(msg.sensor_status),
        bool(msg.driver_error_status),
        bool(msg.driver_enable_status),
        bool(msg.homing_status),
    )


def header_stamp_ns(msg: Any) -> int:
    """Return a ROS Header stamp in nanoseconds (zero is retained)."""
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def reconstructed_status_bits(msg: Any) -> int:
    """Recompose the documented status bits for convenient log inspection.

    This is explicitly a local recomposition of the individual ROS booleans;
    it is not treated as an additional device-provided field.
    """
    flags = (
        "voltage_too_low",
        "motor_overheating",
        "driver_overcurrent",
        "driver_overheating",
        "sensor_status",
        "driver_error_status",
        "driver_enable_status",
        "homing_status",
    )
    return sum(int(bool(getattr(msg, name))) << bit
               for bit, name in enumerate(flags))


@dataclass
class ProbeStats:
    """Pure bookkeeping for receive rate and repeated-payload observations."""

    first_receive_monotonic: Optional[float] = None
    last_receive_monotonic: Optional[float] = None
    sample_count: int = 0
    repeated_count: int = 0
    last_key: Optional[tuple[Any, ...]] = None
    last_width_m: Optional[float] = None
    last_force_n: Optional[float] = None

    def observe(self, msg: Any, receive_monotonic: float) -> int:
        """Record one message and return its consecutive identical count."""
        key = sample_key(msg)
        if self.first_receive_monotonic is None:
            self.first_receive_monotonic = receive_monotonic
        if key == self.last_key:
            self.repeated_count += 1
        else:
            self.repeated_count = 1
            self.last_key = key
        self.last_receive_monotonic = receive_monotonic
        self.sample_count += 1
        self.last_width_m = float(msg.width)
        self.last_force_n = float(msg.force)
        return self.repeated_count

    @property
    def receive_hz(self) -> float:
        """Estimate receive frequency from local receive times."""
        if (self.first_receive_monotonic is None
                or self.last_receive_monotonic is None):
            return 0.0
        elapsed = self.last_receive_monotonic - self.first_receive_monotonic
        if elapsed <= 0.0:
            return 0.0
        return max(0.0, (self.sample_count - 1) / elapsed)


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only subscriber for /feedback/gripper_status; "
            "no arm/gripper commands are ever published."))
    parser.add_argument("--topic", default=DEFAULT_TOPIC,
                        help=f"feedback topic (default: {DEFAULT_TOPIC})")
    parser.add_argument(
        "--csv", type=Path, default=None, metavar="PATH",
        help="optional CSV output path; parent directories are created")
    parser.add_argument(
        "--duration-s", type=float, default=0.0, metavar="SECONDS",
        help="stop after this time; 0 means run until Ctrl-C")
    parser.add_argument(
        "--report-period-s", type=float, default=DEFAULT_REPORT_PERIOD_S,
        metavar="SECONDS", help="local summary period (default: 1.0)")
    parser.add_argument(
        "--freeze-samples", type=int, default=DEFAULT_FREEZE_SAMPLES,
        metavar="N", help="repeated payload warning threshold (default: 20)")
    return parser


def _csv_header() -> list[str]:
    return [
        "receive_monotonic_s",
        "header_stamp_ns",
        "width_m",
        "force_N",
        "voltage_too_low",
        "motor_overheating",
        "driver_overcurrent",
        "driver_overheating",
        "sensor_status",
        "driver_error_status",
        "driver_enable_status",
        "homing_status",
        "status_bits_reconstructed",
        "consecutive_identical_payloads",
    ]


def _csv_row(msg: Any, receive_monotonic: float,
             consecutive_identical: int) -> list[Any]:
    return [
        f"{receive_monotonic:.9f}",
        header_stamp_ns(msg),
        f"{float(msg.width):.9f}",
        f"{float(msg.force):.6f}",
        bool(msg.voltage_too_low),
        bool(msg.motor_overheating),
        bool(msg.driver_overcurrent),
        bool(msg.driver_overheating),
        bool(msg.sensor_status),
        bool(msg.driver_error_status),
        bool(msg.driver_enable_status),
        bool(msg.homing_status),
        reconstructed_status_bits(msg),
        consecutive_identical,
    ]


def main(argv: Optional[list[str]] = None) -> int:
    """Run the read-only ROS subscriber."""
    args = _build_argument_parser().parse_args(argv)
    if args.duration_s < 0.0:
        raise SystemExit("--duration-s must be >= 0")
    if args.report_period_s <= 0.0:
        raise SystemExit("--report-period-s must be > 0")
    if args.freeze_samples < 2:
        raise SystemExit("--freeze-samples must be >= 2")

    # Keep ROS imports after argument parsing, so --help works in a non-ROS
    # shell and this module remains importable for pure software tests.
    import rclpy
    from agx_arm_msgs.msg import GripperStatus
    from rclpy.node import Node

    csv_file: Optional[TextIO] = None
    csv_writer: Optional[csv.writer] = None
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        csv_file = args.csv.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(_csv_header())
        csv_file.flush()

    class GripperFeedbackProbe(Node):
        """One ROS subscription plus local observation bookkeeping."""

        def __init__(self) -> None:
            super().__init__("gripper_feedback_probe")
            self.stats = ProbeStats()
            self.last_msg: Optional[Any] = None
            self.last_warning_count = 0
            self.started_monotonic = time.monotonic()
            self.done = False
            # Intentionally no create_publisher call in this node.
            self.subscription = self.create_subscription(
                GripperStatus, args.topic, self._on_feedback, 20)
            self.timer = self.create_timer(
                args.report_period_s, self._report)

        def _on_feedback(self, msg: Any) -> None:
            now = time.monotonic()
            repeated = self.stats.observe(msg, now)
            self.last_msg = msg
            if csv_writer is not None and csv_file is not None:
                csv_writer.writerow(_csv_row(msg, now, repeated))
                csv_file.flush()
            if repeated >= args.freeze_samples:
                if self.last_warning_count < args.freeze_samples:
                    self.get_logger().warning(
                        f"suspected frozen payload: {repeated} consecutive "
                        "identical samples; this is an observation, not a "
                        "conclusion")
                self.last_warning_count = repeated
            else:
                self.last_warning_count = 0

        def _report(self) -> None:
            if self.last_msg is None:
                self.get_logger().info(
                    f"waiting for {args.topic} "
                    "(read-only; no command publishers)")
            else:
                msg = self.last_msg
                header_ns = header_stamp_ns(msg)
                age_note = "header_stamp=zero" if header_ns == 0 else (
                    f"header_stamp_ns={header_ns}")
                self.get_logger().info(
                    f"samples={self.stats.sample_count} "
                    f"receive_hz={self.stats.receive_hz:.1f} "
                    f"width={float(msg.width):.6f} m "
                    f"force={float(msg.force):.3f} N "
                    f"enable={bool(msg.driver_enable_status)} "
                    f"homing={bool(msg.homing_status)} "
                    f"sensor_error={bool(msg.sensor_status)} "
                    f"driver_error={bool(msg.driver_error_status)} "
                    f"same={self.stats.repeated_count} ({age_note})")
            if (args.duration_s > 0.0
                    and time.monotonic() - self.started_monotonic
                    >= args.duration_s):
                self.done = True

    rclpy.init(args=None)
    node = GripperFeedbackProbe()
    print("READ-ONLY: subscribed to " + args.topic,
          "; no arm/gripper command publishers are created.", flush=True)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if csv_file is not None:
            csv_file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
