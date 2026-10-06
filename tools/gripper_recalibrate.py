#!/usr/bin/env python3
"""Explicit recovery calibration for an AGX gripper on Piper X.

This tool never enables or moves the arm and never commands a gripper target.
It only queries the configured stroke, disables the gripper, waits for the
operator to place it at the fully-closed physical reference, and writes zero
after two exact confirmation phrases.
"""

import argparse
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


PREPARE_CONFIRMATION = "PREPARE_GRIPPER_ZERO"
WRITE_CONFIRMATION = "WRITE_GRIPPER_ZERO"


def _status_text(status):
    foc = status.msg.foc_status
    return (
        f"width={status.msg.value:.6f} "
        f"force={status.msg.force:.6f} "
        f"mode={status.msg.mode} "
        f"enabled={bool(foc.driver_enable_status)} "
        f"homing={bool(foc.homing_status)}"
    )


def _faults(status):
    foc = status.msg.foc_status
    names = (
        "voltage_too_low",
        "motor_overheating",
        "driver_overcurrent",
        "driver_overheating",
        "sensor_status",
        "driver_error_status",
    )
    return [name for name in names if bool(getattr(foc, name))]


def _wait_status(gripper, timeout_s, predicate=lambda _status: True):
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = gripper.get_gripper_status()
        if latest is not None and predicate(latest):
            return latest
        time.sleep(0.05)
    return latest


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Explicitly overwrite a wrong AGX gripper zero after manual "
            "placement at the fully-closed physical reference."
        )
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--expected-max-range-m", type=float, default=0.1)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument(
        "--recalibrate",
        action="store_true",
        help="Required acknowledgement that an existing/wrong zero may be overwritten.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.recalibrate:
        print("ABORT: pass --recalibrate to permit an explicit zero overwrite")
        return 2

    robot = None
    try:
        config = create_agx_arm_config(
            robot=ArmModel.PIPER_X,
            firmeware_version=PiperFW.DEFAULT,
            interface="socketcan",
            channel=args.channel,
        )
        robot = AgxArmFactory.create_arm(config)
        robot.connect()
        gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

        status = _wait_status(gripper, args.timeout_s)
        if status is None:
            print("ABORT: no gripper feedback")
            return 3
        print(f"CURRENT: {_status_text(status)}")

        faults = _faults(status)
        if faults:
            print(f"ABORT: gripper fault flags are active: {', '.join(faults)}")
            return 3
        if status.msg.mode != "width":
            print(f"ABORT: expected width mode, got {status.msg.mode!r}")
            return 3

        param = gripper.get_gripper_teaching_pendant_param(timeout=2.0)
        if param is None:
            print("ABORT: no teaching-pendant parameter response")
            return 3
        max_range = float(param.msg.max_range_config)
        print(
            "PARAM: "
            f"teaching_range_per={param.msg.teaching_range_per} "
            f"max_range_config={max_range:.6f} "
            f"teaching_friction={param.msg.teaching_friction}"
        )
        if abs(max_range - args.expected_max_range_m) > 1e-6:
            print(
                "ABORT: configured max range does not match the measured/expected "
                f"range ({max_range:.6f} != {args.expected_max_range_m:.6f})"
            )
            return 3

        confirmation = input(
            f"Type {PREPARE_CONFIRMATION} to disable the gripper: "
        ).strip()
        if confirmation != PREPARE_CONFIRMATION:
            print("ABORT: prepare confirmation rejected; zero was not written")
            return 2

        gripper.disable_gripper()
        status = _wait_status(
            gripper,
            args.timeout_s,
            lambda sample: not sample.msg.foc_status.driver_enable_status,
        )
        if status is None or status.msg.foc_status.driver_enable_status:
            print("ABORT: gripper disable was not confirmed")
            return 3
        print(f"DISABLED: {_status_text(status)}")

        input(
            "Manually close the empty gripper to its natural physical stop; "
            "do not force it. Press Enter only when it is fully closed: "
        )
        time.sleep(0.3)
        status = _wait_status(gripper, args.timeout_s)
        if status is None:
            print("ABORT: feedback was lost after manual placement")
            return 3
        print(f"ZERO_CANDIDATE: {_status_text(status)}")

        confirmation = input(
            f"Type {WRITE_CONFIRMATION} to write THIS position as zero: "
        ).strip()
        if confirmation != WRITE_CONFIRMATION:
            print("ABORT: write confirmation rejected; zero was not written")
            return 2

        acknowledged = bool(gripper.calibrate_gripper(timeout=2.0))
        print(f"ZERO_ACK: {acknowledged}")
        if not acknowledged:
            print("ABORT: controller did not acknowledge the zero write")
            return 4

        status = _wait_status(
            gripper,
            args.timeout_s,
            lambda sample: bool(sample.msg.foc_status.homing_status),
        )
        if status is None:
            print("VERIFY_FAILED: no post-write gripper feedback")
            return 4
        print(f"AFTER: {_status_text(status)}")
        if not status.msg.foc_status.homing_status:
            print("VERIFY_FAILED: zero ACK was received but homing_status stayed false")
            return 4

        print("SUCCESS: zero write acknowledged and homing_status is true")
        return 0
    except KeyboardInterrupt:
        print("\nABORT: interrupted by operator; no further operation will be sent")
        return 130
    except EOFError:
        print("\nABORT: interactive input ended; run this file from a terminal")
        return 2
    except Exception as error:
        print(f"ABORT: {error}")
        return 5
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception as error:
                print(f"WARNING: disconnect failed: {error}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
