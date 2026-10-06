"""Pure software tests for calibrated gripper startup normalization."""

import pytest

from vendor.piper_ros2_control.gripper_startup import (
    GripperStartupFeedback,
    GripperStartupPreflight,
    GripperStartupState,
)


def checker(*, homing_required=False):
    return GripperStartupPreflight(
        feedback_limit_m=(-0.0015, 0.1015),
        safe_width_m=0.080,
        feedback_timeout_s=0.35,
        timeout_s=2.0,
        target_tolerance_m=0.002,
        minimum_response_m=0.003,
        direction_tolerance_m=0.001,
        settle_dwell_s=0.25,
        homing_required=homing_required,
    )


def sample(width, stamp, *, healthy=True, homed=False, force=-0.02):
    return GripperStartupFeedback(width, force, healthy, homed, stamp)


def test_normalization_requires_directional_motion_and_settle():
    check = checker()
    check.observe(sample(0.100, 1.0), 1.0)
    decision = check.begin(1.0)
    assert decision.action == "normalize"
    assert decision.target_m == pytest.approx(0.080)

    check.observe(sample(0.090, 1.1), 1.1)
    check.observe(sample(0.0805, 1.2), 1.2)
    assert check.observe(sample(0.0804, 1.46), 1.46).action == "ready"
    assert check.state is GripperStartupState.READY


def test_already_safe_still_requires_stable_fresh_feedback():
    check = checker()
    check.observe(sample(0.0805, 1.0), 1.0)
    assert check.begin(1.0).action == "monitor"
    assert check.observe(sample(0.0804, 1.1), 1.1).action is None
    assert check.observe(sample(0.0803, 1.3), 1.3).action == "ready"


def test_wrong_direction_faults():
    check = checker()
    check.observe(sample(0.100, 1.0), 1.0)
    check.begin(1.0)
    decision = check.observe(sample(0.1012, 1.1), 1.1)
    assert "wrong direction" in decision.error
    assert check.state is GripperStartupState.FAULT


def test_no_motion_times_out_instead_of_passing():
    check = checker()
    check.observe(sample(0.100, 1.0), 1.0)
    check.begin(1.0)
    check.observe(sample(0.100, 2.9), 2.9)
    decision = check.tick(3.01)
    assert "without minimum feedback response" in decision.error


def test_out_of_calibrated_range_is_rejected():
    check = checker()
    check.observe(sample(0.120, 1.0), 1.0)
    decision = check.begin(1.0)
    assert "outside calibrated feedback range" in decision.error


def test_homing_false_follows_explicit_device_policy():
    allowed = checker(homing_required=False)
    allowed.observe(sample(0.080, 1.0, homed=False), 1.0)
    assert allowed.begin(1.0).error is None

    required = checker(homing_required=True)
    required.observe(sample(0.080, 1.0, homed=False), 1.0)
    assert "homing status is required" in required.begin(1.0).error
