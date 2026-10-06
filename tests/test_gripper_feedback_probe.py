"""Pure tests for the read-only gripper feedback probe bookkeeping."""

from types import SimpleNamespace

import pytest

from tools.gripper_feedback_probe import (
    ProbeStats,
    header_stamp_ns,
    reconstructed_status_bits,
    sample_key,
)


def message(width=0.04, force=0.2, **flags):
    values = {
        "width": width,
        "force": force,
        "header": SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34)),
        "voltage_too_low": False,
        "motor_overheating": False,
        "driver_overcurrent": False,
        "driver_overheating": False,
        "sensor_status": False,
        "driver_error_status": False,
        "driver_enable_status": True,
        "homing_status": True,
    }
    values.update(flags)
    return SimpleNamespace(**values)


def test_status_key_changes_when_a_real_field_changes():
    assert sample_key(message()) != sample_key(message(width=0.041))
    assert sample_key(message()) != sample_key(message(homing_status=False))


def test_stats_counts_consecutive_identical_payloads_and_rate():
    stats = ProbeStats()
    msg = message()
    assert stats.observe(msg, 10.0) == 1
    assert stats.observe(msg, 10.01) == 2
    assert stats.observe(message(width=0.041), 10.02) == 1
    assert stats.sample_count == 3
    assert stats.receive_hz == pytest.approx(100.0)


def test_status_bits_are_only_a_recomposition_of_ros_booleans():
    msg = message(
        voltage_too_low=True,
        driver_overcurrent=True,
        homing_status=True,
    )
    assert reconstructed_status_bits(msg) == (1 << 0) | (1 << 2) | (1 << 6) | (1 << 7)


def test_header_stamp_keeps_ros_nanoseconds():
    assert header_stamp_ns(message()) == 12_000_000_034
