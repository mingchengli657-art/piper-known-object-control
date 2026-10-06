import numpy as np

from motion_settle import MotionSettler


def pose(x=0.3, y=0.0, z=0.2):
    result = np.eye(4, dtype=float)
    result[:3, 3] = [x, y, z]
    return result


def test_settle_requires_stopped_status_and_dwell():
    gate = MotionSettler(
        required_samples=3,
        translation_limit_m=0.001,
        rotation_limit_deg=0.5,
        dwell_s=0.10,
    )
    assert not gate.observe(pose(), motion_status=1, now=0.0)
    assert not gate.observe(pose(), motion_status=0, now=1.0)
    assert not gate.observe(pose(0.0004), motion_status=0, now=1.05)
    assert not gate.observe(pose(0.0002), motion_status=0, now=1.10)
    assert gate.observe(pose(0.0003), motion_status=0, now=1.21)


def test_settle_resets_after_tcp_jump():
    gate = MotionSettler(required_samples=3, dwell_s=0.05)
    assert not gate.observe(pose(), motion_status=0, now=0.0)
    assert not gate.observe(pose(0.0002), motion_status=0, now=0.05)
    assert not gate.observe(pose(0.01), motion_status=0, now=0.10)
    assert not gate.observe(pose(0.0101), motion_status=0, now=0.15)
    assert not gate.observe(pose(0.0100), motion_status=0, now=0.19)
    assert gate.observe(pose(0.01005), motion_status=0, now=0.26)
