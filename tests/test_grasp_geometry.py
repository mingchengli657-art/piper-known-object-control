"""Tests for hardware-independent grasp geometry."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from grasp_geometry import (
    average_poses,
    lifted_target,
    make_camera_standoff_target,
    pose_reached,
    poses_stable,
    workspace_contains,
)


def transform(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    value[:3, 3] = [x, y, z]
    return value


def test_stable_window_and_average_reject_translation_outlier():
    poses = [transform(0.40 + delta, 0.02, 0.10) for delta in (0.0, 0.002, -0.001)]
    assert poses_stable(poses, translation_limit_m=0.005, rotation_limit_deg=2.0)
    frozen = average_poses(poses)
    assert frozen[0, 3] == pytest.approx(0.40, abs=1e-9)
    poses.append(transform(0.43, 0.02, 0.10))
    assert not poses_stable(poses, translation_limit_m=0.005, rotation_limit_deg=2.0)


def test_camera_standoff_returns_tcp_target():
    base_T_tcp = transform(0.10, 0.0, 0.20)
    tcp_T_camera = transform(0.05, 0.0, 0.0)
    base_T_object = transform(0.55, 0.0, 0.20)
    target = make_camera_standoff_target(
        base_T_tcp, base_T_object, tcp_T_camera, 0.10
    )
    camera_goal = target @ tcp_T_camera
    assert np.linalg.norm(camera_goal[:3, 3] - base_T_object[:3, 3]) == pytest.approx(0.10)
    forward = camera_goal[:3, 2]
    expected = base_T_object[:3, 3] - camera_goal[:3, 3]
    assert np.dot(forward, expected / np.linalg.norm(expected)) == pytest.approx(1.0)


def test_lift_is_exactly_base_z_and_pose_completion_uses_feedback():
    grasp = transform(0.45, 0.02, 0.08, yaw_deg=20.0)
    lift = lifted_target(grasp, 0.05)
    assert lift[:3, 3].tolist() == pytest.approx([0.45, 0.02, 0.13])
    assert pose_reached(
        transform(0.452, 0.02, 0.131, yaw_deg=21.0),
        lift,
        translation_tolerance_m=0.005,
        rotation_tolerance_deg=2.0,
    )


def test_workspace_bounds_reject_low_target():
    bounds = {"x": [0.10, 0.70], "y": [-0.40, 0.40], "z": [0.015, 0.65]}
    assert workspace_contains(transform(0.45, 0.0, 0.05), bounds)
    assert not workspace_contains(transform(0.45, 0.0, -0.01), bounds)
