"""Pure geometry and validation helpers for one-shot Piper grasps."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from scipy.spatial.transform import Rotation


def as_transform(value: np.ndarray | Sequence[Sequence[float]]) -> np.ndarray:
    """Return a finite rigid 4x4 transform or raise ``ValueError``."""
    transform = np.asarray(value, dtype=float).reshape(4, 4)
    if not np.isfinite(transform).all():
        raise ValueError("transform contains a non-finite value")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError("transform has an invalid homogeneous row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError("transform rotation is not orthonormal")
    if np.linalg.det(rotation) < 0.99:
        raise ValueError("transform rotation is reflected or singular")
    return transform


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular difference between two transforms."""
    relative = as_transform(first)[:3, :3] @ as_transform(second)[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def translation_error_m(first: np.ndarray, second: np.ndarray) -> float:
    """Return Euclidean translation difference between two transforms."""
    return float(
        np.linalg.norm(as_transform(first)[:3, 3] - as_transform(second)[:3, 3])
    )


def poses_stable(
    poses: Sequence[np.ndarray],
    *,
    translation_limit_m: float,
    rotation_limit_deg: float,
) -> bool:
    """Check every pose against the first sample in a bounded window."""
    if len(poses) < 2:
        return False
    reference = as_transform(poses[0])
    return all(
        translation_error_m(reference, current) <= translation_limit_m
        and rotation_error_deg(reference, current) <= rotation_limit_deg
        for current in poses[1:]
    )


def average_poses(poses: Sequence[np.ndarray]) -> np.ndarray:
    """Robustly freeze a pose window using median XYZ and mean rotation."""
    if not poses:
        raise ValueError("cannot average an empty pose sequence")
    transforms = [as_transform(pose) for pose in poses]
    result = np.eye(4, dtype=float)
    result[:3, 3] = np.median(
        np.stack([pose[:3, 3] for pose in transforms], axis=0), axis=0
    )
    result[:3, :3] = Rotation.from_matrix(
        np.stack([pose[:3, :3] for pose in transforms], axis=0)
    ).mean().as_matrix()
    return result


def make_camera_standoff_target(
    base_T_tcp: np.ndarray,
    base_T_object: np.ndarray,
    tcp_T_camera: np.ndarray,
    distance_m: float,
) -> np.ndarray:
    """Place the mounted camera ``distance_m`` in front of the object.

    The result is a TCP target.  The camera optical +Z axis points toward the
    frozen object center while the current camera +Y direction is used as the
    roll hint.  This preserves the geometry used by the successful 10 cm poke
    and grasp tests.
    """
    if not np.isfinite(distance_m) or distance_m <= 0.0:
        raise ValueError("camera standoff distance must be positive")
    base_T_tcp = as_transform(base_T_tcp)
    base_T_object = as_transform(base_T_object)
    tcp_T_camera = as_transform(tcp_T_camera)
    base_T_camera = base_T_tcp @ tcp_T_camera
    camera_position = base_T_camera[:3, 3]
    object_position = base_T_object[:3, 3]
    direction = object_position - camera_position
    norm = float(np.linalg.norm(direction))
    if norm < 1e-5:
        raise ValueError("camera and object centers are too close")
    forward = direction / norm

    down_hint = base_T_camera[:3, 1]
    right = np.cross(down_hint, forward)
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(np.array([1.0, 0.0, 0.0]), forward)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    down /= np.linalg.norm(down)

    base_T_camera_goal = np.eye(4, dtype=float)
    base_T_camera_goal[:3, :3] = np.column_stack((right, down, forward))
    base_T_camera_goal[:3, 3] = object_position - forward * float(distance_m)
    return as_transform(base_T_camera_goal @ np.linalg.inv(tcp_T_camera))


def lifted_target(base_T_tcp_target: np.ndarray, lift_m: float) -> np.ndarray:
    """Return a copy translated upward along the robot base +Z axis."""
    if not np.isfinite(lift_m) or lift_m < 0.0:
        raise ValueError("lift distance must be finite and non-negative")
    target = as_transform(base_T_tcp_target).copy()
    target[2, 3] += float(lift_m)
    return target


def pose_reached(
    current: np.ndarray,
    target: np.ndarray,
    *,
    translation_tolerance_m: float,
    rotation_tolerance_deg: float,
) -> bool:
    """Return whether measured TCP feedback is sufficiently near a target."""
    return (
        translation_error_m(current, target) <= translation_tolerance_m
        and rotation_error_deg(current, target) <= rotation_tolerance_deg
    )


def workspace_contains(
    transform: np.ndarray,
    bounds_m: dict[str, Sequence[float]],
) -> bool:
    """Check XYZ against explicit robot-base workspace limits."""
    xyz = as_transform(transform)[:3, 3]
    for index, axis in enumerate(("x", "y", "z")):
        limits = bounds_m.get(axis)
        if limits is None or len(limits) != 2:
            raise ValueError(f"workspace axis {axis!r} needs [minimum, maximum]")
        lower, upper = map(float, limits)
        if not np.isfinite([lower, upper]).all() or lower > upper:
            raise ValueError(f"workspace axis {axis!r} has invalid limits")
        if not lower <= xyz[index] <= upper:
            return False
    return True
