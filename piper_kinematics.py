"""Piper X forward/inverse kinematics used for grasp preflight.

The robot geometry and joint limits are loaded from the installed
``agx_arm_description`` URDF.  No command is sent by this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from grasp_geometry import as_transform, workspace_contains


@dataclass(frozen=True)
class IKResult:
    joints: np.ndarray
    position_error_m: float
    rotation_error_deg: float
    minimum_margin_deg: float


@dataclass(frozen=True)
class GraspMotionPlan:
    pregrasp_joints: np.ndarray
    grasp_joints: np.ndarray
    lift_joints: np.ndarray
    minimum_margin_deg: float
    minimum_tcp_z_m: float


class PiperKinematics:
    """Numerical kinematics for the six revolute Piper X arm joints."""

    def __init__(
        self,
        names: Sequence[str],
        origins_xyz: np.ndarray,
        origins_rpy: np.ndarray,
        axes: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        self.names = tuple(names)
        self.origins_xyz = np.asarray(origins_xyz, dtype=float).reshape(-1, 3)
        self.origins_rpy = np.asarray(origins_rpy, dtype=float).reshape(-1, 3)
        self.axes = np.asarray(axes, dtype=float).reshape(-1, 3)
        self.lower = np.asarray(lower, dtype=float).reshape(-1)
        self.upper = np.asarray(upper, dtype=float).reshape(-1)
        if len(self.names) != 6:
            raise ValueError("Piper X kinematics requires exactly six joints")
        if not np.isfinite(
            np.concatenate(
                (
                    self.origins_xyz.ravel(),
                    self.origins_rpy.ravel(),
                    self.axes.ravel(),
                    self.lower,
                    self.upper,
                )
            )
        ).all():
            raise ValueError("URDF kinematics contains non-finite values")
        norms = np.linalg.norm(self.axes, axis=1)
        if np.any(norms < 1e-9) or np.any(self.lower >= self.upper):
            raise ValueError("URDF kinematics contains invalid axes or limits")
        self.axes = self.axes / norms[:, None]

    @classmethod
    def from_urdf(cls, path: Path, names: Sequence[str]) -> "PiperKinematics":
        root = ET.parse(path).getroot()
        xyz_values = []
        rpy_values = []
        axes = []
        lower = []
        upper = []
        joints_by_name = {
            joint.attrib.get("name", ""): joint for joint in root.findall("joint")
        }
        for name in names:
            if name not in joints_by_name:
                raise ValueError(f"joint {name!r} is absent from {path}")
            joint = joints_by_name[name]
            origin = joint.find("origin")
            axis = joint.find("axis")
            limit = joint.find("limit")
            if origin is None or axis is None or limit is None:
                raise ValueError(f"joint {name!r} has incomplete URDF data")
            xyz_values.append(_vector(origin.attrib.get("xyz", "0 0 0")))
            rpy_values.append(_vector(origin.attrib.get("rpy", "0 0 0")))
            axes.append(_vector(axis.attrib.get("xyz", "0 0 1")))
            lower.append(float(limit.attrib["lower"]))
            upper.append(float(limit.attrib["upper"]))
        return cls(names, xyz_values, rpy_values, axes, lower, upper)

    def forward(self, joints: Sequence[float]) -> np.ndarray:
        q = np.asarray(joints, dtype=float).reshape(6)
        if not np.isfinite(q).all():
            raise ValueError("joint vector contains non-finite values")
        transform = np.eye(4, dtype=float)
        for angle, xyz, rpy, axis in zip(
            q, self.origins_xyz, self.origins_rpy, self.axes
        ):
            origin = np.eye(4, dtype=float)
            origin[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            origin[:3, 3] = xyz
            rotation = np.eye(4, dtype=float)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            transform = transform @ origin @ rotation
        return as_transform(transform)

    def solve(
        self,
        target: np.ndarray,
        seed: Sequence[float],
        *,
        minimum_margin_deg: float,
        position_tolerance_m: float,
        rotation_tolerance_deg: float,
        restarts: int,
    ) -> IKResult:
        target = as_transform(target)
        seed = np.asarray(seed, dtype=float).reshape(6)
        margin = np.deg2rad(float(minimum_margin_deg))
        lower = self.lower + margin
        upper = self.upper - margin
        if np.any(lower >= upper):
            raise ValueError("configured joint margin leaves no valid range")
        clipped_seed = np.clip(seed, lower + 1e-7, upper - 1e-7)
        candidate_seeds = [clipped_seed, (lower + upper) * 0.5]
        rng = np.random.default_rng(19)
        candidate_seeds.extend(
            rng.uniform(lower + 1e-6, upper - 1e-6, size=6)
            for _ in range(max(0, int(restarts)))
        )

        valid: list[IKResult] = []
        for initial in candidate_seeds:
            result = least_squares(
                lambda joints: self._residual(joints, target),
                initial,
                bounds=(lower, upper),
                max_nfev=1800,
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
            )
            actual = self.forward(result.x)
            position_error = float(
                np.linalg.norm(actual[:3, 3] - target[:3, 3])
            )
            rotation_error = float(
                np.degrees(
                    np.linalg.norm(
                        Rotation.from_matrix(
                            actual[:3, :3].T @ target[:3, :3]
                        ).as_rotvec()
                    )
                )
            )
            if (
                position_error <= float(position_tolerance_m)
                and rotation_error <= float(rotation_tolerance_deg)
            ):
                margin_deg = float(
                    np.degrees(
                        np.min(np.minimum(result.x - self.lower, self.upper - result.x))
                    )
                )
                valid.append(
                    IKResult(
                        joints=result.x.copy(),
                        position_error_m=position_error,
                        rotation_error_deg=rotation_error,
                        minimum_margin_deg=margin_deg,
                    )
                )
        if not valid:
            raise ValueError(
                "no Piper X IK solution satisfies the configured error and joint-margin limits"
            )
        scale = self.upper - self.lower
        return min(
            valid,
            key=lambda item: float(np.linalg.norm((item.joints - seed) / scale)),
        )

    def solve_cartesian_path(
        self,
        start: np.ndarray,
        end: np.ndarray,
        start_joints: Sequence[float],
        *,
        samples: int,
        minimum_margin_deg: float,
        position_tolerance_m: float,
        rotation_tolerance_deg: float,
        restarts: int,
        maximum_joint_step_deg: float,
    ) -> tuple[np.ndarray, float]:
        start = as_transform(start)
        end = as_transform(end)
        q = np.asarray(start_joints, dtype=float).reshape(6)
        sample_count = max(2, int(samples))
        rotations = Rotation.from_matrix(
            np.stack((start[:3, :3], end[:3, :3]), axis=0)
        )
        interpolation = Slerp([0.0, 1.0], rotations)
        minimum_margin = float("inf")
        for fraction in np.linspace(0.0, 1.0, sample_count + 1)[1:]:
            target = np.eye(4, dtype=float)
            target[:3, :3] = interpolation(float(fraction)).as_matrix()
            target[:3, 3] = (
                start[:3, 3] * (1.0 - fraction) + end[:3, 3] * fraction
            )
            solution = self.solve(
                target,
                q,
                minimum_margin_deg=minimum_margin_deg,
                position_tolerance_m=position_tolerance_m,
                rotation_tolerance_deg=rotation_tolerance_deg,
                restarts=restarts,
            )
            step_deg = float(np.max(np.abs(np.degrees(solution.joints - q))))
            if step_deg > float(maximum_joint_step_deg):
                raise ValueError(
                    f"Cartesian IK branch jump is {step_deg:.1f} deg, above "
                    f"{float(maximum_joint_step_deg):.1f} deg"
                )
            q = solution.joints
            minimum_margin = min(minimum_margin, solution.minimum_margin_deg)
        return q.copy(), minimum_margin

    def validate_joint_path(
        self,
        start_joints: Sequence[float],
        end_joints: Sequence[float],
        *,
        samples: int,
        workspace_m: dict[str, Sequence[float]],
        start_limit_tolerance_deg: float,
    ) -> float:
        start = np.asarray(start_joints, dtype=float).reshape(6)
        end = np.asarray(end_joints, dtype=float).reshape(6)
        tolerance = np.deg2rad(float(start_limit_tolerance_deg))
        if np.any(start < self.lower - tolerance) or np.any(start > self.upper + tolerance):
            raise ValueError("current joints exceed URDF limits beyond start tolerance")
        minimum_z = float("inf")
        for fraction in np.linspace(0.0, 1.0, max(2, int(samples)) + 1):
            transform = self.forward(start * (1.0 - fraction) + end * fraction)
            if not workspace_contains(transform, workspace_m):
                xyz = np.round(transform[:3, 3], 4).tolist()
                raise ValueError(
                    f"MoveJ interpolation leaves configured TCP workspace at {xyz}"
                )
            minimum_z = min(minimum_z, float(transform[2, 3]))
        return minimum_z

    def plan_grasp_motion(
        self,
        current_joints: Sequence[float],
        pregrasp: np.ndarray,
        grasp: np.ndarray,
        lift: np.ndarray,
        planning: dict,
        workspace_m: dict[str, Sequence[float]],
    ) -> GraspMotionPlan:
        common = dict(
            minimum_margin_deg=float(planning["minimum_joint_margin_deg"]),
            position_tolerance_m=float(planning["ik_position_tolerance_m"]),
            rotation_tolerance_deg=float(planning["ik_rotation_tolerance_deg"]),
            restarts=int(planning["ik_restarts"]),
        )
        current = np.asarray(current_joints, dtype=float).reshape(6)
        pre = self.solve(pregrasp, current, **common)
        maximum_move = float(
            np.max(np.abs(np.degrees(pre.joints - current)))
        )
        if maximum_move > float(planning["maximum_initial_joint_move_deg"]):
            raise ValueError(
                f"initial MoveJ requires {maximum_move:.1f} deg on one joint, above limit"
            )
        minimum_z = self.validate_joint_path(
            current,
            pre.joints,
            samples=int(planning["joint_path_samples"]),
            workspace_m=workspace_m,
            start_limit_tolerance_deg=float(
                planning["current_joint_limit_tolerance_deg"]
            ),
        )
        cartesian_common = dict(
            samples=int(planning["cartesian_path_samples"]),
            maximum_joint_step_deg=float(planning["maximum_cartesian_joint_step_deg"]),
            **common,
        )
        grasp_joints, approach_margin = self.solve_cartesian_path(
            pregrasp, grasp, pre.joints, **cartesian_common
        )
        lift_joints, lift_margin = self.solve_cartesian_path(
            grasp, lift, grasp_joints, **cartesian_common
        )
        return GraspMotionPlan(
            pregrasp_joints=pre.joints,
            grasp_joints=grasp_joints,
            lift_joints=lift_joints,
            minimum_margin_deg=min(
                pre.minimum_margin_deg, approach_margin, lift_margin
            ),
            minimum_tcp_z_m=minimum_z,
        )

    def _residual(self, joints: np.ndarray, target: np.ndarray) -> np.ndarray:
        actual = self.forward(joints)
        translation = (actual[:3, 3] - target[:3, 3]) * 10.0
        rotation = Rotation.from_matrix(
            actual[:3, :3].T @ target[:3, :3]
        ).as_rotvec()
        return np.concatenate((translation, rotation))


def _vector(text: str) -> np.ndarray:
    value = np.fromstring(text, sep=" ", dtype=float)
    if value.shape != (3,):
        raise ValueError(f"expected three-vector, got {text!r}")
    return value
