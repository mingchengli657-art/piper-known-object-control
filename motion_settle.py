"""Feedback-based settling gate for the MoveJ/MoveL transition."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from grasp_geometry import as_transform, poses_stable


class MotionSettler:
    """Require stopped status and a stable TCP feedback window."""

    def __init__(
        self,
        *,
        required_samples: int = 6,
        translation_limit_m: float = 0.0015,
        rotation_limit_deg: float = 0.5,
        dwell_s: float = 0.15,
    ) -> None:
        if int(required_samples) < 2:
            raise ValueError("settle sample count must be at least two")
        values = (
            float(translation_limit_m),
            float(rotation_limit_deg),
            float(dwell_s),
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("settle limits and dwell must be finite and positive")
        self.required_samples = int(required_samples)
        self.translation_limit_m = values[0]
        self.rotation_limit_deg = values[1]
        self.dwell_s = values[2]
        self.reset()

    def reset(self) -> None:
        self._poses: list[np.ndarray] = []
        self._stable_since: float | None = None
        self.settled = False

    def observe(
        self,
        pose: Sequence[Sequence[float]],
        *,
        motion_status: int,
        now: float,
    ) -> bool:
        """Consume one fresh sample and return whether the gate is settled."""
        if not isinstance(now, (int, float)) or not math.isfinite(float(now)):
            raise ValueError("settle timestamp must be finite")
        if int(motion_status) != 0:
            self.reset()
            return False
        transform = as_transform(np.asarray(pose, dtype=float))
        if not np.isfinite(transform).all():
            self.reset()
            return False
        self._poses.append(transform)
        self._poses = self._poses[-self.required_samples :]
        if not poses_stable(
            self._poses,
            translation_limit_m=self.translation_limit_m,
            rotation_limit_deg=self.rotation_limit_deg,
        ):
            # Keep the newest sample as the beginning of a new stable window.
            self._poses = [transform]
            self._stable_since = None
            self.settled = False
            return False
        if self._stable_since is None:
            self._stable_since = float(now)
        self.settled = (
            len(self._poses) >= self.required_samples
            and float(now) - self._stable_since >= self.dwell_s
        )
        return self.settled
