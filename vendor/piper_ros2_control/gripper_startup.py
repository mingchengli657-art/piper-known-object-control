"""Pure startup validation for the calibrated AGX gripper."""

from dataclasses import dataclass
from enum import Enum
import math


class GripperStartupState(Enum):
    """State of the one-shot startup normalization check."""

    WAITING = "waiting"
    NORMALIZING = "normalizing"
    READY = "ready"
    FAULT = "fault"


@dataclass(frozen=True)
class GripperStartupFeedback:
    """One locally timestamped gripper sample in SI units."""

    opening_m: float
    force_n: float
    healthy: bool
    homed: bool
    stamp: float


@dataclass(frozen=True)
class GripperStartupDecision:
    """Side-effect-free decision consumed by the ROS node."""

    action: str | None = None
    target_m: float | None = None
    error: str | None = None


class GripperStartupPreflight:
    """Require calibrated, directional feedback before grasp execution."""

    def __init__(
        self,
        *,
        feedback_limit_m,
        safe_width_m,
        feedback_timeout_s,
        timeout_s,
        target_tolerance_m,
        minimum_response_m,
        direction_tolerance_m,
        settle_dwell_s,
        homing_required,
    ):
        self.feedback_limit_m = self._limits(feedback_limit_m)
        self.safe_width_m = self._finite(safe_width_m, "safe width")
        if not self._in_range(self.safe_width_m, self.feedback_limit_m):
            raise ValueError("safe width must be inside calibrated feedback limits")
        self.feedback_timeout_s = self._positive(
            feedback_timeout_s, "feedback timeout"
        )
        self.timeout_s = self._positive(timeout_s, "normalization timeout")
        self.target_tolerance_m = self._positive(
            target_tolerance_m, "target tolerance"
        )
        self.minimum_response_m = self._positive(
            minimum_response_m, "minimum response"
        )
        self.direction_tolerance_m = self._positive(
            direction_tolerance_m, "direction tolerance"
        )
        self.settle_dwell_s = self._positive(settle_dwell_s, "settle dwell")
        self.homing_required = bool(homing_required)
        self.state = GripperStartupState.WAITING
        self._feedback = None
        self._started_at = None
        self._start_opening = None
        self._direction = 0.0
        self._response_required = False
        self._response_seen = False
        self._target_since = None

    @staticmethod
    def _finite(value, name):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return float(value)

    @classmethod
    def _positive(cls, value, name):
        value = cls._finite(value, name)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    @classmethod
    def _limits(cls, values):
        if len(values) != 2:
            raise ValueError("feedback limits require two values")
        lower = cls._finite(values[0], "feedback lower limit")
        upper = cls._finite(values[1], "feedback upper limit")
        if lower > upper:
            raise ValueError("feedback lower limit must not exceed upper limit")
        return lower, upper

    @staticmethod
    def _in_range(value, limits):
        return limits[0] <= value <= limits[1]

    def _validate(self, feedback, now):
        if feedback is None:
            return "waiting for gripper feedback"
        if not isinstance(feedback, GripperStartupFeedback):
            return "invalid gripper feedback object"
        if not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in (feedback.opening_m, feedback.force_n, feedback.stamp)
        ):
            return "gripper feedback contains a non-finite value"
        age = float(now) - float(feedback.stamp)
        if age < -0.1 or age > self.feedback_timeout_s:
            return f"gripper feedback is stale ({age:.3f}s)"
        if not feedback.healthy:
            return "gripper reports disabled or a driver fault"
        if self.homing_required and not feedback.homed:
            return "gripper homing status is required but false"
        if not self._in_range(float(feedback.opening_m), self.feedback_limit_m):
            return (
                "gripper width is outside calibrated feedback range: "
                f"{float(feedback.opening_m):.6f}m not in "
                f"[{self.feedback_limit_m[0]:.6f}, "
                f"{self.feedback_limit_m[1]:.6f}]m"
            )
        return None

    def observe(self, feedback, now):
        """Record a new sample and evaluate active normalization."""
        self._feedback = feedback
        if self.state is not GripperStartupState.NORMALIZING:
            return GripperStartupDecision()
        error = self._validate(feedback, now)
        if error:
            return self._fault(error)

        opening = float(feedback.opening_m)
        progress = self._direction * (opening - self._start_opening)
        if progress < -self.direction_tolerance_m:
            return self._fault(
                "gripper moved in the wrong direction during startup "
                f"normalization: start={self._start_opening:.6f}m, "
                f"current={opening:.6f}m, target={self.safe_width_m:.6f}m, "
                f"signed_progress={progress:.6f}m"
            )
        if not self._response_required or progress >= self.minimum_response_m:
            self._response_seen = True

        if abs(opening - self.safe_width_m) <= self.target_tolerance_m:
            if self._target_since is None:
                self._target_since = float(now)
            elif (
                self._response_seen
                and float(now) - self._target_since >= self.settle_dwell_s
            ):
                self.state = GripperStartupState.READY
                return GripperStartupDecision(action="ready")
        else:
            self._target_since = None
        return GripperStartupDecision()

    def begin(self, now):
        """Validate the latest sample and request at most one safe target."""
        if self.state is not GripperStartupState.WAITING:
            return GripperStartupDecision(error="gripper startup check already began")
        error = self._validate(self._feedback, now)
        if error:
            return self._fault(error)

        self.state = GripperStartupState.NORMALIZING
        self._started_at = float(now)
        self._start_opening = float(self._feedback.opening_m)
        delta = self.safe_width_m - self._start_opening
        self._direction = 1.0 if delta >= 0.0 else -1.0
        self._response_required = abs(delta) >= self.minimum_response_m
        self._response_seen = not self._response_required
        if abs(delta) <= self.target_tolerance_m:
            self._target_since = float(now)
            return GripperStartupDecision(action="monitor")
        return GripperStartupDecision(
            action="normalize", target_m=self.safe_width_m
        )

    def tick(self, now):
        """Reject stale or non-responsive normalization attempts."""
        if self.state is GripperStartupState.READY:
            return GripperStartupDecision(action="ready")
        if self.state is GripperStartupState.FAULT:
            return GripperStartupDecision(error="gripper startup check faulted")
        if self.state is not GripperStartupState.NORMALIZING:
            return GripperStartupDecision()
        error = self._validate(self._feedback, now)
        if error:
            return self._fault(error)
        if float(now) - self._started_at > self.timeout_s:
            detail = (
                " without minimum feedback response"
                if not self._response_seen
                else " before reaching the safe width"
            )
            return self._fault("gripper startup normalization timed out" + detail)
        return GripperStartupDecision()

    def _fault(self, error):
        self.state = GripperStartupState.FAULT
        return GripperStartupDecision(error=error)
