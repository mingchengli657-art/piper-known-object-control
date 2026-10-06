"""Pure safety decisions for bounded Piper gripper commands."""

from dataclasses import dataclass
from enum import Enum
import math


class GripperExecutionState(Enum):
    """State of the explicit gripper command path."""

    DISARMED = 'disarmed'
    ARMED = 'armed'
    MOVING = 'moving'
    STOPPED = 'stopped'
    FAULT = 'fault'


@dataclass(frozen=True)
class GripperFeedback:
    """Validated SI feedback and fault state for one gripper sample."""

    opening_m: float
    force_n: float
    healthy: bool
    stamp: float


@dataclass(frozen=True)
class GripperIntent:
    """Hardware intent returned by the pure gripper controller."""

    action: str | None = None
    opening_m: float | None = None
    force_n: float | None = None
    error: str | None = None


@dataclass(frozen=True)
class OpeningResponseDecision:
    """Result of checking one post-command opening sample."""

    response_seen: bool = False
    error: str | None = None


class OpeningResponseMonitor:
    """Require real directional motion after an opening command.

    The AGX feedback stream may quantize or coalesce a short opening move into
    one fresh sample at the requested target.  A target-reaching sample with a
    non-zero directional step is therefore valid physical-response evidence;
    ordinary moves still use the configured multi-sample rule.
    """

    def __init__(
        self,
        *,
        start_width_m,
        target_width_m,
        minimum_travel_m=0.003,
        minimum_step_m=0.0005,
        required_samples=2,
        direction_tolerance_m=0.001,
        target_tolerance_m=0.002,
    ):
        values = (
            start_width_m,
            target_width_m,
            minimum_travel_m,
            minimum_step_m,
            direction_tolerance_m,
            target_tolerance_m,
        )
        if not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in values
        ):
            raise ValueError("opening response parameters must be finite")
        if minimum_travel_m <= 0.0 or minimum_step_m <= 0.0:
            raise ValueError("opening response distances must be positive")
        if required_samples < 1:
            raise ValueError("required opening response samples must be positive")
        self.start_width_m = float(start_width_m)
        self.target_width_m = float(target_width_m)
        self.minimum_travel_m = float(minimum_travel_m)
        self.minimum_step_m = float(minimum_step_m)
        self.required_samples = int(required_samples)
        self.direction_tolerance_m = float(direction_tolerance_m)
        self.target_tolerance_m = float(target_tolerance_m)
        delta = self.target_width_m - self.start_width_m
        self.direction = 1.0 if delta >= 0.0 else -1.0
        self.response_required = abs(delta) >= self.minimum_travel_m
        self.response_seen = not self.response_required
        self._last_counted_progress = 0.0
        self._progress_samples = 0

    def observe(self, width_m):
        """Observe a fresh width sample without issuing any command."""
        if not isinstance(width_m, (int, float)) or not math.isfinite(width_m):
            return OpeningResponseDecision(error="opening feedback is non-finite")
        width_m = float(width_m)
        progress = self.direction * (width_m - self.start_width_m)
        if progress < -self.direction_tolerance_m:
            return OpeningResponseDecision(
                error="gripper moved in the wrong direction during opening"
            )
        # Feedback can arrive much faster than the mechanism moves, making
        # every individual sample-to-sample step smaller than minimum_step_m.
        # Count progress whenever displacement from the last counted milestone
        # grows by that amount. Repeated/frozen feedback cannot add evidence.
        if progress - self._last_counted_progress >= self.minimum_step_m:
            self._progress_samples += 1
            self._last_counted_progress = progress
        reached_target_in_one_step = (
            progress >= self.minimum_travel_m
            and self._progress_samples >= 1
            and abs(self.target_width_m - width_m) <= self.target_tolerance_m
        )
        if (
            not self.response_seen
            and progress >= self.minimum_travel_m
            and (
                self._progress_samples >= self.required_samples
                or reached_target_in_one_step
            )
        ):
            self.response_seen = True
        return OpeningResponseDecision(response_seen=self.response_seen)


def close_fallback_due(*, enabled, elapsed_s, fallback_after_s):
    """Return whether the explicitly enabled test-only close fallback is due."""
    values = (elapsed_s, fallback_after_s)
    if not all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for value in values
    ):
        raise ValueError("close fallback timing must be finite")
    if fallback_after_s < 0.0:
        raise ValueError("close fallback timeout must not be negative")
    return bool(enabled) and float(elapsed_s) >= float(fallback_after_s)


class GripperExecution:
    """Gate bounded gripper targets using fresh healthy feedback."""

    def __init__(self, *, feedback_timeout_sec=0.1,
                 command_timeout_sec=5.0, opening_limit=(0.0, 0.08),
                 feedback_opening_limit=None,
                 force_limit=(0.0, 5.0), target_tolerance_m=0.001,
                 max_speed_mps=0.050, max_acceleration_mps2=0.050,
                 command_retry_interval_sec=0.50,
                 max_command_attempts=4):
        self.feedback_timeout_sec = self._positive(
            feedback_timeout_sec, 'feedback timeout')
        self.command_timeout_sec = self._positive(
            command_timeout_sec, 'command timeout')
        self.opening_limit = self._limits(opening_limit, 'opening')
        self.feedback_opening_limit = self._limits(
            opening_limit if feedback_opening_limit is None
            else feedback_opening_limit,
            'feedback opening')
        self.force_limit = self._limits(force_limit, 'force')
        self.target_tolerance_m = self._positive(
            target_tolerance_m, 'target tolerance')
        self.max_speed_mps = self._positive(max_speed_mps, 'maximum speed')
        self.max_acceleration_mps2 = self._positive(
            max_acceleration_mps2, 'maximum acceleration')
        self.command_retry_interval_sec = self._positive(
            command_retry_interval_sec, 'command retry interval')
        if not isinstance(max_command_attempts, int) or max_command_attempts < 1:
            raise ValueError('maximum command attempts must be a positive integer')
        self.max_command_attempts = max_command_attempts
        self.state = GripperExecutionState.DISARMED
        self._feedback = None
        self._target = None
        self._force = None
        self._deadline = None
        self._last_command_time = None
        self._command_attempts = 0
        self._stop_sent = False

    @staticmethod
    def _positive(value, name):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return float(value)

    @staticmethod
    def _limits(limits, name):
        if len(limits) != 2 or not all(math.isfinite(value) for value in limits):
            raise ValueError(f'{name} limits must be finite')
        lower, upper = (float(value) for value in limits)
        if lower > upper:
            raise ValueError(f'{name} lower limit must not exceed upper limit')
        return lower, upper

    @staticmethod
    def _in_range(value, limits):
        return (isinstance(value, (int, float)) and math.isfinite(value)
                and limits[0] <= value <= limits[1])

    def _fresh_and_safe(self, now):
        return (self._feedback is not None and self._feedback.healthy
                and self._in_range(
                    self._feedback.opening_m, self.feedback_opening_limit)
                and now - self._feedback.stamp <= self.feedback_timeout_sec)

    def _clear_command(self):
        self._last_command_time = None
        self._command_attempts = 0

    def observe(self, feedback):
        """Record feedback and report completion or a latched fault stop."""
        if not isinstance(feedback, GripperFeedback):
            raise TypeError('feedback must be GripperFeedback')
        if not all(math.isfinite(value) for value in (
                feedback.opening_m, feedback.force_n, feedback.stamp)):
            raise ValueError('feedback values must be finite')
        if not self._in_range(
                feedback.opening_m, self.feedback_opening_limit):
            feedback = GripperFeedback(
                feedback.opening_m, feedback.force_n, False, feedback.stamp)
        if feedback.healthy:
            self._feedback = feedback
        elif self.state is GripperExecutionState.MOVING:
            return self._fault('gripper feedback is unhealthy')
        if (self.state is GripperExecutionState.MOVING
                and abs(feedback.opening_m - self._target)
                <= self.target_tolerance_m):
            self.state = GripperExecutionState.ARMED
            self._clear_command()
            self._target = None
            self._force = None
            self._deadline = None
            return GripperIntent(action='complete')
        return GripperIntent()

    def arm(self, now):
        """Arm the gripper path after a recent healthy sample."""
        if self.state is GripperExecutionState.STOPPED:
            return GripperIntent(error='gripper controller stopped')
        if not self._fresh_and_safe(now):
            return GripperIntent(error='gripper feedback is not fresh and safe')
        self.state = GripperExecutionState.ARMED
        return GripperIntent(action='arm')

    def accept(self, opening_m, force_n, now):
        """Validate and accept one absolute opening/force target in SI units."""
        if self.state is GripperExecutionState.STOPPED:
            return GripperIntent(error='gripper controller stopped')
        if self.state in (GripperExecutionState.DISARMED,
                          GripperExecutionState.FAULT):
            return GripperIntent(error='gripper controller disarmed')
        if self.state is GripperExecutionState.MOVING:
            return GripperIntent(error='gripper goal already active')
        if not self._fresh_and_safe(now):
            return self._fault('gripper feedback is not fresh and safe')
        if not self._in_range(opening_m, self.opening_limit):
            return self._fault('gripper opening is outside the supported range')
        if not self._in_range(force_n, self.force_limit):
            return self._fault('gripper force is outside the supported range')
        self.state = GripperExecutionState.MOVING
        self._target = float(opening_m)
        self._force = float(force_n)
        self._deadline = now + self.command_timeout_sec
        # AGX 0x159 is an absolute position command and the gripper firmware
        # performs its own closed-loop motion.  Sending a software-generated
        # stream of tiny intermediate positions can repeatedly replace the
        # hardware target and made startup behaviour intermittent.  Send the
        # real target once, then retry that exact target a bounded number of
        # times only when feedback has not completed the command.
        self._last_command_time = float(now)
        self._command_attempts = 1
        self._stop_sent = False
        return GripperIntent(
            action='send_target', opening_m=self._target,
            force_n=self._force)

    def tick(self, now):
        """Stop active movement when feedback or command time expires."""
        if self.state is not GripperExecutionState.MOVING:
            return GripperIntent()
        if not self._fresh_and_safe(now):
            return self._fault('gripper feedback timed out')
        if now > self._deadline:
            return self._fault(
                'gripper command timed out after '
                f'{self._command_attempts} absolute command attempt(s)'
            )
        if (
            self._command_attempts < self.max_command_attempts
            and now - self._last_command_time >= self.command_retry_interval_sec
        ):
            self._command_attempts += 1
            self._last_command_time = float(now)
            return GripperIntent(
                action='send_target', opening_m=self._target,
                force_n=self._force)
        return GripperIntent()

    def cancel(self):
        """Stop one active command while preserving the armed state."""
        if self.state is GripperExecutionState.MOVING:
            return self._stop('gripper goal canceled')
        return GripperIntent()

    def complete_at_physical_limit(self, minimum_opening_m, now):
        """Accept a measured open stop after an external plateau check.

        The caller must first verify that several fresh samples have stopped
        changing, so a single stalled sample cannot masquerade as the open
        limit.
        """
        if self.state is not GripperExecutionState.MOVING:
            return GripperIntent(error='gripper has no active goal')
        if not self._fresh_and_safe(now):
            return self._fault('gripper feedback is not fresh and safe')
        if (not self._in_range(minimum_opening_m, self.opening_limit)
                or self._feedback.opening_m < float(minimum_opening_m)):
            return self._fault('gripper stopped below the minimum open width')
        self.state = GripperExecutionState.ARMED
        self._clear_command()
        self._target = None
        self._force = None
        self._deadline = None
        return GripperIntent(action='complete')

    def stop(self):
        """Latch a stopped state and issue one stop intent."""
        return self._stop('gripper stopped')

    def disarm(self):
        """Return an idle gripper controller to the explicit disarmed state."""
        if self.state is GripperExecutionState.MOVING:
            return self._stop('gripper disarmed')
        if self.state is GripperExecutionState.FAULT:
            return GripperIntent(error='gripper fault requires restart')
        self.state = GripperExecutionState.DISARMED
        self._clear_command()
        self._target = None
        self._force = None
        self._deadline = None
        return GripperIntent(action='disarm')

    def _stop(self, error):
        if self._stop_sent:
            return GripperIntent(error=error)
        self._stop_sent = True
        self.state = GripperExecutionState.STOPPED
        self._clear_command()
        self._target = None
        self._force = None
        self._deadline = None
        return GripperIntent(action='stop', error=error)

    def _fault(self, error):
        result = self._stop(error)
        self.state = GripperExecutionState.FAULT
        return result
