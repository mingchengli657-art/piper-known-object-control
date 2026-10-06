from vendor.piper_ros2_control.gripper_execution import (
    close_fallback_due,
    GripperExecution,
    GripperExecutionState,
    GripperFeedback,
    OpeningResponseMonitor,
)


def test_unverified_close_fallback_is_disabled_by_default():
    assert not close_fallback_due(
        enabled=False, elapsed_s=10.0, fallback_after_s=3.0
    )


def test_unverified_close_fallback_requires_explicit_test_switch():
    assert close_fallback_due(
        enabled=True, elapsed_s=3.0, fallback_after_s=3.0
    )


def test_physical_open_limit_can_complete_a_maximum_command():
    controller = GripperExecution(feedback_timeout_sec=1.0)
    controller.observe(GripperFeedback(0.040, 0.0, True, 1.0))
    assert controller.arm(1.0).error is None
    assert controller.accept(0.080, 1.0, 1.0).error is None
    controller.observe(GripperFeedback(0.0691, 0.0, True, 1.5))
    result = controller.complete_at_physical_limit(0.065, 1.5)
    assert result.action == "complete"
    assert controller.state is GripperExecutionState.ARMED


def test_physical_open_limit_rejects_a_low_stall():
    controller = GripperExecution(feedback_timeout_sec=1.0)
    controller.observe(GripperFeedback(0.040, 0.0, True, 1.0))
    controller.arm(1.0)
    controller.accept(0.080, 1.0, 1.0)
    controller.observe(GripperFeedback(0.050, 0.0, True, 1.5))
    result = controller.complete_at_physical_limit(0.065, 1.5)
    assert result.error == "gripper stopped below the minimum open width"
    assert controller.state is GripperExecutionState.FAULT


def test_feedback_tolerance_is_separate_from_command_limits():
    controller = GripperExecution(
        feedback_timeout_sec=1.0,
        opening_limit=(0.0, 0.1),
        feedback_opening_limit=(-0.0015, 0.1015),
    )
    controller.observe(GripperFeedback(-0.0004, 0.0, True, 1.0))
    assert controller.arm(1.0).error is None
    assert controller.accept(-0.0004, 1.0, 1.0).error == (
        "gripper opening is outside the supported range"
    )


def test_absolute_target_is_sent_immediately_not_profile_start():
    controller = GripperExecution(
        feedback_timeout_sec=1.0,
        opening_limit=(0.0, 0.1),
    )
    controller.observe(GripperFeedback(0.100, 0.0, True, 1.0))
    assert controller.arm(1.0).error is None
    intent = controller.accept(0.080, 1.0, 1.0)
    assert intent.action == "send_target"
    assert intent.opening_m == 0.080
    assert intent.force_n == 1.0


def test_absolute_target_retries_are_bounded_and_identical():
    controller = GripperExecution(
        feedback_timeout_sec=1.0,
        command_timeout_sec=3.0,
        opening_limit=(0.0, 0.1),
        command_retry_interval_sec=0.5,
        max_command_attempts=3,
    )
    controller.observe(GripperFeedback(0.100, 0.0, True, 1.0))
    controller.arm(1.0)
    first = controller.accept(0.080, 1.0, 1.0)
    assert first.opening_m == 0.080

    controller.observe(GripperFeedback(0.100, 0.0, True, 1.49))
    assert controller.tick(1.49).action is None
    controller.observe(GripperFeedback(0.100, 0.0, True, 1.50))
    second = controller.tick(1.50)
    assert second.action == "send_target"
    assert second.opening_m == first.opening_m
    controller.observe(GripperFeedback(0.100, 0.0, True, 2.00))
    third = controller.tick(2.00)
    assert third.action == "send_target"
    assert third.opening_m == first.opening_m
    controller.observe(GripperFeedback(0.100, 0.0, True, 2.50))
    assert controller.tick(2.50).action is None


def test_feedback_completion_stops_absolute_target_retries():
    controller = GripperExecution(
        feedback_timeout_sec=1.0,
        opening_limit=(0.0, 0.1),
        command_retry_interval_sec=0.5,
    )
    controller.observe(GripperFeedback(0.100, 0.0, True, 1.0))
    controller.arm(1.0)
    controller.accept(0.080, 1.0, 1.0)
    completed = controller.observe(GripperFeedback(0.0805, 0.0, True, 1.2))
    assert completed.action == "complete"
    assert controller.state is GripperExecutionState.ARMED
    assert controller.tick(2.0).action is None


def test_opening_response_requires_real_multi_sample_progress():
    monitor = OpeningResponseMonitor(
        start_width_m=0.080,
        target_width_m=0.100,
        minimum_travel_m=0.003,
        minimum_step_m=0.0005,
        required_samples=2,
    )
    assert not monitor.observe(0.0834).response_seen
    assert not monitor.observe(0.0834).response_seen
    assert not monitor.response_seen
    assert monitor.observe(0.0840).response_seen


def test_opening_response_accepts_one_fresh_target_reaching_step():
    """Some AGX updates coalesce the whole short move into one sample."""
    monitor = OpeningResponseMonitor(
        start_width_m=0.080,
        target_width_m=0.095,
        minimum_travel_m=0.003,
        minimum_step_m=0.0005,
        required_samples=2,
        target_tolerance_m=0.002,
    )
    assert monitor.observe(0.095).response_seen


def test_opening_response_counts_cumulative_high_rate_progress():
    monitor = OpeningResponseMonitor(
        start_width_m=0.080,
        target_width_m=0.095,
        minimum_travel_m=0.003,
        minimum_step_m=0.0005,
        required_samples=2,
    )
    assert not monitor.observe(0.0803).response_seen
    assert not monitor.observe(0.0806).response_seen
    assert not monitor.observe(0.0809).response_seen
    assert not monitor.observe(0.0812).response_seen
    assert monitor.observe(0.0831).response_seen


def test_opening_response_rejects_wrong_direction():
    monitor = OpeningResponseMonitor(
        start_width_m=0.080,
        target_width_m=0.100,
    )
    decision = monitor.observe(0.0785)
    assert "wrong direction" in decision.error


def test_opening_response_does_not_require_motion_when_already_at_target():
    monitor = OpeningResponseMonitor(
        start_width_m=0.0997,
        target_width_m=0.100,
    )
    assert not monitor.response_required
    assert monitor.observe(0.0997).response_seen
