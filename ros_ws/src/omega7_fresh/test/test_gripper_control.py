from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from omega7_fresh.gripper_control import (
    GripperConfig,
    GripperIntent,
    GripperIntentMapper,
    ClosingContactDetector,
    LatestGripperIntent,
    bounded_move_target,
    prepare_grasp_intent,
    validate_action_names,
)


@pytest.fixture
def config():
    path = Path(__file__).parents[1] / "config" / "omega7_fresh.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))[
        "omega7_fresh"]["ros__parameters"]
    return GripperConfig.from_parameters(params)


def test_config_has_safe_speeds_force_and_hysteresis(config):
    assert config.move_speed == pytest.approx(0.10)
    assert config.grasp_speed == pytest.approx(0.05)
    assert config.grasp_force == pytest.approx(20.0)
    assert config.omega_engage_gap < config.omega_release_gap
    assert config.epsilon_outer > 0.0
    assert config.omega_engage_gap == pytest.approx(config.omega_full_close_gap)


def test_invalid_config_is_rejected(config):
    with pytest.raises(ValueError, match="engage < release < open"):
        replace(config, omega_release_gap=config.omega_engage_gap).validate()
    with pytest.raises(ValueError, match="Move speed"):
        replace(config, move_speed=0.11).validate()
    with pytest.raises(ValueError, match="force"):
        replace(config, grasp_force=71.0).validate()
    with pytest.raises(ValueError, match="epsilons"):
        replace(config, epsilon_outer=0.08).validate()
    with pytest.raises(ValueError, match="finite"):
        replace(config, deadband=float("nan")).validate()
    with pytest.raises(ValueError, match="Move segment"):
        replace(config, max_move_step=0.0).validate()
    with pytest.raises(ValueError, match="contact release margin"):
        replace(config, omega_contact_release_margin=0.0).validate()
    with pytest.raises(ValueError, match="contact error threshold"):
        replace(config, contact_error_threshold=0.0).validate()
    with pytest.raises(ValueError, match="contact clear error"):
        replace(config, contact_clear_error=config.contact_error_threshold).validate()


def test_action_names_must_be_nonempty():
    validate_action_names({
        "gripper_move_action": "/franka_gripper/move",
        "gripper_grasp_action": "/franka_gripper/grasp",
    })
    with pytest.raises(ValueError, match="non-empty"):
        validate_action_names({
            "gripper_move_action": "",
            "gripper_grasp_action": "/franka_gripper/grasp",
        })


def test_move_position_maps_proportionally_in_free_mode(config):
    mapper = GripperIntentMapper(config)

    assert mapper.update(config.omega_open_gap) == GripperIntent("move", 0.08)
    assert mapper.update(0.020) == GripperIntent("move", pytest.approx(0.064))


def test_grasp_engages_and_holds_mode_until_release_threshold(config):
    mapper = GripperIntentMapper(config)

    assert mapper.update(0.010).mode == "move"
    assert mapper.update(0.002).mode == "move"
    assert mapper.update(config.omega_full_close_gap).mode == "grasp"
    assert mapper.update(0.012).mode == "grasp"
    assert mapper.update(0.0149).mode == "grasp"
    assert mapper.update(0.015).mode == "move"


def test_auto_contact_release_requires_small_opening_margin(config):
    mapper = GripperIntentMapper(config)
    mapper.force_grasp_at_width(0.050, gap=0.020)

    assert mapper.update(0.0219).mode == "grasp"
    assert mapper.update(0.022).mode == "move"


def test_auto_contact_release_still_respects_minimum_release_gap(config):
    mapper = GripperIntentMapper(config)
    mapper.force_grasp_at_width(0.050, gap=0.010)

    assert mapper.update(0.0149).mode == "grasp"
    assert mapper.update(0.015).mode == "move"


def test_release_opens_past_grasp_tolerance_before_free_following(config):
    mapper = GripperIntentMapper(config)
    grasp_intent = mapper.update(config.omega_engage_gap)
    mapper.mark_grasp_succeeded(grasp_intent.width)
    mapper.update(config.omega_release_gap - 0.0001)
    release_intent = mapper.update(config.omega_release_gap)

    assert release_intent.mode == "move"
    assert release_intent.width == pytest.approx(
        max(mapper.map_width(config.omega_release_gap),
            grasp_intent.width + config.epsilon_outer + config.release_margin))
    assert mapper.update(config.omega_release_gap).width == pytest.approx(
        release_intent.width)

    # Once the analog target has opened beyond the release floor, normal Move following resumes.
    gap_at_floor = (config.omega_closed_gap
                    + release_intent.width / config.robot_open_width
                    * (config.omega_open_gap - config.omega_closed_gap))
    assert mapper.update(gap_at_floor).width == pytest.approx(release_intent.width)
    assert mapper.update(config.omega_open_gap).width == pytest.approx(0.08)


def test_release_clearance_uses_actual_wide_grasp_width(config):
    mapper = GripperIntentMapper(config)
    mapper.update(config.omega_engage_gap)
    mapper.mark_grasp_succeeded(0.060, epsilon_outer=0.048)

    released = mapper.update(config.omega_release_gap)

    assert released.mode == "move"
    assert released.width == pytest.approx(config.robot_open_width)


def test_fake_mapping_keeps_continuous_position_behavior(config):
    mapper = GripperIntentMapper(config, apply_release_margin=False)
    mapper.update(config.omega_engage_gap)

    released = mapper.update(config.omega_release_gap)

    assert released.mode == "move"
    assert released.width == pytest.approx(mapper.map_width(config.omega_release_gap))


def test_latest_intent_queue_coalesces_same_mode_updates(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("move", 0.020))
    assert queue.begin_if_ready(0.0) == GripperIntent("move", 0.020)

    queue.update(GripperIntent("move", 0.030))
    queue.update(GripperIntent("move", 0.040))
    assert not queue.needs_cancel()
    queue.complete()

    assert queue.begin_if_ready(0.10) == GripperIntent("move", 0.040)


def test_mode_change_requests_one_cancel_then_dispatches_latest(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("move", 0.060))
    queue.begin_if_ready(0.0)
    queue.update(GripperIntent("grasp", 0.030))

    assert queue.needs_cancel()
    queue.complete()
    assert queue.begin_if_ready(0.10) == GripperIntent("grasp", 0.030)


def test_deadband_suppresses_unnecessary_followup_action(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("move", 0.040))
    queue.begin_if_ready(0.0)
    queue.complete()
    queue.update(GripperIntent("move", 0.0405))

    assert queue.begin_if_ready(0.1) is None


@pytest.mark.parametrize(
    ("target", "basis", "expected"),
    [
        (0.080, 0.020, 0.030),
        (0.000, 0.060, 0.050),
        (0.025, 0.020, 0.025),
    ],
)
def test_bounded_move_target_honors_an_explicit_short_step(config, target, basis, expected):
    max_step = 0.010
    result = bounded_move_target(
        target,
        basis,
        max_step,
        config.robot_closed_width,
        config.robot_open_width,
    )

    assert result == pytest.approx(expected)
    assert abs(result - basis) <= max_step + 1.0e-12


def test_configured_full_range_move_avoids_intermediate_stop(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("move", 0.080))
    first = GripperIntent(
        "move",
        bounded_move_target(0.080, 0.000, config.max_move_step, 0.0, 0.08),
    )
    assert config.max_move_step == pytest.approx(0.080)
    assert queue.begin_if_ready(0.0, first) == GripperIntent("move", 0.080)
    queue.update(GripperIntent("move", 0.035))
    queue.complete(success=True)

    second = GripperIntent(
        "move",
        bounded_move_target(0.035, 0.080, config.max_move_step, 0.0, 0.08),
    )
    assert queue.begin_if_ready(0.1, second) == GripperIntent("move", 0.035)


def test_successful_grasp_latches_until_release_mode_transition(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("grasp", 0.040))
    assert queue.begin_if_ready(0.0) == GripperIntent("grasp", 0.040)
    queue.complete(success=True)

    queue.update(GripperIntent("grasp", 0.020))
    assert queue.begin_if_ready(0.1) is None

    queue.update(GripperIntent("move", 0.060))
    assert queue.begin_if_ready(0.1) == GripperIntent("move", 0.060)


def test_failed_grasp_can_retry_using_latest_narrower_target(config):
    queue = LatestGripperIntent(config.deadband, config.command_interval)
    queue.update(GripperIntent("grasp", 0.040))
    assert queue.begin_if_ready(0.0) == GripperIntent("grasp", 0.040)
    queue.update(GripperIntent("grasp", 0.020))
    queue.complete(success=False)

    assert queue.begin_if_ready(0.1) == GripperIntent("grasp", 0.020)


def test_stationary_active_move_then_success_never_confirms_contact(config):
    detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    detector.move_started(
        basis_width=0.0788, target_width=0.0700, overall_target_width=0.0075)

    # A 1.5 s active action at 78.8 mm was a real slow segment, not proof of contact.
    # The detector has no active-action timer or joint-state polling path; only this
    # successful terminal result can clear the candidate.
    assert detector.pending
    assert detector.width_error(0.0788) == pytest.approx(0.0088)
    assert detector.width_error(0.0788) <= config.max_move_step
    contact, error = detector.move_result(
        failed=False, intentional_cancel=False, measured_width=0.0788,
        state_fresh=True, closing_intent_active=True)

    assert not contact
    assert error == pytest.approx(0.0088)
    assert not detector.pending


def test_failed_move_with_fresh_large_error_uses_exact_accepted_segment(config):
    detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    detector.move_started(
        basis_width=0.0788, target_width=0.0700, overall_target_width=0.0075)

    # The much narrower overall target is for the next segment, not this one.
    assert detector.target_width == pytest.approx(0.0700)
    contact, error = detector.move_result(
        failed=True, intentional_cancel=False, measured_width=0.0788,
        state_fresh=True, closing_intent_active=True)

    assert contact
    assert error == pytest.approx(0.0088)
    assert not detector.pending


def test_normal_progress_and_small_failed_error_do_not_confirm_contact(config):
    detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    detector.move_started(0.080, 0.070, overall_target_width=0.020)

    contact, error = detector.move_result(
        failed=False, intentional_cancel=False, measured_width=0.070,
        state_fresh=True, closing_intent_active=True)
    assert not contact
    assert error == pytest.approx(0.0)

    detector.move_started(0.080, 0.070, overall_target_width=0.020)
    contact, error = detector.move_result(
        failed=True, intentional_cancel=False, measured_width=0.0755,
        state_fresh=True, closing_intent_active=True)
    assert not contact
    assert error == pytest.approx(0.0055)


def test_intentional_cancel_or_reversal_never_confirms_contact(config):
    canceled = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    canceled.move_started(0.080, 0.070, overall_target_width=0.020)
    contact, _ = canceled.move_result(
        failed=True, intentional_cancel=True, measured_width=0.080,
        state_fresh=True, closing_intent_active=False)
    assert not contact

    reversed_detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    reversed_detector.move_started(0.080, 0.070, overall_target_width=0.018)
    assert reversed_detector.update_intent(0.022, config.deadband)
    assert not reversed_detector.pending
    contact, _ = reversed_detector.move_result(
        failed=True, intentional_cancel=False, measured_width=0.080,
        state_fresh=True, closing_intent_active=False)
    assert not contact


def test_stale_measurement_cannot_confirm_contact(config):
    detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    detector.move_started(0.080, 0.070, overall_target_width=0.020)
    contact, error = detector.move_result(
        failed=True, intentional_cancel=False, measured_width=0.080,
        state_fresh=False, closing_intent_active=True)

    assert not contact
    assert error is None
    assert not detector.pending


def test_successful_move_resets_before_next_move_uses_latest_target(config):
    detector = ClosingContactDetector(
        config.contact_error_threshold, config.contact_clear_error)
    detector.move_started(
        basis_width=0.0788, target_width=0.0700, overall_target_width=0.0075)
    contact, _ = detector.move_result(
        failed=False, intentional_cancel=False, measured_width=0.069,
        state_fresh=True, closing_intent_active=True)
    assert not contact
    assert not detector.pending

    fresh_basis = 0.069
    next_target = bounded_move_target(
        0.0075, fresh_basis, config.max_move_step,
        config.robot_closed_width, config.robot_open_width)
    detector.move_started(fresh_basis, next_target, overall_target_width=0.0075)
    assert detector.target_width == pytest.approx(0.0075)


@pytest.mark.parametrize("contact_width", [0.018, 0.045, 0.072])
def test_confirmed_contact_uses_measured_width_and_width_robust_tolerance(config, contact_width):
    mapper = GripperIntentMapper(config)
    mapper.force_grasp_at_width(contact_width, gap=0.008)
    grasp, epsilon_inner, epsilon_outer = prepare_grasp_intent(
        GripperIntent("grasp", config.robot_closed_width),
        config,
        contact_width=contact_width,
    )

    assert grasp.width == pytest.approx(contact_width)
    assert epsilon_inner <= contact_width - config.robot_closed_width
    assert contact_width + epsilon_outer >= config.robot_open_width
    assert mapper.update(0.014).mode == "grasp"


@pytest.mark.parametrize("object_width", [0.018, 0.045, 0.072])
def test_full_close_always_maps_to_grasp_with_fallback_for_unknown_width(config, object_width):
    mapper = GripperIntentMapper(config)
    intent = mapper.update(config.omega_full_close_gap)
    goal, epsilon_inner, epsilon_outer = prepare_grasp_intent(
        intent, config, contact_width=None, full_close=True)

    assert intent.mode == "grasp"
    assert goal.width == config.robot_closed_width
    assert object_width <= goal.width + epsilon_outer
    assert epsilon_inner + epsilon_outer <= (
        config.robot_open_width - config.robot_closed_width)


def test_full_close_uses_force_grasp_even_when_contact_feedback_is_unavailable(config):
    mapper = GripperIntentMapper(config)
    intent = mapper.update(0.0)
    goal, epsilon_inner, epsilon_outer = prepare_grasp_intent(
        intent, config, contact_width=None, full_close=True)

    assert goal.mode == GripperIntentMapper.GRASP
    assert goal.width == 0.0
    assert epsilon_inner == 0.0
    assert epsilon_outer == config.robot_open_width
