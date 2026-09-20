"""Pure Omega gripper intent mapping and latest-only action scheduling."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GripperConfig:
    omega_closed_gap: float
    omega_open_gap: float
    omega_engage_gap: float
    omega_release_gap: float
    omega_full_close_gap: float
    omega_contact_release_margin: float
    robot_closed_width: float
    robot_open_width: float
    max_move_step: float
    move_speed: float
    grasp_speed: float
    grasp_force: float
    epsilon_inner: float
    epsilon_outer: float
    release_margin: float
    contact_error_threshold: float
    contact_clear_error: float
    deadband: float
    command_interval: float

    @classmethod
    def from_parameters(cls, params):
        config = cls(
            omega_closed_gap=float(params["omega_gripper_closed_gap"]),
            omega_open_gap=float(params["omega_gripper_open_gap"]),
            omega_engage_gap=float(params["omega_gripper_engage_gap"]),
            omega_release_gap=float(params["omega_gripper_release_gap"]),
            omega_full_close_gap=float(params["omega_gripper_full_close_gap"]),
            omega_contact_release_margin=float(params["omega_gripper_contact_release_margin"]),
            robot_closed_width=float(params["robot_gripper_closed_width"]),
            robot_open_width=float(params["robot_gripper_open_width"]),
            max_move_step=float(params["gripper_max_move_step"]),
            move_speed=float(params["gripper_speed"]),
            grasp_speed=float(params["gripper_grasp_speed"]),
            grasp_force=float(params["gripper_grasp_force"]),
            epsilon_inner=float(params["gripper_grasp_epsilon_inner"]),
            epsilon_outer=float(params["gripper_grasp_epsilon_outer"]),
            release_margin=float(params["gripper_release_margin"]),
            contact_error_threshold=float(params["gripper_contact_error_threshold"]),
            contact_clear_error=float(params["gripper_contact_clear_error"]),
            deadband=float(params["gripper_deadband"]),
            command_interval=float(params["gripper_command_interval"]),
        )
        config.validate()
        return config

    def validate(self):
        values = vars(self)
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")

        if not 0.0 <= self.omega_closed_gap < self.omega_engage_gap:
            raise ValueError("Omega closed gap must be below the engage gap")
        if not self.omega_engage_gap < self.omega_release_gap < self.omega_open_gap:
            raise ValueError("Omega gaps must satisfy engage < release < open")
        if not self.omega_closed_gap <= self.omega_full_close_gap <= self.omega_engage_gap:
            raise ValueError("full-close gap must be between the closed and engage gaps")
        if not 0.0 < self.omega_contact_release_margin <= self.omega_open_gap:
            raise ValueError("contact release margin must be positive and fit within the Omega gap range")
        if not 0.0 <= self.robot_closed_width < self.robot_open_width <= 0.08:
            raise ValueError("robot gripper widths must satisfy 0 <= closed < open <= 0.08 m")
        if not 0.0 < self.max_move_step <= self.robot_open_width - self.robot_closed_width:
            raise ValueError("gripper Move segment must be positive and fit within the width range")
        if not 0.0 < self.move_speed <= 0.1:
            raise ValueError("gripper Move speed must be in (0, 0.1] m/s")
        if not 0.0 < self.grasp_speed <= 0.1:
            raise ValueError("gripper Grasp speed must be in (0, 0.1] m/s")
        if not 0.0 < self.grasp_force <= 70.0:
            raise ValueError("gripper grasp force must be in (0, 70] N")
        if (self.epsilon_inner < 0.0 or self.epsilon_outer < 0.0
                or self.epsilon_inner + self.epsilon_outer
                > self.robot_open_width - self.robot_closed_width):
            raise ValueError("grasp epsilons must be nonnegative and fit within the gripper width")
        if not 0.0 <= self.release_margin <= self.robot_open_width:
            raise ValueError("gripper release margin must be between 0 and the open width")
        if not 0.0 < self.contact_error_threshold <= self.robot_open_width - self.robot_closed_width:
            raise ValueError("contact error threshold must be positive and fit within the gripper width")
        if not 0.0 <= self.contact_clear_error < self.contact_error_threshold:
            raise ValueError("contact clear error must be nonnegative and below the contact threshold")
        if not 0.0 < self.deadband <= self.robot_open_width - self.robot_closed_width:
            raise ValueError("gripper deadband must be positive and no larger than the width range")
        if not 0.0 <= self.command_interval <= 1.0:
            raise ValueError("gripper command interval must be between 0 and 1 second")


def validate_action_names(params):
    for name in ("gripper_move_action", "gripper_grasp_action"):
        value = params[name]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a non-empty ROS action name")


@dataclass(frozen=True)
class GripperIntent:
    mode: str
    width: float


class GripperIntentMapper:
    """Map analog gap to width and latch Move/Grasp intent with hysteresis."""

    MOVE = "move"
    GRASP = "grasp"

    def __init__(self, config, apply_release_margin=True):
        self.config = config
        self.apply_release_margin = apply_release_margin
        self.mode = self.MOVE
        self.release_floor = None
        self.held_width = None
        self.held_epsilon_outer = None
        self.contact_release_gap = None
        self.last_width = config.robot_open_width

    def map_width(self, gap):
        gap = min(max(float(gap), self.config.omega_closed_gap), self.config.omega_open_gap)
        fraction = ((gap - self.config.omega_closed_gap)
                    / (self.config.omega_open_gap - self.config.omega_closed_gap))
        return (self.config.robot_closed_width
                + fraction * (self.config.robot_open_width - self.config.robot_closed_width))

    def update(self, gap):
        gap = float(gap)
        if not math.isfinite(gap):
            raise ValueError("Omega gripper gap must be finite")

        target_width = self.map_width(gap)
        if self.mode == self.MOVE and gap <= self.config.omega_engage_gap:
            self.mode = self.GRASP
            self.release_floor = None
            self.held_width = None
            self.held_epsilon_outer = None
            self.contact_release_gap = None
        elif (self.mode == self.GRASP
              and gap >= (self.contact_release_gap or self.config.omega_release_gap)):
            if self.apply_release_margin:
                grasp_width = self.held_width if self.held_width is not None else self.last_width
                grasp_epsilon_outer = (self.held_epsilon_outer
                                       if self.held_epsilon_outer is not None
                                       else self.config.epsilon_outer)
                self.release_floor = min(
                    self.config.robot_open_width,
                    grasp_width + grasp_epsilon_outer + self.config.release_margin,
                )
            self.mode = self.MOVE
            self.held_width = None
            self.held_epsilon_outer = None
            self.contact_release_gap = None

        if self.mode == self.MOVE and self.release_floor is not None:
            target_width = max(target_width, self.release_floor)
            if self.map_width(gap) >= self.release_floor:
                self.release_floor = None

        self.last_width = target_width
        return GripperIntent(self.mode, target_width)

    def mark_grasp_succeeded(self, width, epsilon_outer=None):
        width = float(width)
        if not math.isfinite(width):
            raise ValueError("successful grasp width must be finite")
        self.held_width = min(max(
            width, self.config.robot_closed_width), self.config.robot_open_width)
        if epsilon_outer is not None and not math.isfinite(float(epsilon_outer)):
            raise ValueError("successful grasp epsilon must be finite")
        self.held_epsilon_outer = (None if epsilon_outer is None else
                                   min(max(float(epsilon_outer), 0.0),
                                       self.config.robot_open_width
                                       - self.config.robot_closed_width))

    def force_grasp_at_width(self, width, gap):
        """Enter force-hold mode after confirmed contact, retaining hysteresis at this gap."""
        width, gap = float(width), float(gap)
        if not math.isfinite(width) or not math.isfinite(gap):
            raise ValueError("contact grasp width and Omega gap must be finite")
        self.mode = self.GRASP
        self.release_floor = None
        self.held_width = None
        self.held_epsilon_outer = None
        self.contact_release_gap = min(
            self.config.omega_open_gap,
            max(self.config.omega_release_gap,
                gap + self.config.omega_contact_release_margin),
        )
        self.last_width = min(max(
            width, self.config.robot_closed_width), self.config.robot_open_width)


def intent_changed(intent, previous, deadband):
    if previous is None or intent.mode != previous.mode:
        return True
    return abs(intent.width - previous.width) >= deadband


def bounded_move_target(target_width, basis_width, max_step, minimum_width, maximum_width):
    """Return a Move target no farther than max_step from its measured/commanded basis."""
    values = (target_width, basis_width, max_step, minimum_width, maximum_width)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Move segment inputs must be finite")
    if max_step <= 0.0 or minimum_width >= maximum_width:
        raise ValueError("Move segment limits are invalid")
    target = min(max(float(target_width), minimum_width), maximum_width)
    basis = min(max(float(basis_width), minimum_width), maximum_width)
    delta = target - basis
    if abs(delta) <= max_step:
        return target
    return basis + math.copysign(max_step, delta)


def prepare_grasp_intent(intent, config, contact_width=None, full_close=False):
    """Prepare force grasp with tolerance covering any physically reachable wider object.

    A contact-confirmed width is a better nominal target than the Omega-mapped width.
    Without confirmed contact (including explicit full-close), retain the mapped target,
    but extend epsilon.outer through the remaining finger range so an object wider than
    that target does not make the Grasp action fail solely because its width is unknown.
    """
    if intent.mode != GripperIntentMapper.GRASP:
        raise ValueError("Grasp goal preparation requires a grasp intent")
    target_width = intent.width
    if contact_width is not None:
        if not math.isfinite(float(contact_width)):
            raise ValueError("contact width must be finite")
        target_width = min(max(
            float(contact_width), config.robot_closed_width), config.robot_open_width)
    elif full_close:
        target_width = config.robot_closed_width
    # Franka accepts a grasp if its stopped width is within target +/- epsilon.
    # Unknown objects can stop anywhere above the requested target, so use a bounded
    # outer epsilon covering the remaining physical range. This preserves a meaningful
    # narrow target and avoids relying on one fixed object-width tolerance.
    epsilon_inner = min(
        config.epsilon_inner, max(0.0, target_width - config.robot_closed_width))
    epsilon_outer = max(config.epsilon_outer, config.robot_open_width - target_width)
    return GripperIntent(intent.mode, target_width), epsilon_inner, epsilon_outer


class ClosingContactDetector:
    """Confirm contact only from a failed accepted closing Move and fresh feedback.

    A running Move is never evidence of contact: it may simply be moving slowly. The
    accepted segment target is held unchanged until the action result arrives. A wider
    Omega intent invalidates that candidate immediately, so a reversal cannot turn an
    obsolete closing failure into a force grasp.
    """

    def __init__(self, error_threshold, clear_error):
        self.error_threshold = error_threshold
        self.clear_error = clear_error
        self.target_width = None
        self.closing_intent_floor = None

    @property
    def pending(self):
        return self.target_width is not None

    def move_started(self, basis_width, target_width, overall_target_width=None):
        """Track one accepted closing Move against its exact segment target."""
        if target_width >= basis_width - self.clear_error:
            self.reset()
            return
        self.target_width = float(target_width)
        self.closing_intent_floor = (
            self.target_width if overall_target_width is None
            else float(overall_target_width))

    def update_intent(self, overall_target_width, reversal_deadband):
        """Reset on a clear reversal without changing the accepted segment target."""
        if not self.pending:
            return False
        self.closing_intent_floor = min(
            self.closing_intent_floor, float(overall_target_width))
        if overall_target_width > self.closing_intent_floor + reversal_deadband:
            self.reset()
            return True
        return False

    @property
    def closing_intent_active(self):
        return self.pending

    def width_error(self, measured_width):
        """Measured-minus-accepted-segment width; never uses the overall Omega target."""
        if not self.pending:
            return None
        return float(measured_width) - self.target_width

    def move_result(self, failed, intentional_cancel, measured_width,
                    state_fresh, closing_intent_active):
        """Return (contact, segment_error); active/success/cancel/stale cannot grasp.

        ``failed`` must represent an unambiguous aborted action result. Callers should
        pass false for an inconsistent/unknown action status so ambiguous feedback
        cannot cause a force grasp.
        """
        if not self.pending:
            return False, None
        error = self.width_error(measured_width) if state_fresh else None
        confirmed = (
            failed and not intentional_cancel and state_fresh
            and closing_intent_active and self.closing_intent_active
            and error >= self.error_threshold
        )
        self.reset()
        return confirmed, error

    def reset(self):
        self.target_width = None
        self.closing_intent_floor = None


class LatestGripperIntent:
    """One in-flight action; retain only the newest intent for the next dispatch."""

    def __init__(self, deadband, command_interval):
        self.deadband = deadband
        self.command_interval = command_interval
        self.desired = None
        self.active = None
        self.last_dispatched = None
        self.last_dispatched_at = float("-inf")
        self.grasp_hold_latched = False

    def update(self, intent):
        self.desired = intent
        if intent.mode != GripperIntentMapper.GRASP:
            self.grasp_hold_latched = False

    def needs_cancel(self):
        return (self.active is not None and self.desired is not None
                and self.active.mode != self.desired.mode)

    def begin_if_ready(self, now, command=None):
        if self.active is not None or self.desired is None:
            return None
        command = self.desired if command is None else command
        if command.mode != self.desired.mode:
            raise ValueError("dispatched gripper mode must match the latest desired mode")
        if command.mode == GripperIntentMapper.GRASP and self.grasp_hold_latched:
            return None
        if now - self.last_dispatched_at < self.command_interval:
            return None
        if not intent_changed(command, self.last_dispatched, self.deadband):
            return None
        self.active = command
        self.last_dispatched = command
        self.last_dispatched_at = now
        return self.active

    def complete(self, success=False):
        completed = self.active
        self.active = None
        if completed is not None and completed.mode == GripperIntentMapper.GRASP and success:
            self.grasp_hold_latched = True
        return completed
