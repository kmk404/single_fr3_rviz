"""Omega.7 Cartesian control. Arm motion is opt-in and fails closed."""

import math
import os
import time

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from franka_msgs.action import Grasp, Move
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .dhd import DhdDevice, DhdError
from .gripper_control import (
    GripperConfig,
    GripperIntent,
    GripperIntentMapper,
    ClosingContactDetector,
    LatestGripperIntent,
    bounded_move_target,
    prepare_grasp_intent,
    validate_action_names,
)
from .math3d import (
    OMEGA_RAW_TO_FR3_BASE_AXES,
    clamp,
    clamp_workspace,
    finite,
    limit_pose_step,
    mapped_relative_target_with_diagnostics,
)
from .state import MotionGate


class IkBranchJump(ValueError):
    def __init__(self, index, delta):
        super().__init__(f"IK joint {index + 1} branch jump {delta:.5f} rad exceeds limit")
        self.index = index
        self.delta = delta


class Teleop(Node):
    JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
    JOINT_LOWER = (-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508)
    JOINT_UPPER = (2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508)

    def __init__(self):
        super().__init__("omega7_fresh")
        defaults = {
            "sdk_library": os.environ.get("OMEGA7_SDK_LIBRARY", "/tmp/forcedimension_sdk/lib/release/lin-x86_64-gcc/libdhd.so.3.17.7"),
            "fake_hardware": False, "base_frame": "fr3_link0", "ee_link": "fr3_hand_tcp",
            "arm_group": "fr3_arm", "ik_service": "/compute_ik",
            "gripper_move_action": "/franka_gripper/move",
            "gripper_grasp_action": "/franka_gripper/grasp", "command_rate_hz": 50.0,
            "device_timeout": 0.1, "joint_state_timeout": 0.25,
            "position_scale": [2.0, 2.5, 2.5],
            "axis_mapping": [value for row in OMEGA_RAW_TO_FR3_BASE_AXES for value in row],
            "orientation_scale": 1.0, "position_deadzone": 0.001,
            "orientation_deadzone": 0.015, "max_linear_speed": 0.15,
            "max_angular_speed": 0.60, "workspace_min": [0.10, -0.55, 0.00],
            "workspace_max": [0.75, 0.55, 0.90],
            "max_joint_velocity": [0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.10],
            "max_ik_joint_delta": 0.12, "max_servo_following_error": 0.08,
            "min_joint_limit_margin": 0.02,
            "omega_gripper_closed_gap": 0.0, "omega_gripper_open_gap": 0.025,
            "omega_gripper_engage_gap": 0.001, "omega_gripper_release_gap": 0.015,
            "omega_gripper_full_close_gap": 0.001,
            "omega_gripper_contact_release_margin": 0.002,
            "robot_gripper_closed_width": 0.0, "robot_gripper_open_width": 0.08,
            "gripper_max_move_step": 0.080,
            "gripper_speed": 0.10, "gripper_grasp_speed": 0.05,
            "gripper_grasp_force": 20.0,
            "gripper_grasp_epsilon_inner": 0.005,
            "gripper_grasp_epsilon_outer": 0.015,
            "gripper_release_margin": 0.010,
            "gripper_contact_error_threshold": 0.006,
            "gripper_contact_clear_error": 0.003,
            "gripper_deadband": 0.001,
            "gripper_command_interval": 0.05,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.gripper_config = GripperConfig.from_parameters(self.p)
        validate_action_names(self.p)
        if len(self.p["axis_mapping"]) != 9 or len(self.p["max_joint_velocity"]) != 7:
            raise ValueError("axis_mapping needs 9 values and max_joint_velocity needs 7")
        self.axes = tuple(tuple(self.p["axis_mapping"][3*r+c] for c in range(3)) for r in range(3))
        if any(abs(sum(self.axes[i][k] * self.axes[j][k] for k in range(3)) - (1.0 if i == j else 0.0)) > 1e-6
               for i in range(3) for j in range(3)):
            raise ValueError("axis_mapping must be orthogonal with unit-length axes")
        determinant = (self.axes[0][0] * (self.axes[1][1]*self.axes[2][2] - self.axes[1][2]*self.axes[2][1])
                       - self.axes[0][1] * (self.axes[1][0]*self.axes[2][2] - self.axes[1][2]*self.axes[2][0])
                       + self.axes[0][2] * (self.axes[1][0]*self.axes[2][1] - self.axes[1][1]*self.axes[2][0]))
        if abs(determinant - 1.0) > 1e-6:
            raise ValueError("axis_mapping must preserve right-handed orientation")
        self.gate = MotionGate()
        self.device = None
        self.sample = None
        self.sample_at = 0.0
        self.joints = None
        self.joint_velocities = None
        self.joints_at = 0.0
        self.master_zero = self.robot_zero = self.limited_pose = None
        self.hold_q = None
        self.last_ik_solution = None
        self.hold_q_at = 0.0
        self.pose_target_at = 0.0
        self.pending_pose = None
        self.retry_pose = None
        self.ik_pending = False
        self.ik_sent_at = 0.0
        self.ik_failures = 0
        self.ik_branch_rejections = 0
        self.arm_fault_latched = False
        self.controller_active = False
        self.controller_state = "unknown"
        self.controller_checked_at = 0.0
        self.controller_list_pending = False
        self.next_open = 0.0
        self.last_diagnostic_at = 0.0
        self.ik_diagnostic_started_at = 0.0
        self.ik_successes = 0
        self.workspace_clamped = False
        self.last_gripper_width = None
        self.gripper_measured_width = None
        self.gripper_measured_at = 0.0
        self.gripper_contact_width = None
        self.full_close_requested = False
        self.gripper_active_width_before = None
        self.gripper_active_epsilon_outer = None
        self.gripper_pending_contact_start = None
        self.gripper_mapper = GripperIntentMapper(
            self.gripper_config, apply_release_margin=not self.p["fake_hardware"])
        self.gripper_intents = LatestGripperIntent(
            self.gripper_config.deadband, self.gripper_config.command_interval)
        self.gripper_contact = ClosingContactDetector(
            self.gripper_config.contact_error_threshold,
            self.gripper_config.contact_clear_error,
        )
        self.gripper_cancel_requested = False
        self.gripper_cancel_command = None
        self.gripper_goal_handle = None
        self.servo_target = self.create_publisher(
            JointState, "/omega7_servo_controller/target_joint_states", 10)
        self.servo_enable = self.create_publisher(Bool, "/omega7_servo_controller/enable", 10)
        self.controller_list = self.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.create_subscription(Bool, "/omega7_servo_controller/fault_latched",
                                 self.on_servo_fault, 10)
        self.create_timer(0.5, self.check_controller)
        self.status = self.create_publisher(Bool, "~/motion_enabled", 10)
        self.fake_gripper = self.create_publisher(JointState, "/joint_states", 10) if self.p["fake_hardware"] else None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.ik = self.create_client(GetPositionIK, self.p["ik_service"])
        self.gripper_move = ActionClient(self, Move, self.p["gripper_move_action"])
        self.gripper_grasp = ActionClient(self, Grasp, self.p["gripper_grasp_action"])
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_service(SetBool, "~/set_motion", self.set_motion)
        self.create_timer(0.005, self.read_device)
        self.create_timer(1.0 / self.p["command_rate_hz"], self.command)
        self.create_timer(0.02, self.watchdog)
        self.create_timer(0.2, self.publish_status)
        if self.fake_gripper is not None:
            self.create_timer(0.1, self.publish_fake_gripper_state)
        self.get_logger().info("Ready. SPACE explicitly enables or disables arm motion.")

    def publish_status(self):
        self.status.publish(Bool(data=self.gate.enabled))
        self.servo_enable.publish(Bool(data=self.gate.enabled))

    def publish_fake_gripper_state(self):
        width = self.last_gripper_width if self.last_gripper_width is not None else 0.08
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["fr3_finger_joint1", "fr3_finger_joint2"]
        msg.position = [width / 2, width / 2]
        self.fake_gripper.publish(msg)

    def on_servo_fault(self, msg):
        if msg.data:
            self.arm_fault_latched = True
            self.stop("servo controller fault latched")

    def check_controller(self):
        if self.controller_list_pending or not self.controller_list.service_is_ready():
            return
        self.controller_list_pending = True
        self.controller_list.call_async(ListControllers.Request()).add_done_callback(
            self.on_controller_list)

    def on_controller_list(self, future):
        self.controller_list_pending = False
        try:
            controllers = future.result().controller
            previous_state = self.controller_state
            controller = next((item for item in controllers
                               if item.name == "omega7_servo_controller"), None)
            self.controller_state = controller.state if controller is not None else "missing"
            self.controller_active = self.controller_state == "active"
            self.controller_checked_at = time.monotonic()
            if self.controller_state != previous_state:
                if self.controller_active:
                    self.get_logger().info(
                        f"omega7 servo controller state: {self.controller_state}")
                else:
                    self.get_logger().warning(
                        f"omega7 servo controller state: {self.controller_state}")
        except Exception as exc:
            self.controller_active = False
            self.controller_state = "unavailable"
            self.get_logger().warning(f"controller status unavailable: {exc}")

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        finger_names = ("fr3_finger_joint1", "fr3_finger_joint2")
        if all(name in values for name in finger_names):
            measured_width = float(values[finger_names[0]]) + float(values[finger_names[1]])
            if (math.isfinite(measured_width)
                    and self.gripper_config.robot_closed_width <= measured_width
                    <= self.gripper_config.robot_open_width):
                self.gripper_measured_width = measured_width
                self.gripper_measured_at = time.monotonic()
        if all(name in values for name in self.JOINTS):
            joints = tuple(float(values[name]) for name in self.JOINTS)
            if finite(joints):
                self.joints, self.joints_at = joints, time.monotonic()
                if all(name in velocities for name in self.JOINTS):
                    speeds = tuple(float(velocities[name]) for name in self.JOINTS)
                    self.joint_velocities = speeds if finite(speeds) else None
                else:
                    self.joint_velocities = None

    def joint_limit_margin(self, joints):
        margins = [min(joints[i] - self.JOINT_LOWER[i], self.JOINT_UPPER[i] - joints[i])
                   for i in range(7)]
        index = min(range(7), key=lambda i: margins[i])
        return index, margins[index]

    def read_device(self):
        now = time.monotonic()
        if self.device is None:
            if now < self.next_open:
                return
            self.next_open = now + 2.0
            try:
                candidate = DhdDevice(self.p["sdk_library"])
                candidate.connect()
                self.device = candidate
                self.get_logger().info("Omega.7 connected")
            except DhdError as exc:
                self.get_logger().warning(str(exc))
            return
        try:
            sample = self.device.sample()
            if max(abs(v) for v in sample.position) > 0.30:
                raise DhdError("Omega.7 position outside sanity bound")
            self.sample, self.sample_at = sample, now
            self.send_gripper(sample.gripper_gap)
        except (DhdError, ValueError) as exc:
            self.stop(f"device fault: {exc}")
            try:
                self.device.close()
            except Exception as close_exc:
                self.get_logger().warning(f"DHD close failed: {close_exc}")
            self.device, self.sample = None, None
            self.next_open = now + 2.0

    def current_pose(self):
        transform = self.tf.lookup_transform(self.p["base_frame"], self.p["ee_link"], Time())
        t, q = transform.transform.translation, transform.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def set_motion(self, request, response):
        if not request.data:
            self.stop("SPACE disabled motion")
            response.success, response.message = True, "arm disabled"
            return response
        now = time.monotonic()
        missing = []
        if self.device is None or self.sample is None or now - self.sample_at >= self.p["device_timeout"]:
            missing.append("Omega.7 sample")
        if self.joints is None or now - self.joints_at >= self.p["joint_state_timeout"]:
            missing.append("FR3 /joint_states")
        if self.joint_velocities is None:
            missing.append("FR3 joint velocities")
        elif max(abs(v) for v in self.joint_velocities) > 0.05:
            missing.append("FR3 not stationary")
        if not self.ik.service_is_ready():
            missing.append("/compute_ik")
        if not self.controller_active or now - self.controller_checked_at > 1.5:
            missing.append(f"omega7 servo controller active state (reported {self.controller_state})")
        if self.arm_fault_latched:
            missing.append("servo controller fault; restart teleop after inspecting and recovering FR3")
        if self.joints is not None:
            limit_index, limit_margin = self.joint_limit_margin(self.joints)
            if limit_margin < self.p["min_joint_limit_margin"]:
                limit_message = (f"FR3 joint {limit_index + 1} only {limit_margin:.5f} rad "
                                 "from its position limit")
                missing.append(limit_message)
        ready = not missing
        pose = None
        if ready:
            try:
                pose = self.current_pose()
            except TransformException as exc:
                missing.append(f"TF {self.p['base_frame']} -> {self.p['ee_link']}: {exc}")
        success, message = self.gate.enable(ready and pose is not None)
        if not success and missing:
            message = "arm remains disabled; missing " + ", ".join(missing)
        if success:
            self.master_zero = (self.sample.position, self.sample.quaternion)
            self.robot_zero = self.limited_pose = pose
            self.hold_q = tuple(self.joints)
            self.last_ik_solution = tuple(self.joints)
            self.hold_q_at = self.pose_target_at = now
            self.pending_pose = None
            self.retry_pose = None
            self.ik_failures = 0
            self.ik_branch_rejections = 0
            self.ik_diagnostic_started_at = now
            self.ik_successes = 0
            self.workspace_clamped = False
            self.servo_enable.publish(Bool(data=True))
            self.get_logger().info(
                "Arm enabled; DHD orientation and FR3 TCP zero poses captured; "
                "translation maps DHD raw world -> FR3 base with A=%s" % (self.axes,)
            )
        response.success, response.message = success, message
        self.publish_status()
        return response

    def stop(self, reason):
        if self.gate.disable():
            self.get_logger().warning(reason)
        self.servo_enable.publish(Bool(data=False))
        self.ik_pending = False
        self.master_zero = self.robot_zero = self.limited_pose = None
        self.hold_q = self.pending_pose = None
        self.retry_pose = None
        self.last_ik_solution = None
        self.publish_status()

    def watchdog(self):
        if not self.gate.enabled:
            return
        now = time.monotonic()
        if now - self.sample_at > self.p["device_timeout"]:
            self.stop("Omega.7 sample timeout")
        elif now - self.joints_at > self.p["joint_state_timeout"]:
            self.stop("FR3 joint state timeout")
        elif self.joint_velocities is not None and max(abs(v) for v in self.joint_velocities) > 0.30:
            self.stop("FR3 measured joint speed exceeded 0.30 rad/s")
        elif self.ik_pending and now - self.ik_sent_at > 0.2:
            self.stop("IK response timeout")
        elif not self.controller_active or now - self.controller_checked_at > 1.5:
            self.stop("servo controller inactive or status stale")
        elif self.hold_q is not None and self.joints is not None and max(
                abs(self.hold_q[i] - self.joints[i]) for i in range(7)
                ) > self.p["max_servo_following_error"]:
            self.stop("servo following error exceeded %.3f rad" %
                      self.p["max_servo_following_error"])

    def command(self):
        if not self.gate.enabled or self.ik_pending:
            return
        self.watchdog()
        if not self.gate.enabled:
            return
        mp, mq = self.master_zero
        rp, rq = self.robot_zero
        mapping_result = mapped_relative_target_with_diagnostics(
            mp,
            mq,
            self.sample.position,
            self.sample.quaternion,
            rp,
            rq,
            self.axes,
            self.p["position_scale"],
            self.p["orientation_scale"],
            self.p["position_deadzone"],
            self.p["orientation_deadzone"],
        )
        desired = mapping_result[:2]
        raw_world_delta, _mapped_base_delta = mapping_result[2:]
        clamped_position = clamp_workspace(
            desired[0], self.p["workspace_min"], self.p["workspace_max"])
        target = (clamped_position, desired[1])
        now = time.monotonic()
        if self.retry_pose is not None:
            target = self.retry_pose
        else:
            pose_dt = min(max(now - self.pose_target_at, 0.0), 0.1)
            target = limit_pose_step(*self.limited_pose, *target,
                                     self.p["max_linear_speed"] * pose_dt,
                                     self.p["max_angular_speed"] * pose_dt)
        clamped = any(abs(clamped_position[i] - desired[0][i]) > 1.0e-9
                      for i in range(3))
        if clamped and not self.workspace_clamped:
            self.get_logger().warning(
                "Cartesian target clamped: desired xyz=[%.3f, %.3f, %.3f] m, "
                "allowed min=[%.3f, %.3f, %.3f] max=[%.3f, %.3f, %.3f] m" %
                (*desired[0], *self.p["workspace_min"], *self.p["workspace_max"]))
        self.workspace_clamped = clamped
        fr3_base_delta = tuple(clamped_position[i] - rp[i] for i in range(3))
        diagnostic_motion_threshold = max(0.002, 2.0 * self.p["position_deadzone"])
        if (math.dist(self.sample.position, mp) > diagnostic_motion_threshold
                and now - self.last_diagnostic_at > 2.0):
            self.last_diagnostic_at = now
            elapsed = max(now - self.ik_diagnostic_started_at, 1.0e-6)
            ik_rate = self.ik_successes / elapsed
            following_error = (max(abs(self.hold_q[i] - self.joints[i]) for i in range(7))
                               if self.hold_q is not None and self.joints is not None else math.nan)
            self.get_logger().info(
                "DHD raw delta xyz=[%.3f, %.3f, %.3f] m -> mapped FR3 base "
                "desired delta xyz=[%.3f, %.3f, %.3f] m; IK %.1f Hz; "
                "joint tracking error %.3f rad%s" %
                (*raw_world_delta, *fr3_base_delta, ik_rate, following_error,
                 "; workspace clamped" if clamped else ""))
            self.ik_diagnostic_started_at = now
            self.ik_successes = 0
        generation = self.gate.generation
        self.pending_pose = target
        self.send_ik_request(target, self.hold_q, generation, retried=False)

    def send_ik_request(self, target, seed, generation, retried):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name, ik.avoid_collisions = self.p["arm_group"], self.p["ee_link"], True
        ik.robot_state.joint_state.name = self.JOINTS
        ik.robot_state.joint_state.position = list(seed)
        ik.timeout = Duration(sec=0, nanosec=25000000)
        pose = PoseStamped()
        pose.header.frame_id = self.p["base_frame"]
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target[0]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = target[1]
        ik.pose_stamped = pose
        self.ik_pending, self.ik_sent_at = True, time.monotonic()
        self.ik.call_async(request).add_done_callback(
            lambda result: self.on_ik(result, generation, retried))

    def on_ik(self, future, generation, retried=False):
        if generation != self.gate.generation or not self.gate.enabled:
            return
        try:
            result = future.result()
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise ValueError(f"IK code {result.error_code.val}")
            solution = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            target = [float(solution[name]) for name in self.JOINTS]
            if not finite(target):
                raise ValueError("IK returned invalid joints")
            # Compare complete IK solutions with each other. hold_q is deliberately
            # rate-limited and can lag a valid IK path by much more than one branch step.
            branch_reference = self.last_ik_solution or self.hold_q
            deltas = [abs(target[i] - branch_reference[i]) for i in range(7)]
            jump_index = max(range(7), key=lambda i: deltas[i])
            if deltas[jump_index] > self.p["max_ik_joint_delta"]:
                if (not retried and self.joints is not None
                        and time.monotonic() - self.joints_at < self.p["joint_state_timeout"]):
                    self.send_ik_request(
                        self.pending_pose, tuple(self.joints), generation, retried=True)
                    return
                raise IkBranchJump(jump_index, deltas[jump_index])
        except IkBranchJump as exc:
            self.ik_pending = False
            self.retry_pose = self.pending_pose
            self.pending_pose = None
            self.ik_branch_rejections += 1
            if self.ik_branch_rejections == 1 or self.ik_branch_rejections % 10 == 0:
                self.get_logger().warning(
                    f"IK continuity hold ({self.ik_branch_rejections} rejected): {exc}; "
                    "holding the last valid arm target")
            self.refresh_servo_hold()
            return
        except Exception as exc:
            self.ik_pending = False
            self.retry_pose = self.pending_pose
            self.pending_pose = None
            self.ik_failures += 1
            self.get_logger().warning(f"IK failed ({self.ik_failures}/3): {exc}")
            if self.ik_failures >= 3:
                self.stop(f"repeated IK failure: {exc}")
            else:
                self.refresh_servo_hold()
            return
        self.ik_pending = False
        self.ik_failures = 0
        self.ik_branch_rejections = 0
        now = time.monotonic()
        # IK service throughput can be lower than command_rate_hz.  Use the real
        # interval so max_joint_velocity remains a rate rather than shrinking in
        # proportion to IK latency.  The 100 ms cap also keeps each published
        # target below the servo controller's target-jump safety gate.
        step_dt = min(max(now - self.hold_q_at, 0.0), 0.1)
        limited = [self.hold_q[i] + clamp(target[i] - self.hold_q[i],
                   -self.p["max_joint_velocity"][i] * step_dt,
                   self.p["max_joint_velocity"][i] * step_dt) for i in range(7)]
        self.pose_target_at = now
        self.limited_pose = self.pending_pose
        self.pending_pose = None
        self.retry_pose = None
        self.hold_q = tuple(limited)
        self.last_ik_solution = tuple(target)
        self.hold_q_at = now
        self.ik_successes += 1
        limit_index, limit_margin = self.joint_limit_margin(limited)
        if limit_margin < self.p["min_joint_limit_margin"]:
            self.stop(f"FR3 joint {limit_index + 1} target too close to position limit: "
                      f"{limit_margin:.5f} rad")
            return
        self.send_arm_goal(limited)

    def refresh_servo_hold(self):
        """Keep the controller target fresh while an IK result is temporarily rejected."""
        if self.gate.enabled and self.hold_q is not None:
            self.send_arm_goal(self.hold_q)

    def send_arm_goal(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINTS
        msg.position = list(positions)
        self.servo_target.publish(msg)

    def send_gripper(self, gap):
        intent = self.gripper_mapper.update(gap)
        self.full_close_requested = gap <= self.gripper_config.omega_full_close_gap
        if intent.mode == GripperIntentMapper.MOVE:
            self.gripper_contact.update_intent(
                intent.width, self.gripper_config.deadband)
        if intent.mode != GripperIntentMapper.GRASP:
            self.gripper_contact_width = None
        self.gripper_intents.update(intent)
        if self.fake_gripper is not None:
            self.last_gripper_width = intent.width
            return
        self._cancel_gripper_on_mode_change()
        self._dispatch_latest_gripper_intent()

    def _active_gripper_client(self, mode):
        return self.gripper_grasp if mode == GripperIntentMapper.GRASP else self.gripper_move

    def _gripper_width_basis(self, now, fallback=None):
        if (self.gripper_measured_width is not None
                and now - self.gripper_measured_at <= self.p["joint_state_timeout"]):
            return clamp(
                self.gripper_measured_width,
                self.gripper_config.robot_closed_width,
                self.gripper_config.robot_open_width,
            )
        if fallback is not None:
            return clamp(
                fallback,
                self.gripper_config.robot_closed_width,
                self.gripper_config.robot_open_width,
            )
        if self.last_gripper_width is not None:
            return clamp(
                self.last_gripper_width,
                self.gripper_config.robot_closed_width,
                self.gripper_config.robot_open_width,
            )
        return self.gripper_config.robot_open_width

    def _cancel_gripper_on_mode_change(self):
        if (not self.gripper_intents.needs_cancel() or self.gripper_goal_handle is None
                or self.gripper_cancel_requested):
            return
        self.gripper_cancel_requested = True
        self.gripper_cancel_command = self.gripper_intents.active
        try:
            self.gripper_goal_handle.cancel_goal_async().add_done_callback(
                self.on_gripper_cancel_response)
        except Exception as exc:
            self.get_logger().warning(f"Could not request gripper action cancellation: {exc}")

    def on_gripper_cancel_response(self, future):
        try:
            response = future.result()
            if (not response.goals_canceling
                    and self.gripper_intents.active is self.gripper_cancel_command):
                self.get_logger().warning(
                    "Gripper action did not accept cancellation; latest intent will run at completion")
            self.gripper_cancel_command = None
        except Exception as exc:
            self.gripper_cancel_command = None
            self.get_logger().warning(f"Gripper action cancellation failed: {exc}")

    def _dispatch_latest_gripper_intent(self):
        if self.fake_gripper is not None:
            return
        desired = self.gripper_intents.desired
        if desired is None:
            return
        if desired.mode == GripperIntentMapper.MOVE and self.gripper_contact.pending:
            return
        client = self._active_gripper_client(desired.mode)
        if not client.server_is_ready():
            return
        now = time.monotonic()
        basis_width = self._gripper_width_basis(now)
        command = desired
        if desired.mode == GripperIntentMapper.MOVE:
            command = GripperIntent(
                desired.mode,
                bounded_move_target(
                    desired.width,
                    basis_width,
                    self.gripper_config.max_move_step,
                    self.gripper_config.robot_closed_width,
                    self.gripper_config.robot_open_width,
                ),
            )
        intent = self.gripper_intents.begin_if_ready(now, command)
        if intent is None:
            return

        if intent.mode == GripperIntentMapper.GRASP:
            self.gripper_pending_contact_start = None
            contact_width = self.gripper_contact_width
            intent, epsilon_inner, epsilon_outer = prepare_grasp_intent(
                intent, self.gripper_config, contact_width=contact_width,
                full_close=self.full_close_requested)
            # Record the effective target so successful-hold latching describes the
            # exact goal sent to Franka rather than the pre-contact analog mapping.
            self.gripper_intents.active = intent
            self.gripper_intents.last_dispatched = intent
            self.gripper_active_epsilon_outer = epsilon_outer
            goal = Grasp.Goal()
            goal.width = intent.width
            goal.speed = self.gripper_config.grasp_speed
            goal.force = self.gripper_config.grasp_force
            goal.epsilon.inner = epsilon_inner
            goal.epsilon.outer = epsilon_outer
        else:
            self.gripper_active_epsilon_outer = None
            self.gripper_pending_contact_start = (
                (basis_width, intent.width, desired.width)
                if command.width < basis_width - self.gripper_config.contact_clear_error
                else None
            )
            goal = Move.Goal()
            goal.width = intent.width
            goal.speed = self.gripper_config.move_speed

        self.gripper_active_width_before = basis_width
        self.last_gripper_width = intent.width
        self.gripper_goal_handle = None
        self.gripper_cancel_requested = False
        self.gripper_cancel_command = None
        try:
            client.send_goal_async(goal).add_done_callback(self.on_gripper_goal)
        except Exception as exc:
            self.gripper_intents.complete()
            self.gripper_contact.reset()
            self.gripper_pending_contact_start = None
            self.last_gripper_width = self.gripper_active_width_before
            self.gripper_active_width_before = None
            self.gripper_active_epsilon_outer = None
            self.get_logger().error(f"Could not send gripper {intent.mode} goal: {exc}")

    def on_gripper_goal(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                self.gripper_goal_handle = handle
                active_command = self.gripper_intents.active
                if (active_command is not None
                        and active_command.mode == GripperIntentMapper.MOVE
                        and self.gripper_pending_contact_start is not None):
                    basis_width, segment_target_width, overall_target_width = (
                        self.gripper_pending_contact_start)
                    latest = self.gripper_intents.desired
                    if (latest is not None
                            and latest.mode == GripperIntentMapper.MOVE
                            and latest.width <= overall_target_width
                            + self.gripper_config.deadband):
                        self.gripper_contact.move_started(
                            basis_width, segment_target_width, overall_target_width)
                        self.gripper_contact.update_intent(
                            latest.width, self.gripper_config.deadband)
                self.gripper_pending_contact_start = None
                handle.get_result_async().add_done_callback(self.on_gripper_result)
                self._cancel_gripper_on_mode_change()
            else:
                command = self.gripper_intents.complete()
                self.gripper_contact.reset()
                if command is not None and command.mode == GripperIntentMapper.GRASP:
                    self.gripper_contact_width = None
                self.gripper_pending_contact_start = None
                self.gripper_cancel_requested = False
                self.gripper_cancel_command = None
                self.last_gripper_width = self.gripper_active_width_before
                self.gripper_active_width_before = None
                self.gripper_active_epsilon_outer = None
                self.get_logger().warning(
                    f"Franka gripper rejected {command.mode if command else 'unknown'} goal")
                self._dispatch_latest_gripper_intent()
        except Exception as exc:
            command = self.gripper_intents.complete()
            self.gripper_contact.reset()
            if command is not None and command.mode == GripperIntentMapper.GRASP:
                self.gripper_contact_width = None
            self.gripper_pending_contact_start = None
            self.gripper_cancel_requested = False
            self.gripper_cancel_command = None
            self.last_gripper_width = self.gripper_active_width_before
            self.gripper_active_width_before = None
            self.gripper_active_epsilon_outer = None
            self.get_logger().warning(
                f"Could not submit gripper {command.mode if command else 'unknown'} goal: {exc}")
            self._dispatch_latest_gripper_intent()

    def on_gripper_result(self, future):
        cancel_requested = self.gripper_cancel_requested
        self.gripper_goal_handle = None
        self.gripper_cancel_requested = False
        self.gripper_cancel_command = None
        try:
            response = future.result()
            result = response.result
            result_success = bool(result.success)
            succeeded = (response.status == GoalStatus.STATUS_SUCCEEDED
                         and result_success)
            active_command = self.gripper_intents.active
            move_result_failed = (
                active_command is not None
                and active_command.mode == GripperIntentMapper.MOVE
                and not succeeded
            )
            # A cancel request is intentional regardless of how the server reports
            # the terminal status. Only an explicit ABORTED + success=False result
            # is sufficiently unambiguous to support contact inference.
            intentional_cancel = move_result_failed and cancel_requested
            contact_eligible_failure = (
                move_result_failed
                and response.status == GoalStatus.STATUS_ABORTED
                and not result_success
                and not cancel_requested
            )
            now = time.monotonic()
            state_fresh = (
                self.gripper_measured_width is not None
                and now - self.gripper_measured_at <= self.p["joint_state_timeout"]
            )
            sample_fresh = (
                self.sample is not None and now - self.sample_at <= self.p["device_timeout"]
            )
            desired = self.gripper_intents.desired
            closing_intent_active = (
                sample_fresh
                and desired is not None
                and desired.mode == GripperIntentMapper.MOVE
                and self.gripper_contact.closing_intent_active
            )
            contact_confirmed, segment_error = (False, None)
            if active_command is not None and active_command.mode == GripperIntentMapper.MOVE:
                contact_confirmed, segment_error = self.gripper_contact.move_result(
                    failed=contact_eligible_failure,
                    intentional_cancel=intentional_cancel,
                    measured_width=(self.gripper_measured_width if state_fresh else 0.0),
                    state_fresh=state_fresh,
                    closing_intent_active=closing_intent_active,
                )
            else:
                self.gripper_contact.reset()
            command = self.gripper_intents.complete(success=succeeded)
            if (not succeeded and command is not None
                    and command.mode == GripperIntentMapper.GRASP):
                # Let subsequent squeeze updates retry with their latest narrower
                # Omega target instead of pinning every retry to the first contact width.
                self.gripper_contact_width = None
            if contact_confirmed:
                measured_width = self.gripper_measured_width
                self.gripper_mapper.force_grasp_at_width(
                    measured_width, self.sample.gripper_gap)
                self.gripper_contact_width = measured_width
                self.gripper_intents.update(
                    GripperIntent(GripperIntentMapper.GRASP, measured_width))
                self.get_logger().info(
                    "Failed closing Move left %.1f mm segment error with fresh feedback; "
                    "switching to force Grasp at %.1f mm" %
                    (segment_error * 1000.0, measured_width * 1000.0))
            elif move_result_failed:
                if intentional_cancel:
                    self.get_logger().info(
                        "Closing Move ended after an intentional cancellation; "
                        "not assuming contact")
                elif response.status != GoalStatus.STATUS_ABORTED or result_success:
                    self.get_logger().warning(
                        "Closing Move ended with ambiguous action feedback "
                        f"(status={response.status}, success={result_success}); "
                        "not assuming contact")
                elif not state_fresh:
                    self.get_logger().warning(
                        "Closing Move failed, but finger width is stale; not assuming contact")
                elif not closing_intent_active:
                    self.get_logger().info(
                        "Closing Move failed after opening/reversal intent; not assuming contact")
                else:
                    self.get_logger().warning(
                        "Closing Move failed with fresh segment error %.1f mm below the "
                        "contact threshold; not assuming contact" %
                        ((segment_error or 0.0) * 1000.0))
            if not succeeded:
                self.last_gripper_width = self._gripper_width_basis(
                    now, self.gripper_active_width_before)
            if succeeded:
                current_width = getattr(result, "current_width", None)
                if (current_width is not None and math.isfinite(current_width)
                        and self.gripper_config.robot_closed_width <= current_width
                        <= self.gripper_config.robot_open_width):
                    self.last_gripper_width = current_width
                if command is not None and command.mode == GripperIntentMapper.GRASP:
                    hold_width = (current_width if current_width is not None
                                  and math.isfinite(current_width)
                                  and self.gripper_config.robot_closed_width <= current_width
                                  <= self.gripper_config.robot_open_width
                                  else self._gripper_width_basis(
                                      time.monotonic(), command.width))
                    self.gripper_mapper.mark_grasp_succeeded(
                        hold_width, self.gripper_active_epsilon_outer)
                if current_width is None:
                    self.get_logger().debug(f"Gripper {command.mode} completed successfully")
                else:
                    self.get_logger().debug(
                        f"Gripper {command.mode} completed at width {current_width:.4f} m")
            elif response.status == GoalStatus.STATUS_CANCELED and cancel_requested:
                self.get_logger().info(
                    "Gripper action canceled once to apply the new open/grasp intent")
            else:
                error = getattr(result, "error", "")
                current_width = getattr(result, "current_width", float("nan"))
                self.get_logger().warning(
                    f"Gripper {command.mode if command else 'unknown'} action failed "
                    f"(status={response.status}, success={getattr(result, 'success', False)}, "
                    f"width={current_width:.4f} m, error={error!r})")
        except Exception as exc:
            command = self.gripper_intents.complete()
            if command is not None and command.mode == GripperIntentMapper.MOVE:
                self.gripper_contact.reset()
            self.last_gripper_width = self._gripper_width_basis(
                time.monotonic(), self.gripper_active_width_before)
            self.get_logger().warning(
                f"Gripper {command.mode if command else 'unknown'} result unavailable: {exc}")
        self.gripper_active_width_before = None
        self.gripper_active_epsilon_outer = None
        self.gripper_pending_contact_start = None
        self._dispatch_latest_gripper_intent()

    def destroy_node(self):
        self.stop("node shutdown")
        if self.device is not None:
            try:
                self.device.close()
            except Exception as exc:
                self.get_logger().warning(f"DHD close failed: {exc}")
        super().destroy_node()


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
