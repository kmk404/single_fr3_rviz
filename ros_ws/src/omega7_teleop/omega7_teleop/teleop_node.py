"""ROS 2 node that maps an Omega.7 pose to a safely bounded FR3 trajectory."""

import os
import time

from builtin_interfaces.msg import Duration
from franka_msgs.action import Move
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .dhd import DhdDevice, DhdError
from .math3d import (
    clamp,
    clamp_workspace,
    finite,
    limit_pose_step,
    mapped_relative_target,
    matrix_multiply,
    matrix_transpose,
)


class Omega7Teleop(Node):
    """Clutched Cartesian teleoperation with a fixed seventh arm joint."""

    JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]

    def __init__(self):
        super().__init__("omega7_teleop")
        self._declare_parameters()
        self._read_parameters()

        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, self.trajectory_topic, 10
        )
        self._fake_gripper_publisher = (
            self.create_publisher(JointState, "/joint_states", 10)
            if self.fake_hardware
            else None
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._ik_client = self.create_client(GetPositionIK, self.ik_service)
        self._gripper_client = ActionClient(self, Move, self.gripper_action)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_service(Trigger, "~/recalibrate", self._on_recalibrate)

        self._device = None
        self._last_open_attempt = 0.0
        self._last_sample_time = None
        self._last_joint_state_time = None
        self._current_joints = None
        self._fixed_j7 = None
        self._release_seen = False
        self._active = False
        self._generation = 0
        self._ik_pending = False
        self._ik_failure_count = 0
        self._master_origin = None
        self._robot_origin = None
        self._limited_target = None
        self._latest_sample = None
        self._recalibrate_requested = False
        self._gripper_busy = False
        self._gripper_goal_handle = None
        self._gripper_cancel_requested = False
        self._gripper_requested_width = self.robot_gripper_open_width
        self._gripper_last_sent_width = None
        self._last_gripper_send = 0.0
        self._last_log_times = {}

        self.create_timer(1.0 / self.device_rate_hz, self._sample_device)
        self.create_timer(1.0 / self.command_rate_hz, self._command_tick)
        self.create_timer(0.02, self._watchdog_tick)
        self.get_logger().info(
            "Omega.7 teleop ready; release then squeeze the gripper to engage. "
            "J7 will be captured from the first complete /joint_states message."
        )

    def _declare_parameters(self):
        default_library = os.environ.get(
            "OMEGA7_SDK_LIBRARY",
            "/tmp/forcedimension_sdk/lib/release/lin-x86_64-gcc/libdhd.so.3.17.7",
        )
        self.declare_parameter("sdk_library", default_library)
        self.declare_parameter("require_omega7", True)
        self.declare_parameter("fake_hardware", False)
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_link", "fr3_hand_tcp")
        self.declare_parameter("arm_group", "fr3_arm")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter(
            "trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("gripper_action", "/franka_gripper/move")
        self.declare_parameter("device_rate_hz", 200.0)
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("command_duration", 0.12)
        self.declare_parameter("device_timeout", 0.10)
        self.declare_parameter("joint_state_timeout", 0.25)
        self.declare_parameter("ik_timeout", 0.025)
        self.declare_parameter("max_ik_failures", 3)
        self.declare_parameter("max_raw_position", 0.30)
        self.declare_parameter("position_scale", [1.0, 1.0, 1.0])
        self.declare_parameter(
            "axis_mapping", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter("orientation_scale", 1.0)
        self.declare_parameter("position_deadzone", 0.001)
        self.declare_parameter("orientation_deadzone", 0.015)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.40)
        self.declare_parameter("workspace_min", [0.10, -0.55, 0.05])
        self.declare_parameter("workspace_max", [0.75, 0.55, 0.90])
        self.declare_parameter(
            "max_joint_velocity", [0.35, 0.35, 0.35, 0.35, 0.50, 0.50, 0.0]
        )
        self.declare_parameter("j7_constraint_tolerance", 1.0e-4)
        self.declare_parameter("omega_gripper_closed_gap", 0.0)
        self.declare_parameter("omega_gripper_open_gap", 0.025)
        self.declare_parameter("robot_gripper_closed_width", 0.0)
        self.declare_parameter("robot_gripper_open_width", 0.08)
        self.declare_parameter("gripper_speed", 0.08)
        self.declare_parameter("gripper_command_period", 0.15)
        self.declare_parameter("gripper_width_deadband", 0.002)

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _read_parameters(self):
        self.sdk_library = str(self._parameter("sdk_library"))
        self.require_omega7 = bool(self._parameter("require_omega7"))
        self.fake_hardware = bool(self._parameter("fake_hardware"))
        self.base_frame = str(self._parameter("base_frame"))
        self.ee_link = str(self._parameter("ee_link"))
        self.arm_group = str(self._parameter("arm_group"))
        self.ik_service = str(self._parameter("ik_service"))
        self.trajectory_topic = str(self._parameter("trajectory_topic"))
        self.gripper_action = str(self._parameter("gripper_action"))
        self.device_rate_hz = float(self._parameter("device_rate_hz"))
        self.command_rate_hz = float(self._parameter("command_rate_hz"))
        self.command_duration = float(self._parameter("command_duration"))
        self.device_timeout = float(self._parameter("device_timeout"))
        self.joint_state_timeout = float(self._parameter("joint_state_timeout"))
        self.ik_timeout = float(self._parameter("ik_timeout"))
        self.max_ik_failures = int(self._parameter("max_ik_failures"))
        self.max_raw_position = float(self._parameter("max_raw_position"))
        self.position_scale = tuple(float(value) for value in self._parameter("position_scale"))
        mapping = tuple(float(value) for value in self._parameter("axis_mapping"))
        self.axis_mapping = tuple(
            tuple(mapping[row * 3 + column] for column in range(3))
            for row in range(3)
        )
        self.orientation_scale = float(self._parameter("orientation_scale"))
        self.position_deadzone = float(self._parameter("position_deadzone"))
        self.orientation_deadzone = float(self._parameter("orientation_deadzone"))
        self.max_linear_speed = float(self._parameter("max_linear_speed"))
        self.max_angular_speed = float(self._parameter("max_angular_speed"))
        self.workspace_min = tuple(float(value) for value in self._parameter("workspace_min"))
        self.workspace_max = tuple(float(value) for value in self._parameter("workspace_max"))
        self.max_joint_velocity = tuple(
            float(value) for value in self._parameter("max_joint_velocity")
        )
        self.j7_constraint_tolerance = float(
            self._parameter("j7_constraint_tolerance")
        )
        self.omega_gripper_closed_gap = float(
            self._parameter("omega_gripper_closed_gap")
        )
        self.omega_gripper_open_gap = float(self._parameter("omega_gripper_open_gap"))
        self.robot_gripper_closed_width = float(
            self._parameter("robot_gripper_closed_width")
        )
        self.robot_gripper_open_width = float(
            self._parameter("robot_gripper_open_width")
        )
        self.gripper_speed = float(self._parameter("gripper_speed"))
        self.gripper_command_period = float(
            self._parameter("gripper_command_period")
        )
        self.gripper_width_deadband = float(
            self._parameter("gripper_width_deadband")
        )
        self._validate_parameters()

    def _validate_parameters(self):
        vector_parameters = {
            "position_scale": self.position_scale,
            "workspace_min": self.workspace_min,
            "workspace_max": self.workspace_max,
        }
        for name, value in vector_parameters.items():
            if len(value) != 3 or not finite(value):
                raise ValueError(f"{name} must contain three finite values")
        if len(self.max_joint_velocity) != 7 or not finite(self.max_joint_velocity):
            raise ValueError("max_joint_velocity must contain seven finite values")
        if any(
            self.workspace_min[index] >= self.workspace_max[index] for index in range(3)
        ):
            raise ValueError("workspace_min must be strictly below workspace_max")
        if not 1.0 <= self.device_rate_hz <= 2000.0:
            raise ValueError("device_rate_hz must be in [1, 2000]")
        if not 1.0 <= self.command_rate_hz <= 200.0:
            raise ValueError("command_rate_hz must be in [1, 200]")
        if self.omega_gripper_open_gap <= self.omega_gripper_closed_gap:
            raise ValueError("omega gripper open gap must exceed closed gap")
        identity = matrix_multiply(self.axis_mapping, matrix_transpose(self.axis_mapping))
        error = max(
            abs(identity[row][column] - (1.0 if row == column else 0.0))
            for row in range(3)
            for column in range(3)
        )
        if error > 1.0e-6:
            raise ValueError("axis_mapping must be an orthonormal 3x3 matrix")

    def _log_throttled(self, key, message, period=2.0, level="warning"):
        now = time.monotonic()
        if now - self._last_log_times.get(key, 0.0) < period:
            return
        self._last_log_times[key] = now
        getattr(self.get_logger(), level)(message)

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in self.JOINT_NAMES):
            return
        joints = tuple(float(positions[name]) for name in self.JOINT_NAMES)
        if not finite(joints):
            self._deactivate("non-finite FR3 joint state")
            return
        self._current_joints = joints
        self._last_joint_state_time = time.monotonic()
        if self._fixed_j7 is None:
            self._fixed_j7 = joints[6]
            self.get_logger().info(f"Locked J7 at startup angle {joints[6]:.6f} rad")

    def _try_open_device(self):
        now = time.monotonic()
        if now - self._last_open_attempt < 1.0:
            return
        self._last_open_attempt = now
        try:
            self._device = DhdDevice(self.sdk_library, self.require_omega7)
            name = self._device.connect()
            self._release_seen = False
            self.get_logger().info(f"Connected to {name}; release the gripper before use")
        except DhdError as error:
            self._device = None
            self._log_throttled("open", str(error), period=5.0)

    def _sample_device(self):
        if self._device is None or not self._device.connected:
            self._try_open_device()
            return
        try:
            sample = self._device.sample()
            if max(abs(value) for value in sample.position) > self.max_raw_position:
                raise DhdError(
                    f"raw position exceeded {self.max_raw_position:.3f} m sanity limit"
                )
            self._latest_sample = sample
            self._last_sample_time = time.monotonic()
        except (DhdError, ValueError) as error:
            self._disconnect_device(str(error))
            return

        self._gripper_requested_width = self._map_gripper(sample.gripper_gap)
        if not sample.enabled:
            if self._active:
                self._deactivate("Omega.7 gripper released")
            self._release_seen = True
            self._gripper_requested_width = self.robot_gripper_open_width
            self._cancel_gripper_motion()
            self._maybe_send_gripper()

    def _disconnect_device(self, reason):
        self._deactivate(f"Omega.7 data/connection fault: {reason}")
        if self._device is not None:
            try:
                self._device.close()
            except Exception as error:  # best effort after a communication fault
                self._log_throttled("close", f"DHD close failed: {error}")
        self._device = None
        self._latest_sample = None
        self._last_sample_time = None
        self._release_seen = False

    def _map_gripper(self, gap):
        fraction = (gap - self.omega_gripper_closed_gap) / (
            self.omega_gripper_open_gap - self.omega_gripper_closed_gap
        )
        fraction = clamp(fraction, 0.0, 1.0)
        return self.robot_gripper_closed_width + fraction * (
            self.robot_gripper_open_width - self.robot_gripper_closed_width
        )

    def _lookup_ee_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self.base_frame,
            self.ee_link,
            Time(),
            timeout=RclpyDuration(seconds=0.05),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _engage(self):
        if self._latest_sample is None or self._current_joints is None:
            return False
        if self._fixed_j7 is None or not self._release_seen:
            return False
        if self._last_joint_state_time is None or (
            time.monotonic() - self._last_joint_state_time > self.joint_state_timeout
        ):
            self._log_throttled("joint_stale", "Cannot engage: /joint_states is stale")
            return False
        if not self._ik_client.service_is_ready():
            self._log_throttled("ik_service", "Cannot engage: /compute_ik is unavailable")
            return False
        try:
            robot_pose = self._lookup_ee_pose()
        except TransformException as error:
            self._log_throttled("tf", f"Cannot engage: end-effector TF unavailable: {error}")
            return False
        self._master_origin = (
            self._latest_sample.position,
            self._latest_sample.quaternion,
        )
        self._robot_origin = robot_pose
        self._limited_target = robot_pose
        self._active = True
        self._generation += 1
        self._ik_failure_count = 0
        self._recalibrate_requested = False
        self.get_logger().info("Teleoperation engaged; position and orientation zero captured")
        return True

    def _command_tick(self):
        if self._latest_sample is None:
            return
        if not self._latest_sample.enabled:
            return
        if not self._active and not self._engage():
            return
        if self._recalibrate_requested:
            self._deactivate("recalibration requested")
            self._release_seen = False
            return
        if self._ik_pending:
            self._maybe_send_gripper()
            return

        master_origin_position, master_origin_quaternion = self._master_origin
        robot_origin_position, robot_origin_quaternion = self._robot_origin
        desired_position, desired_quaternion = mapped_relative_target(
            master_origin_position,
            master_origin_quaternion,
            self._latest_sample.position,
            self._latest_sample.quaternion,
            robot_origin_position,
            robot_origin_quaternion,
            self.axis_mapping,
            self.position_scale,
            self.orientation_scale,
            self.position_deadzone,
            self.orientation_deadzone,
        )
        desired_position = clamp_workspace(
            desired_position, self.workspace_min, self.workspace_max
        )
        previous_position, previous_quaternion = self._limited_target
        period = 1.0 / self.command_rate_hz
        target = limit_pose_step(
            previous_position,
            previous_quaternion,
            desired_position,
            desired_quaternion,
            self.max_linear_speed * period,
            self.max_angular_speed * period,
        )
        self._limited_target = target
        self._request_ik(target)
        self._maybe_send_gripper()

    def _request_ik(self, target):
        if self._current_joints is None:
            self._deactivate("no complete joint state")
            return
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.arm_group
        ik.avoid_collisions = True
        ik.ik_link_name = self.ee_link
        ik.robot_state.joint_state.name = list(self.JOINT_NAMES)
        seed = list(self._current_joints)
        seed[6] = self._fixed_j7
        ik.robot_state.joint_state.position = seed
        constraint = JointConstraint()
        constraint.joint_name = self.JOINT_NAMES[6]
        constraint.position = self._fixed_j7
        constraint.tolerance_above = self.j7_constraint_tolerance
        constraint.tolerance_below = self.j7_constraint_tolerance
        constraint.weight = 1.0
        ik.constraints.joint_constraints = [constraint]
        ik.pose_stamped = self._pose_stamped(target)
        seconds = int(self.ik_timeout)
        ik.timeout = Duration(
            sec=seconds,
            nanosec=int((self.ik_timeout - seconds) * 1.0e9),
        )
        generation = self._generation
        self._ik_pending = True
        future = self._ik_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_ik_result(completed, generation)
        )

    def _pose_stamped(self, target):
        position, quaternion = target
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = quaternion
        return message

    def _on_ik_result(self, future, generation):
        if generation != self._generation:
            return
        self._ik_pending = False
        if not self._active:
            return
        try:
            response = future.result()
        except Exception as error:
            self._ik_failed(f"IK service error: {error}")
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._ik_failed(
                f"IK failed with code {response.error_code.val}: "
                f"{response.error_code.message}"
            )
            return
        solution = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(name in solution for name in self.JOINT_NAMES):
            self._ik_failed("IK response omitted one or more FR3 joints")
            return
        target = [float(solution[name]) for name in self.JOINT_NAMES]
        if not finite(target):
            self._deactivate("IK returned non-finite joints")
            return
        if abs(target[6] - self._fixed_j7) > self.j7_constraint_tolerance:
            self._deactivate(
                f"IK violated fixed J7 constraint ({target[6]:.6f} vs "
                f"{self._fixed_j7:.6f} rad)"
            )
            return
        target[6] = self._fixed_j7
        self._ik_failure_count = 0
        self._publish_target(target)

    def _ik_failed(self, reason):
        self._ik_failure_count += 1
        self._log_throttled("ik_failed", reason, period=1.0)
        if self._ik_failure_count >= self.max_ik_failures:
            self._deactivate("repeated IK failure")

    def _publish_target(self, target):
        if self._current_joints is None:
            self._deactivate("joint state disappeared before command")
            return
        limited = []
        for index, desired in enumerate(target):
            if index == 6:
                limited.append(self._fixed_j7)
                continue
            maximum_step = self.max_joint_velocity[index] * self.command_duration
            delta = clamp(
                desired - self._current_joints[index], -maximum_step, maximum_step
            )
            limited.append(self._current_joints[index] + delta)
        self._publish_trajectory(limited)

    def _publish_trajectory(self, positions):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        seconds = int(self.command_duration)
        point.time_from_start = Duration(
            sec=seconds,
            nanosec=int((self.command_duration - seconds) * 1.0e9),
        )
        trajectory.points = [point]
        self._trajectory_publisher.publish(trajectory)

    def _publish_hold(self):
        if self._current_joints is None:
            return
        hold = list(self._current_joints)
        if self._fixed_j7 is not None:
            hold[6] = self._fixed_j7
        self._publish_trajectory(hold)

    def _deactivate(self, reason):
        was_active = self._active
        self._active = False
        self._release_seen = False
        self._generation += 1
        self._ik_pending = False
        self._master_origin = None
        self._robot_origin = None
        self._limited_target = None
        self._cancel_gripper_motion()
        if was_active:
            self._publish_hold()
            self.get_logger().warning(f"Teleoperation stopped: {reason}")

    def _maybe_send_gripper(self):
        now = time.monotonic()
        # A close command is only valid while arm teleoperation is engaged.  An
        # explicit release sample is still allowed to command the fully open width.
        if self._latest_sample is None or (
            self._latest_sample.enabled and not self._active
        ):
            return
        if self._gripper_busy:
            if (
                self._gripper_last_sent_width is not None
                and now - self._last_gripper_send >= self.gripper_command_period
                and abs(
                    self._gripper_requested_width - self._gripper_last_sent_width
                )
                >= self.gripper_width_deadband
            ):
                self._cancel_gripper_motion()
            return
        if now - self._last_gripper_send < self.gripper_command_period:
            return
        if self._gripper_last_sent_width is not None and abs(
            self._gripper_requested_width - self._gripper_last_sent_width
        ) < self.gripper_width_deadband:
            return
        if self.fake_hardware:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = ["fr3_finger_joint1", "fr3_finger_joint2"]
            message.position = [
                0.5 * self._gripper_requested_width,
                0.5 * self._gripper_requested_width,
            ]
            self._fake_gripper_publisher.publish(message)
            self._gripper_last_sent_width = self._gripper_requested_width
            self._last_gripper_send = now
            return
        if not self._gripper_client.server_is_ready():
            self._log_throttled(
                "gripper_action", f"Gripper action unavailable: {self.gripper_action}"
            )
            return
        goal = Move.Goal()
        goal.width = self._gripper_requested_width
        goal.speed = self.gripper_speed
        width = goal.width
        self._gripper_busy = True
        self._last_gripper_send = now
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda completed: self._on_gripper_goal(completed, width))

    def _cancel_gripper_motion(self):
        if not self._gripper_busy or self._gripper_cancel_requested:
            return
        self._gripper_cancel_requested = True
        if self._gripper_goal_handle is not None:
            self._send_gripper_cancel()

    def _send_gripper_cancel(self):
        try:
            future = self._gripper_goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_gripper_cancel)
        except Exception as error:
            self._gripper_cancel_requested = False
            self._log_throttled("gripper_cancel", f"Gripper cancel failed: {error}")

    def _on_gripper_cancel(self, future):
        try:
            future.result()
        except Exception as error:
            self._log_throttled("gripper_cancel", f"Gripper cancel failed: {error}")

    def _on_gripper_goal(self, future, width):
        try:
            handle = future.result()
        except Exception as error:
            self._gripper_busy = False
            self._gripper_cancel_requested = False
            self._log_throttled("gripper_send", f"Gripper goal failed: {error}")
            return
        if not handle.accepted:
            self._gripper_busy = False
            self._gripper_cancel_requested = False
            self._log_throttled("gripper_reject", "Franka gripper rejected move goal")
            return
        self._gripper_goal_handle = handle
        self._gripper_last_sent_width = width
        if self._gripper_cancel_requested:
            self._send_gripper_cancel()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future):
        cancellation_requested = self._gripper_cancel_requested
        self._gripper_busy = False
        self._gripper_goal_handle = None
        self._gripper_cancel_requested = False
        try:
            result = future.result().result
            if not result.success and not cancellation_requested:
                self._log_throttled(
                    "gripper_result", f"Gripper move failed: {result.error}"
                )
        except Exception as error:
            self._log_throttled("gripper_result", f"Gripper result failed: {error}")
        self._maybe_send_gripper()

    def _watchdog_tick(self):
        if not self._active:
            return
        now = time.monotonic()
        if self._last_sample_time is None or now - self._last_sample_time > self.device_timeout:
            self._disconnect_device("sample watchdog timeout")
            return
        if (
            self._last_joint_state_time is None
            or now - self._last_joint_state_time > self.joint_state_timeout
        ):
            self._deactivate("FR3 joint-state watchdog timeout")

    def _on_recalibrate(self, _request, response):
        self._recalibrate_requested = True
        if self._active:
            self._deactivate("recalibration requested")
        self._release_seen = False
        response.success = True
        response.message = "Release and squeeze the Omega.7 gripper to capture new zeros"
        return response

    def destroy_node(self):
        self._deactivate("node shutdown")
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Omega7Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
