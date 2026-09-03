#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
log_file="$repo_root/reports/fake_mode.log"
mkdir -p "$repo_root/reports"

source /opt/ros/jazzy/setup.bash
source /opt/franka_ws/install/setup.bash
source "$repo_root/ros_ws/install/setup.bash"
set -u

ros2 launch single_fr3_bringup single_fr3_rviz.launch.py \
  use_fake_hardware:=true use_rviz:=false >"$log_file" 2>&1 &
launch_pid=$!
cleanup() {
  kill -TERM "$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 - <<'PY' | tee -a "$log_file"
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node("single_fr3_fake_smoke_test")
client = ActionClient(
    node,
    FollowJointTrajectory,
    "/joint_trajectory_controller/follow_joint_trajectory",
)
try:
    if not client.wait_for_server(timeout_sec=60.0):
        raise RuntimeError("FollowJointTrajectory action server did not appear")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f"fr3_joint{index}" for index in range(1, 8)]
    point = JointTrajectoryPoint()
    point.positions = [0.10, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
    point.time_from_start.sec = 2
    goal.trajectory.points = [point]

    goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
    goal_handle = goal_future.result() if goal_future.done() else None
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("FollowJointTrajectory goal was not accepted")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
    if not result_future.done():
        raise RuntimeError("FollowJointTrajectory goal timed out")
    result = result_future.result().result
    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        raise RuntimeError(
            f"trajectory failed: error_code={result.error_code} {result.error_string}"
        )
    print("ACTION_SERVER=PASS FollowJointTrajectory")
    print(f"TRAJECTORY_RESULT=PASS error_code={result.error_code}")

    move_group_client = ActionClient(node, MoveGroup, "/move_action")
    if not move_group_client.wait_for_server(timeout_sec=30.0):
        raise RuntimeError("MoveGroup action server did not appear")

    move_goal = MoveGroup.Goal()
    move_goal.request.group_name = "fr3_arm"
    move_goal.request.num_planning_attempts = 5
    move_goal.request.allowed_planning_time = 5.0
    move_goal.request.max_velocity_scaling_factor = 0.2
    move_goal.request.max_acceleration_scaling_factor = 0.2
    constraints = Constraints()
    target = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
    for index, position in enumerate(target, start=1):
        joint = JointConstraint()
        joint.joint_name = f"fr3_joint{index}"
        joint.position = position
        joint.tolerance_above = 0.001
        joint.tolerance_below = 0.001
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    move_goal.request.goal_constraints = [constraints]
    move_goal.planning_options.plan_only = False
    move_goal.planning_options.replan = False
    move_goal.planning_options.look_around = False

    move_goal_future = move_group_client.send_goal_async(move_goal)
    rclpy.spin_until_future_complete(node, move_goal_future, timeout_sec=10.0)
    move_goal_handle = move_goal_future.result() if move_goal_future.done() else None
    if move_goal_handle is None or not move_goal_handle.accepted:
        raise RuntimeError("MoveGroup goal was not accepted")

    move_result_future = move_goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, move_result_future, timeout_sec=30.0)
    if not move_result_future.done():
        raise RuntimeError("MoveGroup plan-and-execute timed out")
    move_result = move_result_future.result().result
    if move_result.error_code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"MoveGroup failed: error_code={move_result.error_code.val}"
        )
    print(f"MOVE_GROUP_PLAN_EXECUTE=PASS error_code={move_result.error_code.val}")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

ros2 control list_controllers --spin-time 2 \
  | tee -a "$log_file" | grep -q 'joint_trajectory_controller.*active'
ros2 topic echo /joint_states --no-daemon --timeout 15 --once \
  | tee -a "$log_file" | grep -q 'fr3_joint7'

grep -q "Loaded hardware 'FrankaHardwareInterface' from plugin 'mock_components/GenericSystem'" "$log_file"
if grep -q "franka_hardware/FrankaHardwareInterface" "$log_file"; then
  echo "Fake mode unexpectedly loaded the real hardware plugin" >&2
  exit 1
fi

echo "FAKE_SMOKE_TEST=PASS" | tee -a "$log_file"
