"""Exercise the high-rate IK-to-servo target path without robot hardware."""

import time
from types import SimpleNamespace

from moveit_msgs.msg import MoveItErrorCodes
import pytest
import rclpy

from omega7_fresh.node import Teleop


def ik_result(node, positions):
    result = SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
        solution=SimpleNamespace(
            joint_state=SimpleNamespace(name=node.JOINTS, position=positions)
        ),
    )
    return SimpleNamespace(result=lambda: result)


def test_joint_target_uses_full_configured_rate_limit():
    rclpy.init()
    node = Teleop()
    try:
        assert node.gate.enable(True)[0]
        initial = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7)
        node.joints = initial
        node.hold_q = initial
        node.pending_pose = ((0.4, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        node.hold_q_at = time.monotonic() - 0.1
        target = list(initial)
        target[1] += 0.05
        published = []
        node.send_arm_goal = lambda positions: published.append(tuple(positions))

        node.on_ik(ik_result(node, target), node.gate.generation)

        expected_step = node.p["max_joint_velocity"][1] * 0.1
        assert node.hold_q[1] - initial[1] == pytest.approx(expected_step, abs=1e-5)
        assert published[-1] == node.hold_q
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_raw_ik_path_can_lead_rate_limited_hold_target():
    rclpy.init()
    node = Teleop()
    try:
        assert node.gate.enable(True)[0]
        initial = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7)
        node.joints = initial
        node.hold_q = initial
        previous_raw = list(initial)
        previous_raw[1] = 0.10
        node.last_ik_solution = tuple(previous_raw)
        node.pending_pose = ((0.4, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        node.hold_q_at = time.monotonic() - 0.1
        target = list(initial)
        target[1] = 0.21  # 0.11 rad from raw IK, but > 0.12 rad from hold_q.
        published = []
        node.send_arm_goal = lambda positions: published.append(tuple(positions))

        node.on_ik(ik_result(node, target), node.gate.generation)

        assert node.gate.enabled
        assert node.last_ik_solution == tuple(target)
        expected_step = node.p["max_joint_velocity"][1] * 0.1
        assert node.hold_q[1] == pytest.approx(expected_step, abs=1e-5)
        assert published[-1] == node.hold_q
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_branch_jump_retries_then_holds_without_disabling():
    rclpy.init()
    node = Teleop()
    try:
        assert node.gate.enable(True)[0]
        initial = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7)
        node.joints = initial
        node.joints_at = time.monotonic()
        node.hold_q = initial
        node.last_ik_solution = initial
        node.limited_pose = ((0.35, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        node.pending_pose = ((0.4, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        target = list(initial)
        target[2] += 0.2
        retries = []
        published = []
        node.send_ik_request = lambda pose, seed, generation, retried: retries.append(
            (pose, seed, generation, retried)
        )
        node.send_arm_goal = lambda positions: published.append(tuple(positions))

        node.on_ik(ik_result(node, target), node.gate.generation)
        assert retries and retries[0][1] == initial and retries[0][3]

        node.on_ik(ik_result(node, target), node.gate.generation, retried=True)
        assert node.gate.enabled
        assert node.hold_q == initial
        assert node.ik_branch_rejections == 1
        assert published == [initial]
        assert node.limited_pose == ((0.35, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        assert node.retry_pose == ((0.4, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_transient_ik_error_refreshes_last_valid_hold_target():
    rclpy.init()
    node = Teleop()
    try:
        assert node.gate.enable(True)[0]
        initial = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7)
        node.hold_q = initial
        node.last_ik_solution = initial
        node.pending_pose = ((0.4, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
        published = []
        node.send_arm_goal = lambda positions: published.append(tuple(positions))
        failure = SimpleNamespace(result=lambda: (_ for _ in ()).throw(
            RuntimeError("temporary IK transport error")))

        node.on_ik(failure, node.gate.generation)

        assert node.gate.enabled
        assert node.ik_failures == 1
        assert published == [initial]
    finally:
        node.destroy_node()
        rclpy.shutdown()
