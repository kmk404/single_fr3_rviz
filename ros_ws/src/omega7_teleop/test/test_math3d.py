import math

from omega7_teleop.math3d import (
    clamp_workspace,
    limit_pose_step,
    mapped_relative_target,
    quaternion_to_matrix,
)
import pytest


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
IDENTITY_MATRIX = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def test_relative_translation_uses_calibrated_tool_axes():
    half_angle = math.pi / 4.0
    robot_rotation = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    position, _ = mapped_relative_target(
        (0.0, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (0.10, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (0.4, 0.0, 0.3),
        robot_rotation,
        IDENTITY_MATRIX,
        (0.5, 1.0, 1.0),
        1.0,
        0.0,
        0.0,
    )
    assert position == pytest.approx((0.4, 0.05, 0.3))


def test_relative_rotation_maps_one_to_one():
    half_angle = math.pi / 8.0
    master_rotation = (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle))
    _, quaternion = mapped_relative_target(
        (0.0, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (0.0, 0.0, 0.0),
        master_rotation,
        (0.4, 0.0, 0.3),
        IDENTITY_QUATERNION,
        IDENTITY_MATRIX,
        (1.0, 1.0, 1.0),
        1.0,
        0.0,
        0.0,
    )
    actual = tuple(value for row in quaternion_to_matrix(quaternion) for value in row)
    expected = tuple(
        value for row in quaternion_to_matrix(master_rotation) for value in row
    )
    assert actual == pytest.approx(expected)


def test_deadzone_workspace_and_rate_limits_are_applied():
    position, quaternion = mapped_relative_target(
        (0.0, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (0.0005, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (0.4, 0.0, 0.3),
        IDENTITY_QUATERNION,
        IDENTITY_MATRIX,
        (1.0, 1.0, 1.0),
        1.0,
        0.001,
        0.01,
    )
    assert position == pytest.approx((0.4, 0.0, 0.3))
    assert quaternion == pytest.approx(IDENTITY_QUATERNION)

    clamped = clamp_workspace((1.0, -1.0, 0.3), (0.1, -0.5, 0.05), (0.75, 0.5, 0.9))
    assert clamped == pytest.approx((0.75, -0.5, 0.3))

    limited_position, _ = limit_pose_step(
        (0.0, 0.0, 0.0),
        IDENTITY_QUATERNION,
        (1.0, 0.0, 0.0),
        IDENTITY_QUATERNION,
        0.02,
        0.1,
    )
    assert limited_position == pytest.approx((0.02, 0.0, 0.0))
