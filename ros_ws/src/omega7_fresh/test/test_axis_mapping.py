import math
from pathlib import Path

import pytest
import yaml

from omega7_fresh.math3d import (
    OMEGA_RAW_TO_FR3_BASE_AXES,
    mapped_relative_target_with_diagnostics,
    matrix_multiply,
    matrix_transpose,
    matrix_vector,
    quaternion_multiply,
    quaternion_to_matrix,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def axis_angle(axis, angle):
    sine = math.sin(angle / 2.0)
    return tuple(component * sine for component in axis) + (math.cos(angle / 2.0),)


def flatten_matrix(matrix):
    return tuple(value for row in matrix for value in row)


def map_pose(
    position,
    quaternion=IDENTITY,
    master_zero_quaternion=IDENTITY,
    robot_zero_quaternion=IDENTITY,
    position_scale=(1.0, 1.0, 1.0),
):
    return mapped_relative_target_with_diagnostics(
        (0.0, 0.0, 0.0),
        master_zero_quaternion,
        position,
        quaternion,
        (0.0, 0.0, 0.0),
        robot_zero_quaternion,
        OMEGA_RAW_TO_FR3_BASE_AXES,
        position_scale,
        1.0,
        0.0,
        0.0,
    )


def test_axis_mapping_is_the_configured_right_handed_transform():
    config_path = Path(__file__).parents[1] / "config" / "omega7_fresh.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    configured = config["omega7_fresh"]["ros__parameters"]["axis_mapping"]
    expected = [value for row in OMEGA_RAW_TO_FR3_BASE_AXES for value in row]

    assert configured == pytest.approx(expected)
    assert matrix_vector(OMEGA_RAW_TO_FR3_BASE_AXES, (1.0, 0.0, 0.0)) == pytest.approx(
        (1.0, 0.0, 0.0)
    )
    assert matrix_vector(OMEGA_RAW_TO_FR3_BASE_AXES, (0.0, 1.0, 0.0)) == pytest.approx(
        (0.0, 1.0, 0.0)
    )
    assert matrix_vector(OMEGA_RAW_TO_FR3_BASE_AXES, (0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 0.0, 1.0)
    )

    a = OMEGA_RAW_TO_FR3_BASE_AXES
    determinant = (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )
    assert determinant == pytest.approx(1.0)


def test_configured_position_scale_expands_lateral_and_vertical_translation():
    config_path = Path(__file__).parents[1] / "config" / "omega7_fresh.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
        "omega7_fresh"]["ros__parameters"]
    configured_scale = params["position_scale"]

    assert configured_scale == pytest.approx([2.0, 2.5, 2.5])
    x_result = map_pose((0.01, 0.0, 0.0), position_scale=configured_scale)
    y_result = map_pose((0.0, 0.01, 0.0), position_scale=configured_scale)
    z_result = map_pose((0.0, 0.0, 0.01), position_scale=configured_scale)
    assert x_result[3] == pytest.approx((0.02, 0.0, 0.0))
    assert y_result[3] == pytest.approx((0.0, 0.025, 0.0))
    assert z_result[3] == pytest.approx((0.0, 0.0, 0.025))


@pytest.mark.parametrize(
    ("raw_axis", "expected_base_delta"),
    [
        ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
    ],
)
def test_raw_translation_axes_map_directly_to_fr3_base(raw_axis, expected_base_delta):
    target_position, _, raw_delta, mapped_base_delta = map_pose(raw_axis)

    assert raw_delta == pytest.approx(raw_axis)
    assert mapped_base_delta == pytest.approx(expected_base_delta)
    assert target_position == pytest.approx(expected_base_delta)


def test_translation_is_independent_of_both_enable_zero_orientations():
    omega_zero = axis_angle((0.0, 0.0, 1.0), math.pi / 2.0)
    fr3_zero = axis_angle((0.0, 1.0, 0.0), -0.7)
    raw_delta = (0.001, -0.036, 0.011)
    expected = matrix_vector(OMEGA_RAW_TO_FR3_BASE_AXES, raw_delta)
    result = map_pose(
        raw_delta,
        quaternion=axis_angle((1.0, 0.0, 0.0), 0.3),
        master_zero_quaternion=omega_zero,
        robot_zero_quaternion=fr3_zero,
    )

    assert result[0] == pytest.approx(expected)
    assert result[2] == pytest.approx(raw_delta)
    assert result[3] == pytest.approx(expected)


@pytest.mark.parametrize(
    "space_axis",
    [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)],
)
def test_each_space_rotation_axis_uses_the_same_mapping(space_axis):
    omega_delta = axis_angle(space_axis, math.pi / 2.0)
    _, target, *_ = map_pose((0.0, 0.0, 0.0), omega_delta)
    expected_axis = matrix_vector(OMEGA_RAW_TO_FR3_BASE_AXES, space_axis)
    expected = axis_angle(expected_axis, math.pi / 2.0)

    assert flatten_matrix(quaternion_to_matrix(target)) == pytest.approx(
        flatten_matrix(quaternion_to_matrix(expected))
    )


def test_spatial_rotation_maps_through_a_and_left_multiplies_nonidentity_robot_zero():
    omega_zero = quaternion_multiply(
        axis_angle((0.0, 0.0, 1.0), 0.7),
        axis_angle((0.0, 1.0, 0.0), -0.4),
    )
    fr3_zero = quaternion_multiply(
        axis_angle((1.0, 0.0, 0.0), -0.6),
        axis_angle((0.0, 0.0, 1.0), 0.3),
    )
    space_delta = axis_angle((0.0, 1.0, 0.0), 0.5)
    omega_current = quaternion_multiply(space_delta, omega_zero)

    _, target, *_ = map_pose(
        (0.0, 0.0, 0.0),
        omega_current,
        master_zero_quaternion=omega_zero,
        robot_zero_quaternion=fr3_zero,
    )
    a = OMEGA_RAW_TO_FR3_BASE_AXES
    delta_world_matrix = matrix_multiply(
        quaternion_to_matrix(omega_current),
        matrix_transpose(quaternion_to_matrix(omega_zero)),
    )
    expected_delta_matrix = matrix_multiply(
        matrix_multiply(a, delta_world_matrix), matrix_transpose(a)
    )
    expected_target_matrix = matrix_multiply(
        expected_delta_matrix, quaternion_to_matrix(fr3_zero)
    )

    assert flatten_matrix(quaternion_to_matrix(target)) == pytest.approx(
        flatten_matrix(expected_target_matrix)
    )


def test_upstream_joint_speed_leaves_controller_ruckig_headroom():
    package_dir = Path(__file__).parents[1]
    fresh = yaml.safe_load(
        (package_dir / "config" / "omega7_fresh.yaml").read_text(encoding="utf-8")
    )["omega7_fresh"]["ros__parameters"]
    node_limits = fresh["max_joint_velocity"]

    for controller_config_name in ("controllers.fake.yaml", "controllers.real.yaml"):
        controller = yaml.safe_load(
            (package_dir.parent / "omega7_servo_controller" / "config"
             / controller_config_name).read_text(encoding="utf-8")
        )["/**"]["omega7_servo_controller"]["ros__parameters"]
        assert max(node_limits) < controller["max_velocity"]
        assert controller["max_velocity"] < 0.30
        assert controller["max_following_error"] == pytest.approx(0.08)

    assert node_limits == pytest.approx([0.12] * 6 + [0.10])
    assert fresh["max_servo_following_error"] == pytest.approx(0.08)
