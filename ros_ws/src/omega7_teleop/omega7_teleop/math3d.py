"""Small dependency-free 3D helpers used by the teleoperation safety path."""

import math


def finite(values):
    return all(math.isfinite(value) for value in values)


def clamp(value, low, high):
    return min(max(value, low), high)


def vector_norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def matrix_transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def matrix_multiply(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def matrix_vector(matrix, vector):
    return tuple(sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3))


def quaternion_normalize(quaternion):
    norm = vector_norm(quaternion)
    if norm < 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return quaternion_normalize(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion_normalize(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        quaternion = (
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        quaternion = (
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    return quaternion_normalize(quaternion)


def quaternion_rotate(quaternion, vector):
    return matrix_vector(quaternion_to_matrix(quaternion), vector)


def quaternion_scaled(quaternion, scale, deadzone=0.0):
    x, y, z, w = quaternion_normalize(quaternion)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), clamp(w, -1.0, 1.0))
    if angle <= deadzone:
        return (0.0, 0.0, 0.0, 1.0)
    scaled_angle = (angle - deadzone) * scale
    axis_norm = math.sqrt(x * x + y * y + z * z)
    if axis_norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    sine = math.sin(0.5 * scaled_angle)
    return (
        x / axis_norm * sine,
        y / axis_norm * sine,
        z / axis_norm * sine,
        math.cos(0.5 * scaled_angle),
    )


def vector_deadzone(vector, deadzone):
    norm = vector_norm(vector)
    if norm <= deadzone:
        return (0.0, 0.0, 0.0)
    scale = (norm - deadzone) / norm
    return tuple(value * scale for value in vector)


def mapped_relative_target(
    master_origin_position,
    master_origin_quaternion,
    master_position,
    master_quaternion,
    robot_origin_position,
    robot_origin_quaternion,
    axis_mapping,
    position_scale,
    orientation_scale,
    position_deadzone,
    orientation_deadzone,
):
    """Map Omega motion in its calibrated local frame into the FR3 tool frame."""
    master_origin_inverse = quaternion_conjugate(master_origin_quaternion)
    delta_world = tuple(
        master_position[index] - master_origin_position[index] for index in range(3)
    )
    delta_master_local = quaternion_rotate(master_origin_inverse, delta_world)
    delta_master_local = vector_deadzone(delta_master_local, position_deadzone)
    delta_robot_local = matrix_vector(axis_mapping, delta_master_local)
    delta_robot_local = tuple(
        delta_robot_local[index] * position_scale[index] for index in range(3)
    )
    delta_robot_world = quaternion_rotate(robot_origin_quaternion, delta_robot_local)
    target_position = tuple(
        robot_origin_position[index] + delta_robot_world[index] for index in range(3)
    )

    delta_master_rotation = quaternion_multiply(master_origin_inverse, master_quaternion)
    delta_matrix = quaternion_to_matrix(delta_master_rotation)
    mapped_matrix = matrix_multiply(
        matrix_multiply(axis_mapping, delta_matrix), matrix_transpose(axis_mapping)
    )
    mapped_delta = quaternion_scaled(
        matrix_to_quaternion(mapped_matrix), orientation_scale, orientation_deadzone
    )
    target_quaternion = quaternion_multiply(robot_origin_quaternion, mapped_delta)
    return target_position, target_quaternion


def clamp_workspace(position, minimum, maximum):
    return tuple(clamp(position[index], minimum[index], maximum[index]) for index in range(3))


def limit_pose_step(
    previous_position,
    previous_quaternion,
    desired_position,
    desired_quaternion,
    max_linear_step,
    max_angular_step,
):
    delta = tuple(desired_position[index] - previous_position[index] for index in range(3))
    distance = vector_norm(delta)
    if distance > max_linear_step > 0.0:
        ratio = max_linear_step / distance
        position = tuple(previous_position[index] + delta[index] * ratio for index in range(3))
    else:
        position = desired_position

    relative = quaternion_multiply(quaternion_conjugate(previous_quaternion), desired_quaternion)
    x, y, z, w = relative
    angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))
    if angle > max_angular_step > 0.0:
        relative = quaternion_scaled(relative, max_angular_step / angle)
        quaternion = quaternion_multiply(previous_quaternion, relative)
    else:
        quaternion = desired_quaternion
    return position, quaternion
