"""Small SE(3) composition layer; SO(3) operations are delegated to SciPy."""

import numpy as np
from scipy.spatial.transform import Rotation


def as_transform(values):
    matrix = np.asarray(values, dtype=float).reshape(4, 4)
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("homogeneous transform has an invalid last row")
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6):
        raise ValueError("transform rotation is not orthonormal")
    if np.linalg.det(matrix[:3, :3]) < 0.999999:
        raise ValueError("transform rotation is not right-handed")
    return matrix


def inverse(transform):
    result = np.eye(4)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -(rotation.T @ transform[:3, 3])
    return result


def rotation_distance_deg(first, second):
    delta = Rotation.from_matrix(first).inv() * Rotation.from_matrix(second)
    return float(np.degrees(delta.magnitude()))


def fuse_transforms(transforms, weights):
    if not transforms:
        raise ValueError("cannot fuse an empty transform list")
    normalized = np.asarray(weights, dtype=float)
    normalized /= normalized.sum()
    result = np.eye(4)
    result[:3, 3] = np.average(
        np.asarray([item[:3, 3] for item in transforms]), axis=0, weights=normalized
    )
    rotations = Rotation.from_matrix([item[:3, :3] for item in transforms])
    result[:3, :3] = rotations.mean(weights=normalized).as_matrix()
    return result


def reject_outliers(transforms, translation_threshold, rotation_threshold_deg):
    """Keep the largest mutually consistent camera-pose consensus set."""
    if len(transforms) <= 1:
        return list(range(len(transforms)))
    best = []
    for anchor, candidate in enumerate(transforms):
        consensus = []
        for index, other in enumerate(transforms):
            translation = np.linalg.norm(candidate[:3, 3] - other[:3, 3])
            rotation = rotation_distance_deg(candidate[:3, :3], other[:3, :3])
            if translation <= translation_threshold and rotation <= rotation_threshold_deg:
                consensus.append(index)
        if len(consensus) > len(best):
            best = consensus
    return best


def pose_spread(transforms, fused):
    if not transforms:
        return float("inf"), float("inf")
    translations = [np.linalg.norm(item[:3, 3] - fused[:3, 3]) for item in transforms]
    rotations = [
        rotation_distance_deg(fused[:3, :3], item[:3, :3]) for item in transforms
    ]
    return float(max(translations)), float(max(rotations))
