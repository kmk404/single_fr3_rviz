"""Camera-model-independent ArUco extrinsic estimator."""

from dataclasses import dataclass

import cv2
import numpy as np

from .se3 import fuse_transforms, inverse, pose_spread, reject_outliers


@dataclass
class MarkerEstimate:
    marker_id: int
    corners: np.ndarray
    camera_from_marker: np.ndarray
    table_from_camera: np.ndarray
    reprojection_error_px: float


def marker_object_points(length):
    half = length / 2.0
    # Required order for SOLVEPNP_IPPE_SQUARE: TL, TR, BR, BL. Marker +y points
    # toward its printed bottom and +z therefore points into a face-up table.
    return np.array([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ], dtype=np.float64)


def estimate_marker(marker_id, corners, marker_length, camera_matrix, distortion,
                    table_from_marker):
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    object_points = marker_object_points(marker_length)
    success, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success or len(rvecs) == 0:
        return None
    valid = [index for index, value in enumerate(tvecs) if value.reshape(3)[2] > 0.0]
    if not valid:
        return None
    error_values = np.asarray(errors, dtype=float).reshape(-1)
    selected = min(valid, key=lambda index: error_values[index])
    rotation, _ = cv2.Rodrigues(rvecs[selected])
    camera_from_marker = np.eye(4)
    camera_from_marker[:3, :3] = rotation
    camera_from_marker[:3, 3] = tvecs[selected].reshape(3)
    projected, _ = cv2.projectPoints(
        object_points, rvecs[selected], tvecs[selected], camera_matrix, distortion
    )
    residual = projected.reshape(4, 2) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return MarkerEstimate(
        marker_id=marker_id,
        corners=image_points,
        camera_from_marker=camera_from_marker,
        table_from_camera=table_from_marker @ inverse(camera_from_marker),
        reprojection_error_px=rms,
    )


def fuse_estimates(estimates, quality):
    reprojection_limit = float(quality["max_marker_reprojection_error_px"])
    candidates = [item for item in estimates if item.reprojection_error_px <= reprojection_limit]
    transforms = [item.table_from_camera for item in candidates]
    inlier_indices = reject_outliers(
        transforms,
        float(quality["outlier_translation_threshold_m"]),
        float(quality["outlier_rotation_threshold_deg"]),
    )
    inliers = [candidates[index] for index in inlier_indices]
    if not inliers:
        return None
    weights = [1.0 / max(item.reprojection_error_px, 0.05) ** 2 for item in inliers]
    fused = fuse_transforms([item.table_from_camera for item in inliers], weights)
    translation_spread, rotation_spread = pose_spread(
        [item.table_from_camera for item in inliers], fused
    )
    weighted_error = float(np.average(
        [item.reprojection_error_px for item in inliers], weights=weights
    ))
    valid = (
        weighted_error <= float(quality["max_fused_reprojection_error_px"])
        and translation_spread <= float(quality["max_translation_spread_m"])
        and rotation_spread <= float(quality["max_rotation_spread_deg"])
    )
    confidence = min(1.0, len(inliers) / 2.0)
    confidence *= max(0.0, 1.0 - weighted_error / reprojection_limit)
    confidence *= max(0.0, 1.0 - translation_spread /
                      float(quality["outlier_translation_threshold_m"]))
    confidence *= max(0.0, 1.0 - rotation_spread /
                      float(quality["outlier_rotation_threshold_deg"]))
    return {
        "table_from_camera": fused,
        "inliers": inliers,
        "reprojection_error": weighted_error,
        "translation_spread": translation_spread,
        "rotation_spread_deg": rotation_spread,
        "calibration_valid": bool(valid),
        "confidence": float(confidence),
    }
