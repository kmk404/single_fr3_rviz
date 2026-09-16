import cv2
import numpy as np

from camera_extrinsic_calibration.calibrator import estimate_marker, marker_object_points


def test_pnp_recovers_camera_from_marker_and_table_camera():
    camera_matrix = np.array([[700.0, 0.0, 640.0], [0.0, 700.0, 360.0], [0, 0, 1]])
    distortion = np.zeros(5)
    rvec = np.array([0.08, -0.04, 0.1])
    tvec = np.array([0.03, -0.02, 0.8])
    corners, _ = cv2.projectPoints(
        marker_object_points(0.077), rvec, tvec, camera_matrix, distortion
    )
    estimate = estimate_marker(
        1, corners, 0.077, camera_matrix, distortion, np.eye(4)
    )
    expected_rotation, _ = cv2.Rodrigues(rvec)
    assert estimate is not None
    assert np.allclose(estimate.camera_from_marker[:3, :3], expected_rotation, atol=1e-6)
    assert np.allclose(estimate.camera_from_marker[:3, 3], tvec, atol=1e-6)
    assert np.allclose(
        estimate.table_from_camera @ estimate.camera_from_marker, np.eye(4), atol=1e-6
    )
    assert estimate.reprojection_error_px < 1e-6
