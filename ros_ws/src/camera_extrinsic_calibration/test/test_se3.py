import numpy as np
from scipy.spatial.transform import Rotation

from camera_extrinsic_calibration.se3 import (
    fuse_transforms,
    inverse,
    pose_spread,
    reject_outliers,
)


def transform(translation, yaw_degrees=0.0):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", yaw_degrees, degrees=True).as_matrix()
    result[:3, 3] = translation
    return result


def test_inverse_round_trip():
    value = transform([0.4, -0.2, 1.1], 37.0)
    assert np.allclose(value @ inverse(value), np.eye(4))


def test_rotation_mean_wraps_at_180_degrees():
    fused = fuse_transforms(
        [transform([0, 0, 0], 179.0), transform([2, 0, 0], -179.0)],
        [1.0, 1.0],
    )
    assert np.allclose(fused[:3, 3], [1.0, 0.0, 0.0])
    assert abs(abs(Rotation.from_matrix(fused[:3, :3]).as_euler("zyx", degrees=True)[0])
               - 180.0) < 1e-6


def test_rejects_distant_pose():
    poses = [
        transform([0.0, 0.0, 1.0], 0.0),
        transform([0.004, 0.0, 1.0], 0.5),
        transform([0.3, 0.0, 1.0], 20.0),
    ]
    assert reject_outliers(poses, 0.02, 2.0) == [0, 1]
    fused = fuse_transforms(poses[:2], [1.0, 1.0])
    translation, rotation = pose_spread(poses[:2], fused)
    assert translation < 0.003
    assert rotation < 0.3
