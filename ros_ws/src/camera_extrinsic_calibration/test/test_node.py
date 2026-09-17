"""ROS front-end integration with synthetic detections; no physical camera needed."""

from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
from sensor_msgs.msg import CameraInfo  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from camera_extrinsic_calibration.calibrator_node import (  # noqa: E402
    ExtrinsicCalibratorNode, atomic_yaml,
)
from camera_extrinsic_calibration.se3 import inverse  # noqa: E402


@pytest.fixture
def node(tmp_path):
    layout = Path(__file__).parents[1] / 'config/extrinsic_calibration.yaml'
    rclpy.init(args=['--ros-args', '-p', f'layout_file:={layout}',
                     '-p', f'output_yaml:={tmp_path / "result.yaml"}',
                     '-p', 'expected_optical_frame:=left', '-p', 'debug:=false'])
    instance = ExtrinsicCalibratorNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def data(node, index):
    k = np.array([[528.8273, 0, 630.107], [0, 528.8273, 356.424], [0, 0, 1.]])
    table_camera = np.eye(4)
    table_camera[:3, :3] = Rotation.from_euler('xyz', [155, 8, -5], degrees=True).as_matrix()
    table_camera[:3, 3] = [.38, -.2, 1.2]
    pose = inverse(table_camera)
    ids = sorted(node.objects)
    corners = [cv2.projectPoints(node.objects[i], cv2.Rodrigues(pose[:3, :3])[0],
                                 pose[:3, 3], k, np.zeros(5))[0].reshape(1, 4, 2) for i in ids]
    node.detect = lambda gray: (corners, np.array(ids).reshape(-1, 1), [])
    image = node.bridge.cv2_to_imgmsg(np.zeros((720, 1280, 3), dtype=np.uint8), encoding='bgr8')
    image.header.frame_id = 'left'
    timestamp = 1_000_000_000 + index * 70_000_000
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
    info = CameraInfo()
    info.header = image.header
    info.width, info.height = 1280, 720
    info.p = np.c_[k, np.zeros(3)].ravel().tolist()
    info.k = k.ravel().tolist()
    info.r = np.eye(3).ravel().tolist()
    return image, info


def feed(node, index):
    image, info = data(node, index)
    if index % 2:
        node._image_callback(image)
        node._camera_info_callback(info)
    else:
        node._camera_info_callback(info)
        node._image_callback(image)


def test_node_save_lock_recollect_and_metadata(node):
    for i in range(29):
        feed(node, i)
    path = Path(node.output_path)
    assert not path.exists()
    feed(node, 29)
    assert path.exists() and node.window.state == 'saved'
    original = path.read_bytes()
    saved = yaml.safe_load(original)
    assert saved['stability']['window_frames'] == 30
    assert saved['stability']['window_duration_sec'] >= 2
    assert saved['stability']['state'] == 'saved'
    assert saved['image_size'] == [1280, 720]
    assert saved['layout_snapshot'] == node.layout
    assert saved['table_to_base_is_approximate']
    assert np.allclose(np.array(saved['T_camera_table']) @ saved['T_table_camera'], np.eye(4))
    assert len(saved['per_frame_metrics']) == 30
    feed(node, 30)
    assert path.read_bytes() == original
    response = node._recollect(Trigger.Request(), Trigger.Response())
    assert response.success and node.window.state == 'waiting'
    feed(node, 31)
    assert len(node.window.frames) == 1 and path.read_bytes() == original


def test_node_sync_size_and_watchdog_reset(node):
    feed(node, 0)
    image, info = data(node, 1)
    # Never reuse the previous CameraInfo for a later frame.
    node._image_callback(image)
    assert len(node.window.frames) == 1
    info.width = 2560
    node._camera_info_callback(info)
    assert not node.window.frames
    feed(node, 2)
    assert len(node.window.frames) == 1
    node.last_pair_wall -= 1
    node._watchdog()
    assert not node.window.frames
    assert not Path(node.output_path).exists()


def test_failed_atomic_save_preserves_previous_yaml_and_rearms(node):
    path = Path(node.output_path)
    path.write_text('previous: true\n')
    for i in range(29):
        feed(node, i)
    with patch('camera_extrinsic_calibration.calibrator_node.os.replace',
               side_effect=OSError('disk error')):
        feed(node, 29)
    assert path.read_text() == 'previous: true\n'
    assert node.window.state == 'waiting'
    assert list(path.parent.iterdir()) == [path]


def test_atomic_yaml(tmp_path):
    path = tmp_path / 'nested' / 'result.yaml'
    atomic_yaml(str(path), {'valid': True})
    assert yaml.safe_load(path.read_text()) == {'valid': True}


def test_rectification_rotation_and_saved_optical_frame(node):
    rotation = Rotation.from_euler('y', 2, degrees=True).as_matrix()
    for i in range(30):
        image, info = data(node, i)
        info.r = rotation.ravel().tolist()
        node._image_callback(image)
        node._camera_info_callback(info)
    saved = yaml.safe_load(Path(node.output_path).read_text())
    rect_from_optical = np.eye(4)
    rect_from_optical[:3, :3] = rotation
    assert np.allclose(saved['T_table_camera'],
                       inverse(np.array(saved['T_rectified_camera_table'])) @ rect_from_optical)
    assert np.allclose(saved['T_base_camera'],
                       inverse(node.table_from_base) @ np.array(saved['T_table_camera']))


def test_detector_and_debug_keep_raw_and_rejected_quads(node):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    raw = np.full((720, 1280, 3), 255, dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 150)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 0, 150)
    raw[300:450, 850:1000] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    before = raw.copy()
    corners, ids, rejected = node.detect(cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY))
    assert ids.reshape(-1).tolist() == [0]
    from camera_extrinsic_calibration.calibrator import failure
    rejected_quad = np.array([[[10, 50], [30, 50], [30, 70], [10, 70]]], dtype=float)
    debug = node._draw_debug(raw, corners, [0], [rejected_quad], failure('single_board', [0]))
    assert np.array_equal(raw, before)
    assert not np.array_equal(debug, raw)
    assert np.array_equal(debug[50, 10], [255, 0, 255])
