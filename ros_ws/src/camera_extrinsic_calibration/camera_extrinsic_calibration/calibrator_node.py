"""ROS 2 front end for fixed-table multi-marker camera calibration."""

import json
import os
import tempfile
from datetime import datetime, timezone

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from .calibrator import estimate_marker, fuse_estimates
from .se3 import as_transform, inverse


class ExtrinsicCalibratorNode(Node):
    def __init__(self):
        super().__init__("camera_extrinsic_calibrator")
        self.declare_parameter("layout_file", "")
        self.declare_parameter("image_topic", "/zed/zed_node/left/color/rect/image")
        self.declare_parameter(
            "camera_info_topic", "/zed/zed_node/left/color/rect/camera_info"
        )
        self.declare_parameter("expected_optical_frame", "zed_left_camera_frame_optical")
        self.declare_parameter("output_yaml", "camera_extrinsic.yaml")
        self.declare_parameter("debug_image_output", "")
        self.declare_parameter("debug", True)
        self.declare_parameter("save_continuously", False)

        layout_path = self.get_parameter("layout_file").value
        if not layout_path or not os.path.isfile(layout_path):
            raise RuntimeError(f"layout_file does not exist: {layout_path!r}")
        with open(layout_path, "r", encoding="utf-8") as stream:
            self.layout = yaml.safe_load(stream)
        self.marker_length = float(self.layout["aruco"]["marker_length"])
        dictionary_name = self.layout["aruco"]["dictionary"]
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f"unsupported ArUco dictionary: {dictionary_name}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(dictionary, detector_parameters)
            self._detect = self.detector.detectMarkers
        else:
            # Ubuntu's OpenCV 4.5/4.6 exposes the equivalent pre-4.7 API.
            detector_parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self._detect = lambda image: cv2.aruco.detectMarkers(
                image, dictionary, parameters=detector_parameters
            )
        self.table_from_markers = {
            int(marker_id): as_transform(data["T_table_marker"])
            for marker_id, data in self.layout["markers"].items()
        }
        self.table_from_base = as_transform(self.layout["franka"]["T_table_base"])
        self.bridge = CvBridge()
        self.camera_info = None
        self.saved_once = False
        self.debug_enabled = bool(self.get_parameter("debug").value)
        self.quality_pub = self.create_publisher(String, "~/quality", 10)
        self.debug_pub = self.create_publisher(Image, "~/debug_image", 2)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._camera_info_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._image_callback,
            qos_profile_sensor_data
        )
        output_path = self.get_parameter("output_yaml").value
        self.get_logger().info(
            f"waiting for ZED image and CameraInfo; output={output_path}"
        )

    def _camera_info_callback(self, message):
        expected = self.get_parameter("expected_optical_frame").value
        if expected and message.header.frame_id != expected:
            self.get_logger().error(
                f"CameraInfo frame is {message.header.frame_id!r}, "
                f"expected {expected!r}; ignoring it"
            )
            return
        if message.distortion_model not in ("plumb_bob", "rational_polynomial", ""):
            self.get_logger().error(
                f"unsupported distortion model: {message.distortion_model!r}"
            )
            return
        self.camera_info = message

    def _image_callback(self, message):
        info = self.camera_info
        if info is None:
            return
        expected = self.get_parameter("expected_optical_frame").value
        if expected and message.header.frame_id != expected:
            self.get_logger().error(
                f"Image frame is {message.header.frame_id!r}, expected {expected!r}"
            )
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        corners, ids, _ = self._detect(image)
        detected_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
        estimates = []
        if ids is not None:
            for marker_corners, marker_id in zip(corners, detected_ids):
                if marker_id not in self.table_from_markers:
                    continue
                estimate = estimate_marker(
                    marker_id, marker_corners, self.marker_length, camera_matrix,
                    distortion, self.table_from_markers[marker_id]
                )
                if estimate is not None:
                    estimates.append(estimate)
        result = fuse_estimates(estimates, self.layout["quality"])
        metrics = {
            "detected_marker_ids": detected_ids,
            "num_valid_markers": 0,
            "reprojection_error": None,
            "translation_spread": None,
            "rotation_spread_deg": None,
            "calibration_valid": False,
            "confidence": 0.0,
        }
        if result is not None:
            metrics.update({
                "num_valid_markers": len(result["inliers"]),
                "reprojection_error": result["reprojection_error"],
                "translation_spread": result["translation_spread"],
                "rotation_spread_deg": result["rotation_spread_deg"],
                "calibration_valid": result["calibration_valid"],
                "confidence": result["confidence"],
            })
            base_from_camera = inverse(self.table_from_base) @ result["table_from_camera"]
            metrics["markers_used"] = [item.marker_id for item in result["inliers"]]
            if result["calibration_valid"] and (
                not self.saved_once or self.get_parameter("save_continuously").value
            ):
                for item in estimates:
                    self.get_logger().info(
                        f"marker {item.marker_id}: "
                        f"reprojection={item.reprojection_error_px:.3f}px "
                        f"T_table_camera="
                        f"{np.array2string(item.table_from_camera, precision=6)}"
                    )
                self.get_logger().info(
                    "fused T_base_camera="
                    + np.array2string(base_from_camera, precision=6)
                )
                self._write_result(message, base_from_camera, metrics)
                self._save_debug_image(
                    image.copy(), ids, corners, estimates, camera_matrix, distortion
                )
                self.saved_once = True
        quality_message = String()
        quality_message.data = json.dumps(metrics, separators=(",", ":"))
        self.quality_pub.publish(quality_message)
        if self.debug_enabled:
            self._publish_debug(message, image, ids, corners, estimates, camera_matrix,
                                distortion)

    def _publish_debug(self, source, image, ids, corners, estimates, camera_matrix,
                       distortion):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for estimate in estimates:
            rotation_vector, _ = cv2.Rodrigues(estimate.camera_from_marker[:3, :3])
            cv2.drawFrameAxes(
                image, camera_matrix, distortion, rotation_vector,
                estimate.camera_from_marker[:3, 3], self.marker_length * 0.5
            )
            anchor = tuple(np.rint(estimate.corners[0]).astype(int))
            cv2.putText(
                image, f"ID {estimate.marker_id}: {estimate.reprojection_error_px:.2f}px",
                (anchor[0], max(20, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 255), 2, cv2.LINE_AA
            )
        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = source.header
        self.debug_pub.publish(output)

    def _write_result(self, message, base_from_camera, metrics):
        quaternion = Rotation.from_matrix(base_from_camera[:3, :3]).as_quat()
        stamp = message.header.stamp
        document = {
            "base_frame": self.layout["franka"]["base_frame"],
            "camera_frame": message.header.frame_id,
            "transform_convention": "T_base_camera maps camera-frame points into base",
            "translation": {
                "x": float(base_from_camera[0, 3]),
                "y": float(base_from_camera[1, 3]),
                "z": float(base_from_camera[2, 3]),
            },
            "quaternion": {
                "x": float(quaternion[0]), "y": float(quaternion[1]),
                "z": float(quaternion[2]), "w": float(quaternion[3]),
            },
            "matrix_4x4": base_from_camera.tolist(),
            "timestamp": datetime.fromtimestamp(
                stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc
            ).isoformat(),
            "markers_used": metrics["markers_used"],
            "reprojection_error": metrics["reprojection_error"],
            "translation_spread": metrics["translation_spread"],
            "rotation_spread_deg": metrics["rotation_spread_deg"],
            "confidence": metrics["confidence"],
            "calibration_valid": metrics["calibration_valid"],
            "table_to_base_is_approximate": bool(self.layout["franka"]["approximate"]),
        }
        path = os.path.abspath(os.path.expanduser(self.get_parameter("output_yaml").value))
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=directory, delete=False
        ) as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            temporary_path = stream.name
        os.replace(temporary_path, path)
        self.get_logger().info(f"saved valid camera extrinsic to {path}")

    def _save_debug_image(self, image, ids, corners, estimates, camera_matrix,
                          distortion):
        path_parameter = self.get_parameter("debug_image_output").value
        if not path_parameter:
            return
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for estimate in estimates:
            rotation_vector, _ = cv2.Rodrigues(estimate.camera_from_marker[:3, :3])
            cv2.drawFrameAxes(
                image, camera_matrix, distortion, rotation_vector,
                estimate.camera_from_marker[:3, 3], self.marker_length * 0.5
            )
            anchor = tuple(np.rint(estimate.corners[0]).astype(int))
            cv2.putText(
                image, f"ID {estimate.marker_id}: {estimate.reprojection_error_px:.2f}px",
                (anchor[0], max(20, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 255), 2, cv2.LINE_AA
            )
        path = os.path.abspath(os.path.expanduser(path_parameter))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if not cv2.imwrite(path, image):
            raise RuntimeError(f"failed to save debug image: {path}")
        self.get_logger().info(f"saved calibration debug image to {path}")


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A composed image publisher can disappear between DDS wake-up and
        # take_message() while launch is handling Ctrl-C.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
