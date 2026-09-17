"""ROS 2 front end for synchronized multi-board fixed-camera calibration."""

from collections import OrderedDict
from datetime import datetime, timezone
import hashlib
import json
import os
import tempfile
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .acquisition import StableWindow, rectified_camera_model
from .calibrator import failure, finalize_window, quality_config, solve_joint, table_object_points
from .se3 import as_transform, inverse


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def atomic_yaml(path, document):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                         delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


class ExtrinsicCalibratorNode(Node):
    def __init__(self):
        super().__init__("camera_extrinsic_calibrator")
        for name, default in {
            "layout_file": "", "image_topic": "/zed/zed_node/left/color/rect/image",
            "camera_info_topic": "/zed/zed_node/left/color/rect/camera_info",
            "expected_optical_frame": "zed_left_camera_frame_optical",
            "output_yaml": "camera_extrinsic.yaml", "debug_image_output": "",
            "debug": True, "save_continuously": False,
            "expected_image_width": 1280, "expected_image_height": 720,
        }.items():
            self.declare_parameter(name, default)
        self.layout_path = os.path.realpath(
            os.path.expanduser(
                self.get_parameter("layout_file").value))
        self.output_path = os.path.abspath(
            os.path.expanduser(
                self.get_parameter("output_yaml").value))
        with open(self.layout_path, encoding="utf-8") as stream:
            layout_text = stream.read()
        self.layout = yaml.safe_load(layout_text)
        self.layout_hash = hashlib.sha256(layout_text.encode()).hexdigest()
        self.quality = quality_config(self.layout.get("quality"))
        self.window = StableWindow(self.quality, self.get_parameter("save_continuously").value)
        self.objects = table_object_points(float(self.layout["aruco"]["marker_length"]), {
            int(i): as_transform(v["T_table_marker"]) for i, v in self.layout["markers"].items()})
        self.table_from_base = as_transform(self.layout["franka"]["T_table_base"])
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.layout["aruco"]["dictionary"]))
        params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "ArucoDetector")
                  else cv2.aruco.DetectorParameters_create())
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 40
        params.cornerRefinementMinAccuracy = 0.01
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(dictionary, params)
            self.detect = self.detector.detectMarkers
        else:
            self.detect = lambda image: cv2.aruco.detectMarkers(
                image, dictionary, parameters=params)
        self.bridge = CvBridge()
        self.images, self.infos = OrderedDict(), OrderedDict()
        self.last_pair_wall = time.monotonic()
        self.latest_metrics = failure("waiting_for_synchronized_data")
        self.quality_pub = self.create_publisher(String, "~/quality", 10)
        self.debug_pub = self.create_publisher(Image, "~/debug_image", 2)
        self.raw_pub = self.create_publisher(Image, "~/raw_image", 2)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._image_callback, qos_profile_sensor_data)
        self.create_service(Trigger, "~/recollect", self._recollect)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(f"layout_file={self.layout_path} (sha256={self.layout_hash})")
        self.get_logger().info(
            f"output_yaml={self.output_path}; waiting for exact-stamp left rect Image/CameraInfo")

    def _recollect(self, request, response):
        self.window.reset("manual_recollect", rearm=True)
        self.images.clear()
        self.infos.clear()
        self.last_pair_wall = time.monotonic()
        self.latest_metrics = failure("manual_recollect")
        self._publish_quality()
        response.success = True
        response.message = "Rearmed; previous YAML stays until a new stable window passes."
        return response

    def _watchdog(self):
        if time.monotonic() - self.last_pair_wall > self.quality["max_data_gap_sec"]:
            self.window.reset("data_gap_or_missing_synchronized_camera_info")
            self.images.clear()
            self.infos.clear()
            self.latest_metrics = failure("data_gap_or_missing_synchronized_camera_info")
            self._publish_quality()

    def _camera_info_callback(self, message):
        self._receive(message, self.infos, self.images, False)

    def _image_callback(self, message):
        self._receive(message, self.images, self.infos, True)

    def _receive(self, message, own, other, is_image):
        timestamp = stamp_ns(message)
        own[timestamp] = message
        if timestamp in other:
            paired = other.pop(timestamp)
            own.pop(timestamp)
            now = time.monotonic()
            if now - self.last_pair_wall > self.quality["max_data_gap_sec"]:
                self.window.reset("wall_clock_data_gap")
            self.last_pair_wall = now
            image, info = (message, paired) if is_image else (paired, message)
            try:
                self._process(image, info)
            except (ValueError, cv2.error, CvBridgeError, RuntimeError,
                    OSError, yaml.YAMLError) as exc:
                self.window.reset(f"processing_failed: {exc}")
                self.latest_metrics = failure(str(exc))
                self.get_logger().error(str(exc))
                self._publish_quality()
        if len(own) > 10:
            own.popitem(last=False)
            self.window.reset("synchronization_queue_overflow")
            self.latest_metrics = failure("synchronization_queue_overflow")
            self._publish_quality()

    def _process(self, message, info):
        matrix, distortion, rectification, signature = rectified_camera_model(
            info, message.width, message.height, message.header.frame_id,
            self.get_parameter("expected_optical_frame").value,
            (self.get_parameter("expected_image_width").value,
             self.get_parameter("expected_image_height").value))
        self.window.continuous = bool(self.get_parameter("save_continuously").value)
        raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if raw.shape[:2] != (message.height, message.width):
            raise ValueError("decoded_image_size_mismatch")
        corners, ids, rejected = self.detect(cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY))
        detected = [] if ids is None else [int(i) for i in ids.reshape(-1)]
        observations = {i: np.asarray(c, dtype=float).reshape(4, 2)
                        for i, c in zip(detected, corners)}
        result = (failure("duplicate_marker_ids", detected) if len(set(detected)) != len(detected)
                  else solve_joint(observations, self.objects, matrix, distortion, self.quality))
        self.latest_metrics = {k: v for k, v in result.items() if not isinstance(v, np.ndarray)}
        configured_obs = {i: v for i, v in observations.items() if i in self.objects}
        ready = self.window.add(stamp_ns(message), signature, result, configured_obs)
        if ready:
            self._publish_quality()  # expose finalizing before optimization / I/O
            final = finalize_window(
                self.window.frames,
                self.objects,
                matrix,
                distortion,
                self.quality)
            if not final["calibration_valid"]:
                self.window.reset(final["reason"])
                self.latest_metrics.update(calibration_valid=False, reason=final["reason"],
                                           candidate_ambiguity=final["candidate_ambiguity"]
                                           if "candidate_ambiguity" in final else False)
            else:
                self._write_result(message, info, final, matrix, distortion, rectification)
                self.window.mark_saved()
                self.latest_metrics["saved_global_rms_px"] = final["global_rms_px"]
                self._save_images(raw, self._draw_debug(raw, corners, detected, rejected, result))
        self._publish_quality()
        if self.get_parameter("debug").value:
            self.raw_pub.publish(message)  # original unmodified image
            debug = self.bridge.cv2_to_imgmsg(
                self._draw_debug(raw, corners, detected, rejected, result), encoding="bgr8")
            debug.header = message.header
            self.debug_pub.publish(debug)

    def _publish_quality(self):
        metrics = dict(self.latest_metrics, **self.window.metrics())
        metrics["saved_once"] = self.window.state == "saved"
        metrics["output_yaml"] = self.output_path
        msg = String()
        msg.data = json.dumps(metrics, separators=(",", ":"), allow_nan=False)
        self.quality_pub.publish(msg)

    def _draw_debug(self, raw, corners, detected, rejected, result):
        debug = raw.copy()
        for quad in rejected:
            cv2.polylines(debug, [np.rint(quad).astype(np.int32).reshape(-1, 2)],
                          True, (255, 0, 255), 1)
        for i, quad in zip(detected, corners):
            support = i in result["support_ids"] and result["calibration_valid"]
            color = (0, 255, 0) if support else (0, 0, 255)
            if i not in self.objects:
                color = (0, 255, 255)
            points = np.rint(quad).astype(np.int32).reshape(4, 2)
            cv2.polylines(debug, [points], True, color, 2)
            error = result["per_marker_rms_px"].get(i)
            label = f"ID {i} {'support' if support else 'rejected/detected'}"
            if error is not None:
                label += f" {error:.2f}px"
            cv2.putText(debug, label, (int(points[0, 0]), max(20, int(points[0, 1]) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        label = f"{self.window.state}: {self.window.reason}; rejected quads={len(rejected)}"
        cv2.putText(debug, label,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return debug

    def _write_result(self, message, info, final, matrix, distortion, rectification):
        # PnP camera is the rectified optical coordinate system. R maps physical
        # optical -> rectified optical; account for a nonidentity CameraInfo.R.
        rectified_from_optical = np.eye(4)
        rectified_from_optical[:3, :3] = rectification
        table_from_camera = final["table_from_camera"] @ rectified_from_optical
        camera_from_table = inverse(table_from_camera)
        base_from_camera = inverse(self.table_from_base) @ table_from_camera
        quaternion = Rotation.from_matrix(base_from_camera[:3, :3]).as_quat()
        document = {
            "base_frame": self.layout["franka"]["base_frame"],
            "table_frame": self.layout["table"]["frame_id"],
            "camera_frame": message.header.frame_id,
            "transform_convention": (
                "T_destination_source maps source points into destination; "
                "camera is left optical"),
            "marker_corner_convention": (
                "decoded TL,TR,BR,BL; marker +x right, +y down; table +z up"),
            "T_camera_table": camera_from_table.tolist(),
            "T_table_camera": table_from_camera.tolist(),
            "T_base_camera": base_from_camera.tolist(),
            "T_rectified_camera_table": final["camera_from_table"].tolist(),
            "matrix_4x4": base_from_camera.tolist(),
            "translation": dict(zip("xyz", map(float, base_from_camera[:3, 3]))),
            "quaternion": dict(zip("xyzw", map(float, quaternion))),
            "timestamp_ns": stamp_ns(message),
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "markers_used": final["support_ids"],
            "calibration_valid": True,
            "candidate_ambiguity": final["candidate_ambiguity"],
            "global_rms_px": final["global_rms_px"],
            "per_marker_rms_px": final["per_marker_rms_px"],
            "per_frame_metrics": final["per_frame_metrics"],
            "stability": dict(self.window.metrics(), state="saved", window_reason="final_accepted",
                              final_translation_spread_m=final["final_translation_spread_m"],
                              final_rotation_spread_deg=final["final_rotation_spread_deg"]),
            "quality_thresholds": self.quality,
            "image_size": [message.width, message.height],
            "camera_model": {"pnp_matrix": matrix.tolist(), "pnp_distortion": distortion.tolist(),
                             "CameraInfo_K": list(map(float, info.k)),
                             "CameraInfo_D": list(map(float, info.d)),
                             "CameraInfo_R": list(map(float, info.r)),
                             "CameraInfo_P": list(map(float, info.p)),
                             "distortion_model": info.distortion_model},
            "layout_file": self.layout_path, "layout_sha256": self.layout_hash,
            "layout_snapshot": self.layout,
            "table_to_base_is_approximate": bool(self.layout["franka"]["approximate"]),
        }
        atomic_yaml(self.output_path, document)
        self.get_logger().info(f"saved stable camera extrinsic to {self.output_path}")

    def _save_images(self, raw, debug):
        value = self.get_parameter("debug_image_output").value
        if not value:
            return
        path = os.path.abspath(os.path.expanduser(value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        raw_path = os.path.splitext(path)[0] + "_raw.png"
        for target, image in ((path, debug), (raw_path, raw)):
            if not cv2.imwrite(target, image):
                self.get_logger().error(f"failed to save image: {target}")


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
