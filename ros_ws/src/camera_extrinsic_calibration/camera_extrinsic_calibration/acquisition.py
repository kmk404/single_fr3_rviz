"""ROS-independent camera-model validation and fixed acquisition window."""

import numpy as np

from .calibrator import quality_config
from .se3 import fuse_transforms, pose_spread


def rectified_camera_model(info, width, height, frame_id, expected_frame,
                           expected_size=(1280, 720)):
    """Use rectified left P, zero distortion, and retain R for optical conversion.

    Nontrivial ROI/binning is rejected rather than silently applying ambiguous
    driver-specific size scaling. P's fourth column must identify the left eye.
    """
    if frame_id != info.header.frame_id or (expected_frame and frame_id != expected_frame):
        raise ValueError("image_camera_info_frame_mismatch")
    if (width, height) != (info.width, info.height):
        raise ValueError("image_camera_info_size_mismatch")
    if (width, height) != tuple(expected_size):
        raise ValueError("unexpected_left_image_size")
    if width <= 0 or height <= 0:
        raise ValueError("invalid_image_size")
    roi = info.roi
    if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or roi.x_offset or roi.y_offset
            or roi.width not in (0, width) or roi.height not in (0, height)):
        raise ValueError("unsupported_roi_or_binning")
    p = np.asarray(info.p, dtype=float).reshape(3, 4)
    k = p[:, :3].copy()
    r = np.asarray(info.r, dtype=float).reshape(3, 3)
    if (not np.isfinite(p).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or not np.allclose([k[0, 1], k[1, 0]], 0)
            or not (0 <= k[0, 2] < width and 0 <= k[1, 2] < height)):
        raise ValueError("invalid_rectified_projection")
    if not np.allclose(p[:, 3], 0, atol=1e-9):
        raise ValueError("not_left_camera_projection")
    if (not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6)):
        raise ValueError("invalid_rectification_rotation")
    # Fingerprint all calibration inputs, including raw K/D, for reset detection.
    signature = (width, height, frame_id, info.distortion_model,
                 tuple(info.k), tuple(info.d), tuple(info.r), tuple(info.p),
                 info.binning_x, info.binning_y, roi.x_offset, roi.y_offset,
                 roi.width, roi.height, roi.do_rectify)
    return k, np.zeros(5), r, signature


class StableWindow:
    """A growing, non-sliding window. Caller marks saved only after atomic I/O."""

    def __init__(self, quality=None, continuous=False):
        self.quality = quality_config(quality)
        self.continuous = continuous
        self.state = "waiting"
        self.frames = []
        self.last_timestamp_ns = None
        self.signature = None
        self.reason = "waiting_for_synchronized_data"
        self.translation_spread = None
        self.rotation_spread_deg = None

    def reset(self, reason, rearm=False):
        self.frames = []
        self.reason = reason
        self.translation_spread = None
        self.rotation_spread_deg = None
        if rearm:
            self.last_timestamp_ns = None
            self.signature = None
        if self.state != "saved" or rearm:
            self.state = "waiting"

    def metrics(self):
        duration = ((self.frames[-1]["timestamp_ns"] - self.frames[0]["timestamp_ns"]) * 1e-9
                    if self.frames else 0.0)
        return dict(state=self.state, window_frames=len(self.frames), window_duration_sec=duration,
                    translation_spread_m=self.translation_spread,
                    rotation_spread_deg=self.rotation_spread_deg, window_reason=self.reason)

    def add(self, timestamp_ns, signature, result, observations):
        if self.state == "saved":
            if not self.continuous:
                return False
            # Keep timestamp/model history across automatic windows so a replay,
            # duplicate, or model change cannot masquerade as fresh acquisition.
            self.state = "waiting"
            self.reset("continuous_new_window")
        previous = self.last_timestamp_ns
        self.last_timestamp_ns = timestamp_ns
        if timestamp_ns <= 0 or (previous is not None and timestamp_ns <= previous):
            self.reset("nonmonotonic_or_duplicate_timestamp")
            return False
        if (previous is not None
                and (timestamp_ns - previous) * 1e-9 > self.quality["max_data_gap_sec"]):
            self.reset("data_gap")
            return False
        if self.signature is not None and self.signature != signature:
            self.signature = signature
            self.reset("camera_model_changed")
            return False
        self.signature = signature
        if not result["calibration_valid"] or len(
                result["support_ids"]) < self.quality["min_support_markers"]:
            self.reset(result["reason"])
            return False
        self.frames.append(dict(timestamp_ns=timestamp_ns, result=result,
                                observations={i: v.copy() for i, v in observations.items()}))
        poses = [f["result"]["table_from_camera"] for f in self.frames]
        center = fuse_transforms(poses, np.ones(len(poses)))
        self.translation_spread, self.rotation_spread_deg = pose_spread(poses, center)
        # Also anchor to the first frame: no gradual recentering of this window.
        anchor_translation, anchor_rotation = pose_spread(poses, poses[0])
        q = self.quality
        if (max(self.translation_spread, anchor_translation) > q["max_translation_spread_m"]
                or max(self.rotation_spread_deg, anchor_rotation) > q["max_rotation_spread_deg"]):
            self.reset("camera_motion_or_drift")
            return False
        self.state = "collecting"
        self.reason = "collecting_stable_observations"
        if (len(self.frames) >= q["window_min_frames"]
                and self.metrics()["window_duration_sec"] >= q["window_duration_sec"]):
            self.state = "finalizing"
            self.reason = "checking_common_pose"
            return True
        if len(self.frames) >= q["max_window_frames"]:
            self.reset("window_capacity_before_minimum_duration")
        return False

    def mark_saved(self):
        if self.state != "finalizing":
            raise RuntimeError("cannot save without a complete stable window")
        self.state = "saved"
        self.reason = "saved_and_locked" if not self.continuous else "saved_next_window_required"
