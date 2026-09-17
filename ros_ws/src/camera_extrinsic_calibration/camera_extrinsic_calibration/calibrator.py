"""Joint planar corner estimation. Every transform is destination_from_source."""

from itertools import combinations

import cv2
import numpy as np

from .se3 import inverse, pose_spread, rotation_distance_deg


DEFAULT_QUALITY = {
    "min_support_markers": 2,
    "max_global_rms_px": 1.0,
    "max_marker_rms_px": 1.5,
    "ambiguity_rms_gap_px": 0.15,
    "ambiguity_translation_m": 0.005,
    "ambiguity_rotation_deg": 0.5,
    "window_duration_sec": 2.0,
    "window_min_frames": 30,
    "max_translation_spread_m": 0.005,
    "max_rotation_spread_deg": 0.5,
    "max_data_gap_sec": 0.5,
    "max_window_frames": 1000,
}


def quality_config(values=None):
    result = dict(DEFAULT_QUALITY)
    result.update(values or {})
    for name in DEFAULT_QUALITY:
        if not np.isfinite(result[name]) or result[name] <= 0:
            raise ValueError(f"quality.{name} must be finite and positive")
    for name in ("min_support_markers", "window_min_frames", "max_window_frames"):
        if int(result[name]) != result[name]:
            raise ValueError(f"quality.{name} must be an integer")
    if result["min_support_markers"] < 2 or result["window_min_frames"] < 2:
        raise ValueError("formal calibration requires multiple markers and frames")
    if result["max_window_frames"] < result["window_min_frames"]:
        raise ValueError("max_window_frames is smaller than window_min_frames")
    return result


def marker_object_points(length):
    half = length / 2.0
    # ArUco decoded order: printed TL, TR, BR, BL. +y is printed down.
    # This is NOT IPPE_SQUARE's required object-coordinate ordering.
    return np.array([[-half, -half, 0], [half, -half, 0],
                     [half, half, 0], [-half, half, 0]], dtype=np.float64)


def table_object_points(length, table_from_markers):
    if not np.isfinite(length) or length <= 0 or len(table_from_markers) < 2:
        raise ValueError("layout requires a positive marker length and at least two markers")
    local = marker_object_points(length)
    points = {i: local @ t[:3, :3].T + t[:3, 3]
              for i, t in table_from_markers.items()}
    all_points = np.concatenate(list(points.values()))
    if not np.isfinite(all_points).all() or not np.allclose(all_points[:, 2], 0, atol=1e-8):
        raise ValueError("IPPE layout must lie on table z=0")
    return points


def stack_points(observations, objects, ids):
    return (np.ascontiguousarray(np.concatenate([objects[i] for i in ids]), dtype=float),
            np.ascontiguousarray(np.concatenate([observations[i] for i in ids]), dtype=float))


def pose_from_vectors(rvec, tvec):
    pose = np.eye(4)
    pose[:3, :3] = cv2.Rodrigues(rvec)[0]
    pose[:3, 3] = np.asarray(tvec).reshape(3)
    return pose


def refine(pose, obj, img, matrix, distortion):
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    rvec, tvec = cv2.solvePnPRefineLM(
        obj, img, matrix, distortion, rvec, pose[:3, 3].copy().reshape(3, 1))
    return pose_from_vectors(rvec, tvec)


def valid_pose(pose, obj):
    if not np.isfinite(pose).all():
        return False
    depths = (obj @ pose[:3, :3].T + pose[:3, 3])[:, 2]
    return bool(np.all(depths > 1e-6) and inverse(pose)[2, 3] > 0)


def residuals(pose, observations, objects, matrix, distortion):
    errors = {}
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    for marker_id, pixels in observations.items():
        projected = cv2.projectPoints(objects[marker_id], rvec, pose[:3, 3],
                                      matrix, distortion)[0].reshape(4, 2)
        errors[marker_id] = float(np.sqrt(np.mean(np.sum((projected - pixels)**2, axis=1))))
    return errors


def global_rms(errors, ids):
    return float(np.sqrt(np.mean([errors[i]**2 for i in ids])))


def distinct_pose(first, second, quality):
    a, b = inverse(first), inverse(second)
    return (np.linalg.norm(a[:3, 3] - b[:3, 3]) > quality["ambiguity_translation_m"]
            or rotation_distance_deg(a[:3, :3], b[:3, :3]) > quality["ambiguity_rotation_deg"])


def failure(reason, detected=(), **details):
    return dict(calibration_valid=False, reason=reason, detected_marker_ids=list(detected),
                support_ids=[], rejected_reasons={str(i): reason for i in detected},
                global_rms_px=None, per_marker_rms_px={}, candidate_ambiguity=False,
                **details)


def select_candidate(candidates, quality):
    """Rank by whole-board support, then residual; never by IPPE return order."""
    candidates.sort(key=lambda c: (-len(c["support_ids"]), c["global_rms_px"]))
    best = candidates[0]
    ambiguous = any(
        len(c["support_ids"]) == len(best["support_ids"])
        and c["global_rms_px"] - best["global_rms_px"] <= quality["ambiguity_rms_gap_px"]
        and distinct_pose(c["camera_from_table"], best["camera_from_table"], quality)
        for c in candidates[1:])
    return best, ambiguous


def solve_joint(observations, objects, matrix, distortion, quality=None):
    q = quality_config(quality)
    detected = sorted(observations)
    obs = {i: np.asarray(v, dtype=float).reshape(4, 2)
           for i, v in observations.items() if i in objects}
    if any(not np.isfinite(v).all() for v in obs.values()):
        return failure("nonfinite_corners", detected)
    if len(obs) < q["min_support_markers"]:
        return failure("insufficient_configured_markers", detected)
    ids = sorted(obs)
    candidates = []
    diagnostic = None
    for count in range(int(q["min_support_markers"]), len(ids) + 1):
        for subset in combinations(ids, count):
            obj, img = stack_points(obs, objects, subset)
            try:
                success, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                    obj, img, matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
                if not success:
                    continue
                for rvec, tvec in zip(rvecs, tvecs):
                    pose = pose_from_vectors(rvec, tvec)
                    if not valid_pose(pose, obj):
                        continue
                    pose = refine(pose, obj, img, matrix, distortion)
                    # Refit the entire support, then require that support is a fixed point.
                    previous = None
                    for _ in range(len(ids) + 2):
                        errors = residuals(pose, obs, objects, matrix, distortion)
                        all_rms = global_rms(errors, ids)
                        if (np.isfinite(all_rms)
                                and (diagnostic is None or all_rms < diagnostic[0])):
                            diagnostic = (all_rms, errors)
                        support = [i for i in ids if errors[i] <= q["max_marker_rms_px"]
                                   and valid_pose(pose, objects[i])]
                        if len(support) < q["min_support_markers"]:
                            break
                        if support == previous:
                            rms = global_rms(errors, support)
                            if rms <= q["max_global_rms_px"]:
                                candidates.append(dict(camera_from_table=pose,
                                                       support_ids=support,
                                                       global_rms_px=rms,
                                                       per_marker_rms_px=errors))
                            break
                        previous = support
                        support_obj, support_img = stack_points(obs, objects, support)
                        pose = refine(pose, support_obj, support_img, matrix, distortion)
            except cv2.error:
                continue
    if not candidates:
        rejected = failure("no_consistent_multiboard_pose", detected, candidate_count=0)
        if diagnostic is not None:
            rejected.update(global_rms_px=diagnostic[0], per_marker_rms_px=diagnostic[1],
                            residual_scope="best_rejected_joint_pose_all_configured_detections")
        return rejected
    best, ambiguous = select_candidate(candidates, q)
    support = best["support_ids"]
    reasons = {str(i): ("not_configured" if i not in objects else
                        "whole_board_residual_or_depth") for i in detected if i not in support}
    return dict(best, table_from_camera=inverse(best["camera_from_table"]),
                detected_marker_ids=detected, rejected_reasons=reasons,
                candidate_ambiguity=ambiguous, candidate_count=len(candidates),
                calibration_valid=not ambiguous,
                reason="ambiguous_candidates" if ambiguous else "accepted")


def finalize_window(frames, objects, matrix, distortion, quality):
    """One LM fit to all accepted corner observations, checked frame by frame."""
    q = quality_config(quality)
    obj = np.concatenate([stack_points(f["observations"], objects,
                                       f["result"]["support_ids"])[0] for f in frames])
    img = np.concatenate([stack_points(f["observations"], objects,
                                       f["result"]["support_ids"])[1] for f in frames])
    candidates = []
    # Preserve alternate basins from every accepted frame during the final fit.
    for frame in frames:
        initial = frame["result"]["camera_from_table"]
        if any(not distinct_pose(initial, c["initial"], q) for c in candidates):
            continue
        pose = refine(initial, obj, img, matrix, distortion)
        if not valid_pose(pose, obj):
            return failure("final_invalid_pose")
        frame_metrics = []
        all_errors = []
        board_errors = {}
        for f in frames:
            errors = residuals(pose, f["observations"], objects, matrix, distortion)
            ids = f["result"]["support_ids"]
            rms = global_rms(errors, ids)
            if (rms > q["max_global_rms_px"]
                    or any(errors[i] > q["max_marker_rms_px"] for i in ids)):
                return failure("final_frame_residual_failed")
            # A previously rejected board must not become an unexamined new supporter.
            support = [i for i in sorted(errors) if errors[i] <= q["max_marker_rms_px"]
                       and valid_pose(pose, objects[i])]
            if support != ids:
                return failure("final_support_changed")
            frame_metrics.append(dict(timestamp_ns=f["timestamp_ns"], support_ids=ids,
                                      global_rms_px=rms, per_marker_rms_px=errors))
            all_errors.extend(errors[i] for i in ids)
            for i in ids:
                board_errors.setdefault(i, []).append(errors[i])
        candidates.append(dict(initial=initial, camera_from_table=pose,
                               support_ids=sorted(board_errors),
                               global_rms_px=float(np.sqrt(np.mean(np.square(all_errors)))),
                               per_marker_rms_px={i: float(np.sqrt(np.mean(np.square(v))))
                                                  for i, v in board_errors.items()},
                               per_frame_metrics=frame_metrics))
    best, ambiguous = select_candidate(candidates, q)
    best.pop("initial")
    translation, rotation = pose_spread(
        [f["result"]["table_from_camera"] for f in frames], inverse(best["camera_from_table"]))
    if (translation > q["max_translation_spread_m"]
            or rotation > q["max_rotation_spread_deg"]):
        return failure("final_pose_outside_stable_window")
    best["final_translation_spread_m"] = translation
    best["final_rotation_spread_deg"] = rotation
    return dict(best, calibration_valid=not ambiguous, candidate_ambiguity=ambiguous,
                table_from_camera=inverse(best["camera_from_table"]),
                reason="final_ambiguous" if ambiguous else "final_accepted")
