from pathlib import Path
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from camera_extrinsic_calibration.acquisition import StableWindow, rectified_camera_model
from camera_extrinsic_calibration.calibrator import (
    DEFAULT_QUALITY, failure, finalize_window, marker_object_points,
    select_candidate, solve_joint, table_object_points,
)
from camera_extrinsic_calibration.se3 import as_transform, inverse


@pytest.fixture
def scene():
    layout = yaml.safe_load(
        (Path(__file__).parents[1] /
         'config/extrinsic_calibration.yaml').read_text())
    objects = table_object_points(0.077, {int(i): as_transform(v['T_table_marker'])
                                          for i, v in layout['markers'].items()})
    k = np.array([[528.8273315429688, 0, 630.1070556640625],
                  [0, 528.8273315429688, 356.42413330078125], [0, 0, 1.]])
    table_camera = np.eye(4)
    table_camera[:3, :3] = Rotation.from_euler('xyz', [155, 8, -5], degrees=True).as_matrix()
    table_camera[:3, 3] = [0.38, -0.2, 1.2]
    return objects, k, inverse(table_camera), layout


def project(scene, pose=None, ids=None):
    objects, k, ground_truth, _ = scene
    pose = ground_truth if pose is None else pose
    return {i: cv2.projectPoints(v, cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3],
                                 k, np.zeros(5))[0].reshape(4, 2)
            for i, v in objects.items() if ids is None or i in ids}


def solve(scene, observations):
    return solve_joint(observations, scene[0], scene[1], np.zeros(5))


@pytest.mark.parametrize('ids', [[0, 1], [1, 2], [0, 1, 2, 5]])
def test_joint_noiseless_pose_and_transform_direction(scene, ids):
    result = solve(scene, project(scene, ids=ids))
    assert result['calibration_valid'], result
    assert sorted(result['support_ids']) == sorted(ids)
    assert result['global_rms_px'] < 1e-7
    assert np.allclose(result['camera_from_table'], scene[2], atol=1e-7)
    assert np.allclose(result['table_from_camera'], inverse(scene[2]), atol=1e-7)
    base_table = inverse(as_transform(scene[3]['franka']['T_table_base']))
    base_camera = base_table @ result['table_from_camera']
    p_table = np.array([0.2, 0.3, 0, 1])
    assert np.allclose(base_camera @ scene[2] @ p_table, base_table @ p_table)


def test_rotated_upper_markers_keep_decoded_corner_order(scene):
    obj = scene[0]
    assert obj[2][0, 0] > obj[2][1, 0]
    assert obj[1][0, 0] < obj[1][1, 0]
    assert marker_object_points(.077)[0, 1] < 0
    assert solve(scene, project(scene, ids=[2, 5]))['calibration_valid']
    bad = project(scene, ids=[1, 2])
    bad[2] = np.roll(bad[2], 2, axis=0)
    assert not solve(scene, bad)['calibration_valid']


def test_single_never_valid_or_saved(scene):
    obs = project(scene, ids=[1])
    result = solve(scene, obs)
    window = StableWindow()
    for i in range(50):
        assert not window.add(1_000_000_000 + i * 100_000_000, ('model',), result, obs)
    assert window.state == 'waiting'
    assert not window.frames


def test_two_conflicting_boards_fail(scene):
    obs = project(scene, ids=[0, 1])
    obs[0] += [35, -20]
    result = solve(scene, obs)
    assert not result['calibration_valid']
    assert not result['support_ids']


@pytest.mark.parametrize('ids', [[0, 1, 2], [0, 1, 2, 5]])
def test_outlier_scored_as_whole_board(scene, ids):
    obs = project(scene, ids=ids)
    obs[0] += [35, -20]
    result = solve(scene, obs)
    assert result['calibration_valid'], result
    assert result['support_ids'] == sorted(set(ids) - {0})
    assert result['per_marker_rms_px'][0] > 1.5
    assert '0' in result['rejected_reasons']


def test_ambiguous_equal_support_not_return_order(scene):
    a = dict(camera_from_table=scene[2], support_ids=[0, 1], global_rms_px=.2)
    other = scene[2].copy()
    other[0, 3] += .03
    b = dict(camera_from_table=other, support_ids=[2, 5], global_rms_px=.25)
    for candidates in ([a, b], [b, a]):
        best, ambiguous = select_candidate(candidates, DEFAULT_QUALITY)
        assert ambiguous and best is a


def test_real_two_consensus_ambiguity(scene):
    # Two different cameras supply two boards each; neither interpretation may win by order.
    obs = project(scene, ids=[0, 1])
    moved = scene[2].copy()
    moved[0, 3] += .08
    obs.update(project(scene, moved, ids=[2, 5]))
    result = solve(scene, obs)
    assert not result['calibration_valid']
    assert result['candidate_ambiguity'] or result['reason'] == 'no_consistent_multiboard_pose'


def test_positive_depth_and_camera_above_table(scene):
    below = np.eye(4)
    below[2, 3] = 1.2  # camera below table, all corners still in front of camera
    result = solve(scene, project(scene, below))
    assert not result['calibration_valid']


def collect(scene, perturb=None, count=31):
    window = StableWindow()
    rng = np.random.default_rng(17)
    ready = False
    for i in range(count):
        obs = project(scene)
        for points in obs.values():
            points += rng.normal(0, .07, points.shape)
        result = solve(scene, obs)
        assert result['calibration_valid']
        if perturb:
            perturb(i, result)
        ready = window.add(1_000_000_000 + i * 70_000_000, ('model',), result, obs)
    return window, ready


def test_stable_sequence_final_common_pose(scene):
    window, ready = collect(scene)
    assert ready and window.state == 'finalizing'
    final = finalize_window(window.frames, scene[0], scene[1], np.zeros(5), DEFAULT_QUALITY)
    assert final['calibration_valid']
    assert len(final['per_frame_metrics']) == 31
    assert np.linalg.norm(final['table_from_camera'][:3, 3] - inverse(scene[2])[:3, 3]) < .001
    assert not np.array_equal(final['camera_from_table'],
                              window.frames[-1]['result']['camera_from_table'])
    window.mark_saved()
    assert not window.add(5_000_000_000, ('model',), failure('lost'), {})
    assert window.state == 'saved'
    window.reset('manual', rearm=True)
    assert window.state == 'waiting' and not window.frames


@pytest.mark.parametrize('kind', ['jump', 'slow_drift', 'rotation'])
def test_motion_prevents_complete_window(scene, kind):
    def perturb(i, result):
        pose = result['table_from_camera'].copy()
        if kind == 'jump':
            pose[0, 3] += .02 if i >= 15 else 0
        elif kind == 'slow_drift':
            pose[0, 3] += .0004 * i
        else:
            pose[:3, :3] = Rotation.from_euler(
                'z', i * .04, degrees=True).as_matrix() @ pose[:3, :3]
        result['table_from_camera'] = pose
    window, ready = collect(scene, perturb)
    assert not ready and window.state != 'finalizing'
    assert len(window.frames) < 30


@pytest.mark.parametrize('issue', ['lost', 'intrinsics', 'size',
                         'duplicate', 'backward', 'gap', 'zero'])
def test_bad_data_resets_window(scene, issue):
    obs = project(scene)
    result = solve(scene, obs)
    window = StableWindow()
    for i in range(10):
        assert not window.add(1_000_000_000 + i * 70_000_000, ('model', 1280), result, obs)
    stamp, signature = 1_700_000_000, ('model', 1280)
    if issue == 'lost':
        result = failure('no_detection')
    elif issue == 'intrinsics':
        signature = ('changed', 1280)
    elif issue == 'size':
        signature = ('model', 2560)
    elif issue == 'duplicate':
        stamp = 1_630_000_000
    elif issue == 'backward':
        stamp = 1_600_000_000
    elif issue == 'zero':
        stamp = 0
    else:
        stamp = 3_000_000_000
    assert not window.add(stamp, signature, result, obs)
    assert not window.frames and window.state == 'waiting'


def test_continuous_still_requires_new_full_window(scene):
    window, ready = collect(scene)
    assert ready
    window.continuous = True
    window.mark_saved()
    obs = project(scene)
    assert not window.add(3_170_000_000, ('model',), solve(scene, obs), obs)
    assert len(window.frames) == 1 and window.state == 'collecting'


def test_common_pose_rechecks_each_frame_each_board(scene):
    window, _ = collect(scene)
    window.frames[7]['observations'][0] += [10, 0]
    final = finalize_window(window.frames, scene[0], scene[1], np.zeros(5), DEFAULT_QUALITY)
    assert not final['calibration_valid']
    assert final['reason'] == 'final_frame_residual_failed'


def camera_info(scene):
    return NS(header=NS(frame_id='left'), width=1280, height=720,
              p=np.c_[scene[1], np.zeros(3)].ravel().tolist(),
              k=(scene[1] * 1.1).ravel().tolist(), d=[.1, .2, 0, 0, 0],
              r=np.eye(3).ravel().tolist(), distortion_model='plumb_bob',
              binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))


def test_rect_uses_p_zero_distortion_not_raw_k_d(scene):
    info = camera_info(scene)
    k, d, r, signature = rectified_camera_model(info, 1280, 720, 'left', 'left')
    assert np.array_equal(k, scene[1]) and np.all(d == 0)
    assert np.array_equal(r, np.eye(3))
    info.p[0] += 1
    assert rectified_camera_model(info, 1280, 720, 'left', 'left')[3] != signature


@pytest.mark.parametrize('issue', ['stitched', 'frame', 'right',
                         'binning', 'roi', 'invalid_p', 'invalid_r'])
def test_invalid_camera_model_rejected(scene, issue):
    info = camera_info(scene)
    width, frame = 1280, 'left'
    if issue == 'stitched':
        width = 2560
    elif issue == 'frame':
        frame = 'right'
    elif issue == 'right':
        info.p[3] = -63
    elif issue == 'binning':
        info.binning_x = 2
    elif issue == 'roi':
        info.roi.x_offset = 1
    elif issue == 'invalid_p':
        info.p[0] = 0
    else:
        info.r = [0.] * 9
    with pytest.raises(ValueError):
        rectified_camera_model(info, width, 720, frame, 'left')


def test_stitched_size_rejected_even_when_info_agrees(scene):
    info = camera_info(scene)
    info.width = 2560
    with pytest.raises(ValueError, match='unexpected_left_image_size'):
        rectified_camera_model(info, 2560, 720, 'left', 'left')


@pytest.mark.parametrize('period_ns,count', [(10_000_000, 40), (150_000_000, 20)])
def test_both_minimum_duration_and_count_required(scene, period_ns, count):
    obs = project(scene)
    result = solve(scene, obs)
    window = StableWindow()
    for i in range(count):
        assert not window.add(1_000_000_000 + i * period_ns, ('model',), result, obs)
    assert window.state == 'collecting'


def test_continuous_duplicate_at_window_boundary_is_rejected(scene):
    window, ready = collect(scene)
    assert ready
    window.continuous = True
    window.mark_saved()
    obs = project(scene)
    assert not window.add(3_100_000_000, ('model',), solve(scene, obs), obs)
    assert not window.frames
    assert window.reason == 'nonmonotonic_or_duplicate_timestamp'
