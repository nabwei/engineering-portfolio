import numpy as np

from run_relative_euroc import pose_metrics, score
from vio.geometry import rotation_exp


def test_sparse_timestamp_segments_report_missing_rpe_without_discarding_ate():
    stamps = np.r_[np.arange(11)*1_000_000, 10_000_000_000+np.arange(11)*1_000_000].astype(np.int64)
    poses = np.tile(np.eye(4), (22, 1, 1))
    metrics = pose_metrics(stamps, poses, poses)
    assert metrics['rpe_pairs'] == 0
    assert metrics['rpe_1s_translation_rmse_m'] is None
    assert metrics['ate_se3_rmse_m'] == 0


def test_one_second_pose_error_detects_orientation_error_without_scale_alignment():
    stamps = np.arange(61, dtype=np.int64)*50_000_000 + 10**18
    truth = np.tile(np.eye(4), (61, 1, 1))
    estimate = truth.copy()
    for index in range(61):
        truth[index, 0, 3] = index*.05
        estimate[index, 0, 3] = index*.10
        estimate[index, :3, :3] = rotation_exp([0, 0, index*.05*.1])
    metrics = pose_metrics(stamps, estimate, truth)
    assert metrics['ate_se3_rmse_m'] > .8
    np.testing.assert_allclose(metrics['rpe_1s_rotation_rmse_deg'], .1*180/np.pi, atol=1e-10)
    assert metrics['rpe_1s_translation_rmse_m'] >= 1


def test_scoring_keeps_inertial_only_frames_during_visual_dropout(tmp_path):
    stamps = np.arange(61, dtype=np.int64)*50_000_000 + 10**18
    rows, trajectory = [], []
    for index, stamp in enumerate(stamps):
        truth = np.eye(4)
        truth[0, 3] = index*.05
        estimate = truth.copy()
        missing = index == 30
        if missing:
            estimate[1, 3] += 20
        trajectory.append((int(stamp), estimate, None if missing else truth.copy()))
        rows.append(','.join(map(str, [stamp, index*.05, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])))
    truth_path = tmp_path/'truth.csv'
    truth_path.write_text('\n'.join(rows))
    report = score(trajectory, truth_path, int(stamps[-1]))['all']
    assert report['frames'] == 61
    assert report['visual_frames'] == 60
    assert report['inertial_only_frames'] == 1
    assert report['status'] == 'FAIL'
    assert report['vio']['ate_se3_rmse_m'] > 2
