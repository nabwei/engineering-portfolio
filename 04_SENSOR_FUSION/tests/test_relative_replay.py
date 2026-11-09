import cv2
import numpy as np
import pytest

from run_relative_euroc import ReplayConfiguration, StereoReplay, replay, replay_frames
from test_euroc import make_sequence, write_csv
from test_flow_stereo import plane_images
from vio.eskf import State
from vio.euroc import camera_records, numeric_records, stereo_rectification
from vio.flow_stereo import FlowStereoOdometry
from vio.geometry import rotation_log
from vio.relative import RelativePoseFilter
from vio.noisy_frame import NoisyFramePoseFilter
from vio.replay_initialization import moving_start
from vio.stereo import StereoOdometry


def test_autonomous_replay_does_not_require_ground_truth_file(tmp_path):
    make_sequence(tmp_path)
    for camera_name in ['cam0', 'cam1']:
        camera = camera_records(tmp_path/camera_name)
        write_csv(tmp_path/camera_name/'data.csv', [[stamp+1_000_000_000, path.name] for stamp, path in camera.items()])
    epoch = 1_400_000_000_000_000_000
    write_csv(tmp_path/'imu0/data.csv', [[epoch+i*10_000_000, 0, 0, 0, 0, 0, 9.80665] for i in range(161)])
    (tmp_path/'state_groundtruth_estimate0/data.csv').unlink()
    report, trajectory = replay(tmp_path)
    assert report['initialization']['ground_truth_used'] is False
    assert report['provenance'] == 'UNVERIFIED_INPUT_FILES'
    assert report['input_format'] == 'EUROC_ASL'
    assert len(trajectory) == 7
    np.testing.assert_allclose(trajectory[0][1], np.eye(4), atol=1e-12)
    assert np.isfinite([row[1] for row in trajectory]).all()


@pytest.mark.parametrize("filter_type, update_indices", [(RelativePoseFilter, [1, 4, 6]),
                                                         (NoisyFramePoseFilter, [1, 2, 4, 5, 6])])
def test_real_stereo_pipeline_retains_failed_frame_and_applies_declared_pair_policy(tmp_path, filter_type, update_indices):
    cv2.setNumThreads(1)
    make_sequence(tmp_path)
    cameras = camera_records(tmp_path/'cam0'), camera_records(tmp_path/'cam1')
    times = sorted(cameras[0])
    # A stationary scene isolates loss/reacquisition from the coarse integer-
    # pixel renderer's motion error (a separate public-sequence accuracy check).
    for camera in cameras:
        stationary_image = cv2.imread(str(camera[times[0]]), cv2.IMREAD_GRAYSCALE)
        for path in camera.values():
            cv2.imwrite(str(path), stationary_image)
    cv2.imwrite(str(cameras[0][times[3]]), np.zeros((480, 640), np.uint8))
    imu = numeric_records(tmp_path/'imu0/data.csv', 7)
    rectification = stereo_rectification(tmp_path)
    estimator = filter_type(State())
    stream = StereoReplay((times, cameras), rectification, StereoOdometry(rectification.calibration))
    trajectory, diagnostics = replay_frames(estimator, imu, stream)
    assert len(trajectory) == 7
    assert trajectory[3][2] is None
    assert [row['timestamp_ns'] for row in diagnostics if 'accepted' in row] == [times[index] for index in update_indices]
    assert all(row['accepted'] for row in diagnostics if 'accepted' in row)
    np.testing.assert_allclose(trajectory[-1][1][:3, 3], [0, 0, 0], atol=1e-5)
    assert np.linalg.eigvalsh(estimator.covariance).min() > -1e-10
    repeated_stream = StereoReplay((times, cameras), rectification, StereoOdometry(rectification.calibration))
    repeated, repeated_diagnostics = replay_frames(filter_type(State()), imu, repeated_stream)
    np.testing.assert_array_equal([row[1] for row in repeated], [row[1] for row in trajectory])
    assert repeated_diagnostics == diagnostics


def test_moving_initializer_images_are_excluded_from_subsequent_filter_measurements(tmp_path):
    make_sequence(tmp_path)
    epoch = 1_400_000_000_000_000_000
    for camera_name in ["cam0", "cam1"]:
        camera = camera_records(tmp_path/camera_name)
        first_image = camera[min(camera)].name
        write_csv(tmp_path/camera_name/"data.csv", [[epoch+i*50_000_000, first_image] for i in range(105)])
    write_csv(tmp_path/"imu0/data.csv", [[epoch+i*5_000_000, 0, 0, 0, 0, 0, 9.80665] for i in range(1051)])
    (tmp_path/"state_groundtruth_estimate0/data.csv").unlink()
    report, trajectory = replay(tmp_path, settings=ReplayConfiguration(initialize=moving_start))
    assert report["initialization"]["ground_truth_used"] is False
    assert report["initialization"]["end_ns"] == epoch+5_000_000_000
    assert len(trajectory) == 4
    assert trajectory[0][0] == epoch+5_050_000_000
    np.testing.assert_allclose(trajectory[-1][1][:3, 3], 0, atol=1e-5)


@pytest.mark.parametrize("filter_type", [RelativePoseFilter, NoisyFramePoseFilter])
def test_moving_image_replay_recovers_metric_motion_from_incorrect_initial_velocity(tmp_path, filter_type):
    # The older dropout fixture is stationary. This independent moving scene
    # catches a replay that ignores visual updates: zero-rate IMU samples alone
    # cannot recover the deliberately omitted constant initial velocity.
    cv2.setNumThreads(1)
    make_sequence(tmp_path)
    (tmp_path / "state_groundtruth_estimate0/data.csv").unlink()
    cameras = camera_records(tmp_path / "cam0"), camera_records(tmp_path / "cam1")
    times = sorted(cameras[0])
    rectification = stereo_rectification(tmp_path)
    velocity = np.array([.12, .025, 0.])
    elapsed = np.array([(stamp - times[0]) * 1e-9 for stamp in times])
    expected_positions = elapsed[:, None] * velocity
    for stamp, position in zip(times, expected_positions):
        camera_pose = np.eye(4)
        camera_pose[:3, 3] = position
        for paths, frame in zip(cameras, plane_images(rectification.calibration, camera_pose)):
            assert cv2.imwrite(str(paths[stamp]), frame)
    imu = numeric_records(tmp_path / "imu0/data.csv", 7)
    # This fixture supplies known attitude and biases, but uncertain velocity.
    # A default uncertain tilt prior also explains translation via gravity and
    # cannot support a tight absolute-attitude claim over only 0.6 seconds.
    # This restricted prior was selected after two failed development fixtures;
    # it is a visual-update wiring regression, not an arbitrary-start benchmark.
    prior = np.diag(np.repeat([.01, 1., 1e-10, 1e-10, 1e-10], 3))
    estimator = filter_type(State(), initial_covariance=prior)
    stream = StereoReplay((times, cameras), rectification, FlowStereoOdometry(rectification.calibration))
    trajectory, diagnostics = replay_frames(estimator, imu, stream)

    assert [row[0] for row in trajectory] == times
    assert all(row[2] is not None for row in trajectory)
    assert all(row["accepted"] for row in diagnostics if "accepted" in row)
    visual_positions = np.array([row[2][:3, 3] for row in trajectory])
    fused_positions = np.array([row[1][:3, 3] for row in trajectory])
    np.testing.assert_allclose(visual_positions, expected_positions, atol=.01)
    np.testing.assert_allclose(fused_positions, expected_positions, atol=.015)
    assert max(np.linalg.norm(rotation_log(row[1][:3, :3])) for row in trajectory) < .003
    assert np.linalg.eigvalsh(estimator.covariance).min() > -1e-10
