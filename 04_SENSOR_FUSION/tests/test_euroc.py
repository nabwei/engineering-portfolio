import csv

import cv2
import numpy as np
import pytest
import yaml

from run_euroc import replay
from vio.eskf import ErrorStateFilter, State
from vio.euroc import camera_records, ground_truth_state, integrate_interval, numeric_records, stereo_rectification
from vio.geometry import rotation_exp
from vio.synthetic import Scene


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.writer(stream).writerows(rows)


def make_sequence(root):
    scene = Scene()
    epoch = 1_400_000_000_000_000_000
    velocity = np.array([.12, .025, 0.])
    for camera, baseline in [("cam0", 0.), ("cam1", .25)]:
        transform = np.eye(4)
        transform[0, 3] = baseline
        configuration = {"T_BS": {"data": transform.ravel().tolist()}, "camera_model": "pinhole",
                         "distortion_model": "radial-tangential", "distortion_coefficients": [0., 0., 0., 0.],
                         "intrinsics": [360., 360., 320., 240.], "resolution": [640, 480]}
        (root / camera / "data").mkdir(parents=True)
        (root / camera / "sensor.yaml").write_text(yaml.safe_dump(configuration), encoding="utf-8")
        rows = []
        for step in range(7):
            timestamp = epoch+step*100000000
            image = scene.image(velocity*step*.1 + [baseline, 0, 0])
            assert cv2.imwrite(str(root / camera / "data" / f"{timestamp}.png"), image)
            rows.append([timestamp, f"{timestamp}.png"])
        write_csv(root / camera / "data.csv", rows)
    imu_config = {"T_BS": {"data": np.eye(4).ravel().tolist()}, "accelerometer_noise_density": .03,
                  "gyroscope_noise_density": .002, "accelerometer_random_walk": .0003, "gyroscope_random_walk": .00002}
    write_csv(root / "imu0/data.csv", [[epoch+i*10000000, 0, 0, 0, 0, 0, 9.80665] for i in range(61)])
    (root / "imu0/sensor.yaml").write_text(yaml.safe_dump(imu_config), encoding="utf-8")
    write_csv(root / "state_groundtruth_estimate0/data.csv", [[epoch+i*10000000, *(velocity*i*.01), 1, 0, 0, 0, *velocity, 0, 0, 0, 0, 0, 0] for i in range(61)])


def test_euroc_format_image_imu_replay_is_deterministic_and_metric(tmp_path):
    # Explicitly synthetic fixture in EuRoC format, never a public benchmark.
    make_sequence(tmp_path)
    times, trajectory, report = replay(tmp_path, 7, "SYNTHETIC_EUROC_FORMAT")
    times_again, trajectory_again, report_again = replay(tmp_path, 7, "SYNTHETIC_EUROC_FORMAT")
    assert times == times_again and report == report_again
    np.testing.assert_array_equal(trajectory, trajectory_again)
    assert report["frames"] == 7 and report["pose_updates_accepted"] >= 5
    assert report["metrics"]["raw_rmse_m"] < .04
    assert report["covariance_min_eigenvalue"] > 0
    _, bounded_trajectory, bounded_report = replay(tmp_path, 7, "SYNTHETIC_EUROC_FORMAT", "bounded")
    assert bounded_report["recovery_mode"] == "bounded" and bounded_report["recovery_events"] == []
    np.testing.assert_array_equal(trajectory, bounded_trajectory)


def test_rectification_preserves_metric_baseline_and_body_frame(tmp_path):
    make_sequence(tmp_path)
    rectification = stereo_rectification(tmp_path)
    assert rectification.calibration.baseline_m == pytest.approx(.25)
    np.testing.assert_allclose(rectification.calibration.body_from_camera, np.eye(4), atol=1e-12)


def test_nonidentity_camera_extrinsic_is_not_inverted_during_rectification(tmp_path):
    make_sequence(tmp_path)
    body_left = np.eye(4)
    body_left[:3, :3] = rotation_exp([.2, -.1, .3])
    body_left[:3, 3] = [.1, -.2, .05]
    for camera, baseline in [("cam0", 0.), ("cam1", .25)]:
        path = tmp_path / camera / "sensor.yaml"
        configuration = yaml.safe_load(path.read_text())
        transform = body_left.copy()
        transform[:3, 3] += body_left[:3, :3] @ [baseline, 0, 0]
        configuration["T_BS"]["data"] = transform.ravel().tolist()
        path.write_text(yaml.safe_dump(configuration))
    calibration = stereo_rectification(tmp_path).calibration
    np.testing.assert_allclose(calibration.body_from_camera, body_left, atol=1e-12)
    assert calibration.baseline_m == pytest.approx(.25)


def test_failed_initial_image_cannot_silently_shift_visual_world_frame(tmp_path):
    make_sequence(tmp_path)
    first = min((tmp_path / "cam0/data").glob("*.png"))
    cv2.imwrite(str(first), np.zeros((480, 640), np.uint8))
    with pytest.raises(ValueError, match="first stereo frame"):
        replay(tmp_path, 7, "SYNTHETIC_EUROC_FORMAT")


def test_ground_truth_quaternion_uses_wxyz_and_short_arc():
    epoch = 1400000000000000000
    samples = np.zeros((2, 16))
    samples[:, 3:7] = [[1, 0, 0, 0], [np.sqrt(.5), 0, 0, np.sqrt(.5)]]
    samples[1, 0] = 2
    state = ground_truth_state(epoch+5, (np.array([epoch, epoch+10]), samples))
    np.testing.assert_allclose(state.rotation, rotation_exp([0, 0, np.pi/4]), atol=1e-12)
    np.testing.assert_allclose(state.position, [1, 0, 0])


def test_replay_integrates_partial_imu_intervals_without_epoch_rounding():
    epoch = 1400000000000000000
    times = np.array([epoch+i*10000000 for i in range(12)], dtype=np.int64)
    samples = np.tile([0., 0., 0., 2., 0., 9.80665], (len(times), 1))
    estimator = ErrorStateFilter()
    integrate_interval(estimator, (epoch+5000000, epoch+105000000), (times, samples))
    np.testing.assert_allclose(estimator.state.position, [.01, 0, 0], atol=1e-12)
    np.testing.assert_allclose(estimator.state.velocity, [.2, 0, 0], atol=1e-12)


def test_duplicate_imu_timestamps_are_rejected(tmp_path):
    path = tmp_path / "imu.csv"
    write_csv(path, [[123, 0, 0, 0, 0, 0, 9.8], [123, 0, 0, 0, 0, 0, 9.8]])
    with pytest.raises(ValueError, match="increasing"):
        numeric_records(path, 7)


@pytest.mark.parametrize("rows, message", [
    ([[123, "a.png"], [123, "b.png"]], "duplicate camera timestamp"),
    ([[124, "a.png"], [123, "b.png"]], "timestamps must increase"),
    ([[123, "../outside.png"]], "invalid camera CSV row or filename"),
    ([[123, "a.png", "extra"]], "invalid camera CSV row or filename"),
])
def test_invalid_camera_records_cannot_silently_change_frame_associations(tmp_path, rows, message):
    write_csv(tmp_path / "data.csv", rows)
    with pytest.raises(ValueError, match=message):
        camera_records(tmp_path)


@pytest.mark.parametrize("acceleration,gyro", [([1], [0, 0, 0]), ([0, 0, 0], [1]), ([[0, 0, 0]], [0, 0, 0])])
def test_wrong_shape_imu_does_not_broadcast_or_mutate_covariance(acceleration, gyro):
    estimator = ErrorStateFilter()
    before = estimator.covariance.copy()
    with pytest.raises(ValueError, match="3-vectors"):
        estimator.predict(acceleration, gyro, .01)
    np.testing.assert_array_equal(estimator.covariance, before)


@pytest.mark.parametrize("rotation", [np.diag([1., 1., -1.]), np.eye(3)*2, np.full((3, 3), np.nan)])
def test_invalid_initial_attitude_is_rejected(rotation):
    with pytest.raises(ValueError, match="rotation"):
        State(rotation=rotation)
