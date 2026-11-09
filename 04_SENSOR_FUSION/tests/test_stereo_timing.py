import numpy as np
import pytest

from vio.stereo import StereoOdometry, triangulate_rectified
from vio.synthetic import Scene, calibration
from vio.timing import aligned_camera_times, sample_interpolated_imu


def test_rectified_stereo_recovers_metric_depth_and_rejects_bad_epipolar_match():
    left = np.array([[320., 240.], [356., 240.], [320., 240.], [320., 240.]])
    right = np.array([[302., 240.], [338., 240.], [322., 240.], [302., 245.]])
    points, mask = triangulate_rectified(left, right, calibration())
    np.testing.assert_allclose(points, [[0, 0, 5], [0.5, 0, 5]])
    np.testing.assert_array_equal(mask, [True, True, False, False])


def test_image_driven_stereo_recovers_known_camera_translation():
    scene = Scene()
    tracker = StereoOdometry(scene.calibration)
    initial = tracker.track(*scene.stereo(np.zeros(3)))
    pose = tracker.track(*scene.stereo(np.array([0.08, 0., 0.])))
    np.testing.assert_allclose(initial, np.eye(4))
    assert tracker.last_inliers >= 15
    assert np.linalg.norm(pose[:3, 3] - [0.08, 0., 0.]) < 0.035


def test_blank_camera_fails_explicitly():
    frame = np.zeros((480, 640), np.uint8)
    with pytest.raises(ValueError, match="insufficient"):
        StereoOdometry(calibration()).track(frame, frame)


def test_nanosecond_alignment_preserves_large_epoch_precision_and_endpoints():
    epoch = 1_403_636_579_000_000_000
    imu = np.array([epoch, epoch+10, epoch+20], dtype=np.int64)
    shifted = aligned_camera_times([epoch-3, epoch+17], 3, imu)
    np.testing.assert_array_equal(shifted, [epoch, epoch+20])
    samples = np.array([[0., 0., 0.], [10., 0., 0.], [20., 0., 0.]])
    np.testing.assert_allclose(sample_interpolated_imu(epoch+15, imu, samples), [15, 0, 0])
    np.testing.assert_allclose(sample_interpolated_imu(epoch+20, imu, samples), [20, 0, 0])


@pytest.mark.parametrize("camera,offset", [([0, 0], 0), ([0, 21], 0), ([0, 10], -1)])
def test_disordered_or_uncovered_camera_times_fail(camera, offset):
    with pytest.raises(ValueError):
        aligned_camera_times(camera, offset, [0, 10, 20])
