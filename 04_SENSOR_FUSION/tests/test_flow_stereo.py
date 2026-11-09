import cv2
import numpy as np
import pytest

from vio.flow_stereo import FlowStereoOdometry, estimate_motion, stereo_points
from vio.geometry import rotation_exp, rotation_log
from vio.stereo import Calibration


def plane_images(calibration, camera_pose):
    # An exact plane-induced homography preserves continuous subpixel geometry;
    # unlike integer-stamped landmarks it does not quantize camera translation.
    random = np.random.default_rng(621)
    texture = cv2.GaussianBlur(random.integers(0, 256, (480, 640), np.uint8), (5, 5), .9)
    images = []
    for baseline in [0., calibration.baseline_m]:
        position = camera_pose[:3, 3] + camera_pose[:3, :3] @ [baseline, 0, 0]
        homography = camera_pose[:3, :3].T @ (np.eye(3)-np.outer(position, [0, 0, 1])/5.)
        homography = calibration.intrinsic @ homography @ np.linalg.inv(calibration.intrinsic)
        images.append(cv2.warpPerspective(texture, homography, (640, 480), flags=cv2.INTER_LINEAR))
    return images


def camera_calibration(extrinsic=None):
    return Calibration(np.array([[360., 0, 320], [0, 360., 240], [0, 0, 1]]), .25,
                       np.eye(4) if extrinsic is None else extrinsic)


def test_subpixel_stereo_recovers_known_metric_plane_depth():
    calibration = camera_calibration()
    # A noninteger disparity exercises LK refinement, not exact integer shifts.
    calibration.baseline_m = .237
    pixels, points = stereo_points(*plane_images(calibration, np.eye(4)), calibration)
    assert len(points) > 200
    assert np.median(np.abs(points[:, 2]-5.)) < .015
    reprojection = points @ calibration.intrinsic.T
    np.testing.assert_allclose(reprojection[:, :2]/reprojection[:, 2:], pixels, atol=1e-5)


@pytest.mark.parametrize("nonidentity_extrinsic", [False, True])
def test_subpixel_rotation_translation_and_camera_body_contract(nonidentity_extrinsic):
    cv2.setNumThreads(1)
    extrinsic = np.eye(4)
    if nonidentity_extrinsic:
        extrinsic[:3, :3], extrinsic[:3, 3] = rotation_exp([.3, -.4, .1]), [.2, -.1, .05]
    calibration = camera_calibration(extrinsic)
    camera_pose = np.eye(4)
    camera_pose[:3, :3], camera_pose[:3, 3] = rotation_exp([.008, -.012, .017]), [.0375, -.021, .014]
    tracker = FlowStereoOdometry(calibration)
    first = tracker.track(*plane_images(calibration, np.eye(4)))
    final = tracker.track(*plane_images(calibration, camera_pose))
    expected = extrinsic @ camera_pose @ np.linalg.inv(extrinsic)
    actual = np.linalg.inv(first) @ final
    np.testing.assert_allclose(actual[:3, 3], expected[:3, 3], atol=.01)
    assert np.linalg.norm(rotation_log(expected[:3, :3].T @ actual[:3, :3])) < .003
    assert tracker.last_inliers > 100


def test_blank_frame_preserves_reference_and_reacquires_deterministically():
    calibration = camera_calibration()
    tracker = FlowStereoOdometry(calibration)
    initial_images = plane_images(calibration, np.eye(4))
    tracker.track(*initial_images)
    before = tracker.world_from_camera.copy()
    with pytest.raises(ValueError, match="texture"):
        tracker.track(np.zeros_like(initial_images[0]), np.zeros_like(initial_images[1]))
    np.testing.assert_array_equal(tracker.world_from_camera, before)
    pose = np.eye(4)
    pose[0, 3] = .055
    moved_images = plane_images(calibration, pose)
    recovered = tracker.track(*moved_images)
    control = FlowStereoOdometry(calibration)
    control.track(*initial_images)
    np.testing.assert_array_equal(recovered, control.track(*moved_images))
    np.testing.assert_allclose(recovered[:3, 3], pose[:3, 3], atol=.01)


def test_zero_disparity_cannot_initialize_metric_odometry():
    calibration = camera_calibration()
    image = plane_images(calibration, np.eye(4))[0]
    with pytest.raises(ValueError, match="stereo flow depths"):
        FlowStereoOdometry(calibration).track(image, image.copy())


def test_large_image_gap_reacquires_metric_motion_without_resetting_origin():
    # The first recorded flow trial locked out after losing its reference.
    # A 108-pixel synthetic gap exercises actual descriptor-seeded reacquisition.
    calibration = camera_calibration()
    tracker = FlowStereoOdometry(calibration)
    tracker.track(*plane_images(calibration, np.eye(4)))
    pose = np.eye(4)
    pose[0, 3] = 1.5
    recovered = tracker.track(*plane_images(calibration, pose))
    assert tracker.last_method == "descriptor_seeded_subpixel_flow"
    np.testing.assert_allclose(recovered[:3, 3], [1.5, 0, 0], atol=.01)
    pose[0, 3] += .0375
    continued = tracker.track(*plane_images(calibration, pose))
    np.testing.assert_allclose(continued[:3, 3], pose[:3, 3], atol=.015)


def test_collinear_correspondences_are_rejected_before_pose_acceptance():
    points = np.c_[np.linspace(-1, 1, 30), np.zeros(30), np.full(30, 5.)]
    intrinsic = camera_calibration().intrinsic
    pixels = points @ intrinsic.T
    pixels = pixels[:, :2]/pixels[:, 2:]
    with pytest.raises(ValueError, match="degenerate"):
        estimate_motion(points, pixels, camera_calibration(), pixels-[18., 0.])


def test_robust_refined_pose_rejects_false_tracks_in_known_nonplanar_scene():
    random = np.random.default_rng(717)
    points = random.uniform([-2., -1.5, 4.], [2., 1.5, 9.], (250, 3))
    calibration = camera_calibration()
    intrinsic = calibration.intrinsic
    rotation, translation = np.array([.01, -.02, .03]), np.array([.08, -.03, .02])
    pixels = cv2.projectPoints(points, rotation, translation, intrinsic, None)[0].reshape(-1, 2)
    right = cv2.projectPoints(points, rotation, translation-[calibration.baseline_m, 0, 0], intrinsic, None)[0].reshape(-1, 2)
    pixels += random.normal(0, .05, pixels.shape)
    pixels[:60] = random.uniform([0, 0], [640, 480], (60, 2))
    estimated, inliers = estimate_motion(points, pixels, calibration, right)
    np.testing.assert_allclose(estimated[:3, 3], translation, atol=.003)
    assert np.linalg.norm(rotation_log(rotation_exp(rotation).T @ estimated[:3, :3])) < .001
    assert 170 <= inliers <= 195


def test_left_only_consistent_wrong_depth_is_rejected_by_current_right_camera():
    # Wrong reference depth can fit the left camera perfectly with a scaled
    # translation. The calibrated right baseline makes that hypothesis invalid.
    random = np.random.default_rng(19)
    points = random.uniform([-1., -.8, 4.], [1., .8, 8.], (100, 3))
    calibration = camera_calibration()
    rotation, translation = np.array([.01, -.02, .005]), np.array([.04, -.01, .02])
    left = cv2.projectPoints(points, rotation, translation, calibration.intrinsic, None)[0].reshape(-1, 2)
    right = cv2.projectPoints(points, rotation, translation-[calibration.baseline_m, 0, 0], calibration.intrinsic, None)[0].reshape(-1, 2)
    with pytest.raises(ValueError, match="four-view"):
        estimate_motion(points*2, left, calibration, right)
