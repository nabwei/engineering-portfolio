import numpy as np
from scipy.spatial.transform import Rotation

from reconstruct import backproject_metric


def calibration(width, height):
    return {"width": width, "height": height, "horizontal_fov_rad": 1.274,
            "near_m": .15, "far_m": 8, "position_model_m": [.15, 0, .25],
            "quaternion_xyzw": [0, 0, 0, 1]}


def test_plane_depth_and_camera_mount_transform():
    depth = np.full((3, 3), 2.0)
    position = np.array([10, 20, 30])
    points = backproject_metric(depth, position, [0, 0, 0, 1], calibration(3, 3))
    np.testing.assert_allclose(points[4], [12.15, 20, 30.25])
    np.testing.assert_allclose(points[:, 0], 12.15)
    assert points[0, 1] > points[4, 1] > points[8, 1]
    assert points[0, 2] > points[4, 2] > points[8, 2]


def test_world_rotation_and_invalid_depth_rejection():
    quaternion = Rotation.from_euler("z", 90, degrees=True).as_quat()
    points = backproject_metric(np.array([[np.nan, 2.0, np.inf]]), [0, 0, 0], quaternion, calibration(3, 1))
    np.testing.assert_allclose(points, [[0, 2.15, 0.25]], atol=1e-12)


def test_downward_camera_extrinsic_reconstructs_known_ground_plane():
    intrinsic = calibration(3, 3)
    intrinsic["position_model_m"] = [0, 0, 0]
    intrinsic["quaternion_xyzw"] = Rotation.from_euler("y", 90, degrees=True).as_quat()
    points = backproject_metric(np.full((3, 3), 2.0), [0, 0, 2], [0, 0, 0, 1], intrinsic)
    np.testing.assert_allclose(points[:, 2], 0, atol=1e-12)
