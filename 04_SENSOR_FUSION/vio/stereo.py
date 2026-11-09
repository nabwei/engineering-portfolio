from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Calibration:
    intrinsic: np.ndarray
    baseline_m: float
    body_from_camera: np.ndarray

    def __post_init__(self):
        if self.intrinsic.shape != (3, 3) or not np.isfinite(self.intrinsic).all():
            raise ValueError("intrinsic must be finite 3x3")
        if self.intrinsic[0, 0] <= 0 or self.intrinsic[1, 1] <= 0:
            raise ValueError("focal lengths must be positive")
        if not np.isfinite(self.baseline_m) or self.baseline_m <= 0:
            raise ValueError("stereo baseline must be positive")
        transform = self.body_from_camera
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("body_from_camera must be finite 4x4")
        if not np.allclose(transform[3], [0, 0, 0, 1]):
            raise ValueError("invalid homogeneous transform")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(rotation), 1):
            raise ValueError("extrinsic rotation must be proper orthonormal")


def triangulate_rectified(left_pixels, right_pixels, calibration):
    disparity = left_pixels[:, 0] - right_pixels[:, 0]
    accepted = (disparity > 0.75) & (np.abs(left_pixels[:, 1]-right_pixels[:, 1]) < 1.5)
    intrinsic = calibration.intrinsic
    depth = intrinsic[0, 0] * calibration.baseline_m / disparity[accepted]
    rays = np.c_[left_pixels[accepted], np.ones(accepted.sum())] @ np.linalg.inv(intrinsic).T
    return rays * depth[:, None], accepted


def descriptor_matches(first, second):
    if first is None or second is None or len(first) < 2 or len(second) < 2:
        return []
    candidates = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(first, second, k=2)
    return [pair[0] for pair in candidates if len(pair) == 2 and pair[0].distance < 0.7*pair[1].distance]


class StereoOdometry:
    def __init__(self, calibration):
        self.calibration = calibration
        self.detector = cv2.ORB_create(nfeatures=2000, edgeThreshold=12, fastThreshold=8)
        self.world_from_camera = np.eye(4)
        self.reference_points = None
        self.reference_descriptors = None
        self.last_inliers = 0

    def _features(self, left, right):
        if left.ndim != 2 or left.dtype != np.uint8 or right.shape != left.shape or right.dtype != np.uint8:
            raise ValueError("rectified stereo inputs must be same-size grayscale uint8")
        key_left, desc_left = self.detector.detectAndCompute(left, None)
        key_right, desc_right = self.detector.detectAndCompute(right, None)
        matches = descriptor_matches(desc_left, desc_right)
        if len(matches) < 8:
            raise ValueError("insufficient stereo matches")
        pixels_left = np.array([key_left[m.queryIdx].pt for m in matches])
        pixels_right = np.array([key_right[m.trainIdx].pt for m in matches])
        points, accepted = triangulate_rectified(pixels_left, pixels_right, self.calibration)
        descriptors = np.array([desc_left[m.queryIdx] for m in matches])[accepted]
        if len(points) < 8:
            raise ValueError("insufficient valid stereo depths")
        return key_left, desc_left, points, descriptors

    def track(self, left, right):
        keypoints, descriptors, points, stereo_descriptors = self._features(left, right)
        if self.reference_points is not None:
            self._estimate_motion(keypoints, descriptors)
        self.reference_points = points
        self.reference_descriptors = stereo_descriptors
        camera_from_body = np.linalg.inv(self.calibration.body_from_camera)
        return self.world_from_camera @ camera_from_body

    def _estimate_motion(self, keypoints, descriptors):
        matches = descriptor_matches(self.reference_descriptors, descriptors)
        if len(matches) < 8:
            raise ValueError("insufficient temporal correspondences")
        points = np.array([self.reference_points[m.queryIdx] for m in matches])
        pixels = np.array([keypoints[m.trainIdx].pt for m in matches])
        cv2.setRNGSeed(42)
        solved, rotvec, translation, inliers = cv2.solvePnPRansac(
            points, pixels, self.calibration.intrinsic, None, iterationsCount=150,
            reprojectionError=1.5, confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
        if not solved or inliers is None or len(inliers) < 8:
            raise ValueError("PnP pose estimation failed")
        relative = np.eye(4)
        relative[:3, :3] = cv2.Rodrigues(rotvec)[0]
        relative[:3, 3] = translation.ravel()
        self.world_from_camera = self.world_from_camera @ np.linalg.inv(relative)
        self.last_inliers = len(inliers)
