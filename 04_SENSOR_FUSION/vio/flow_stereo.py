"""Subpixel rectified stereo and temporal LK flow with robust metric PnP.

The public contract matches StereoOdometry: return first-camera-from-current-
body poses. Failed frames leave the previous successful reference unchanged.
This frontend does not consume inertial measurements or ground truth.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from .stereo import descriptor_matches, triangulate_rectified

LK = {"winSize": (21, 21), "maxLevel": 4,
      "criteria": (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01)}
MIN_POINTS = 12


def checked_flow(first, second, pixels, initial=None):
    """Reject tracking loss, image-boundary exits and >0.5 pixel round trips."""
    pixels = np.ascontiguousarray(pixels, dtype=np.float32).reshape(-1, 1, 2)
    options = {} if initial is None else {"flags": cv2.OPTFLOW_USE_INITIAL_FLOW}
    seed = None if initial is None else np.ascontiguousarray(initial, dtype=np.float32).reshape(-1, 1, 2)
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(first, second, pixels, seed, **LK, **options)
    if tracked is None:
        raise ValueError("optical flow failed")
    reverse_seed = None if initial is None else pixels.copy()
    backward, back_status, _ = cv2.calcOpticalFlowPyrLK(second, first, tracked, reverse_seed, **LK, **options)
    if backward is None:
        raise ValueError("backward optical flow failed")
    target = tracked.reshape(-1, 2)
    valid = (status.ravel() != 0) & (back_status.ravel() != 0)
    valid &= np.linalg.norm(backward.reshape(-1, 2)-pixels.reshape(-1, 2), axis=1) <= .5
    valid &= np.isfinite(target).all(axis=1)
    valid &= (target[:, 0] >= 5) & (target[:, 0] < second.shape[1]-5)
    valid &= (target[:, 1] >= 5) & (target[:, 1] < second.shape[0]-5)
    return target, valid


def descriptor_seeded_flow(first, second):
    detector = cv2.ORB_create(nfeatures=2000, edgeThreshold=12, fastThreshold=8)
    keys_first, desc_first = detector.detectAndCompute(first, None)
    keys_second, desc_second = detector.detectAndCompute(second, None)
    matches = descriptor_matches(desc_first, desc_second)
    if len(matches) < MIN_POINTS:
        raise ValueError("insufficient descriptor seeds for flow reacquisition")
    pixels = np.array([keys_first[match.queryIdx].pt for match in matches], np.float32)
    seeds = np.array([keys_second[match.trainIdx].pt for match in matches], np.float32)
    pixels = cv2.cornerSubPix(first, pixels.reshape(-1, 1, 2), (3, 3), (-1, -1), LK["criteria"]).reshape(-1, 2)
    seeds = cv2.cornerSubPix(second, seeds.reshape(-1, 1, 2), (3, 3), (-1, -1), LK["criteria"]).reshape(-1, 2)
    tracked, valid = checked_flow(first, second, pixels, seeds)
    return pixels, tracked, valid


def triangulate_flow(pixels, right_pixels, valid, calibration):
    valid = valid & (np.abs(pixels[:, 1]-right_pixels[:, 1]) <= .5)
    points, accepted = triangulate_rectified(pixels[valid], right_pixels[valid], calibration)
    if len(points) < MIN_POINTS:
        raise ValueError("insufficient valid stereo flow depths")
    return points, valid, accepted


def stereo_points(left, right, calibration):
    corners = cv2.goodFeaturesToTrack(left, maxCorners=1500, qualityLevel=.01, minDistance=8, blockSize=7)
    if corners is None or len(corners) < MIN_POINTS:
        raise ValueError("insufficient image texture")
    corners = cv2.cornerSubPix(left, corners, (3, 3), (-1, -1), LK["criteria"]).reshape(-1, 2)
    right_pixels, valid = checked_flow(left, right, corners)
    try:
        points, valid, accepted = triangulate_flow(corners, right_pixels, valid, calibration)
    except ValueError:
        corners, right_pixels, valid = descriptor_seeded_flow(left, right)
        points, valid, accepted = triangulate_flow(corners, right_pixels, valid, calibration)
    return corners[valid][accepted], points


def reacquire_motion(reference, current, calibration):
    pixels, tracked, temporal_valid = descriptor_seeded_flow(reference[0], current[0])
    right_pixels, stereo_valid = checked_flow(reference[0], reference[1], pixels)
    points, valid, accepted = triangulate_flow(pixels, right_pixels, temporal_valid & stereo_valid, calibration)
    return stereo_motion(points, tracked[valid][accepted], current, calibration)


def require_spatial_support(points, pixels):
    if len(points) < MIN_POINTS:
        raise ValueError("insufficient temporal flow correspondences")
    spread = np.linalg.svd(pixels-pixels.mean(axis=0), compute_uv=False)
    spatial = np.linalg.svd(points-points.mean(axis=0), compute_uv=False)
    if spread[1] < 2 or spatial[1] < 1e-4:
        raise ValueError("degenerate collinear motion correspondences")


@dataclass
class StereoTracks:
    points: np.ndarray
    left: np.ndarray
    right: np.ndarray


def stereo_residual(parameters, tracks, calibration):
    rotation, translation = parameters[:3], parameters[3:]
    left = cv2.projectPoints(tracks.points, rotation, translation, calibration.intrinsic, None)[0]
    right_translation = translation - [calibration.baseline_m, 0, 0]
    right = cv2.projectPoints(tracks.points, rotation, right_translation, calibration.intrinsic, None)[0]
    return np.c_[left.reshape(-1, 2)-tracks.left, right.reshape(-1, 2)-tracks.right].ravel()


def stereo_inliers(parameters, tracks, calibration):
    residual = stereo_residual(parameters, tracks, calibration).reshape(-1, 4)
    left_error, right_error = np.linalg.norm(residual[:, :2], axis=1), np.linalg.norm(residual[:, 2:], axis=1)
    depths = (tracks.points @ cv2.Rodrigues(parameters[:3])[0].T + parameters[3:])[:, 2]
    return np.flatnonzero((left_error <= 1.) & (right_error <= 1.) & (depths > 0))


def pose_hypotheses(tracks, calibration):
    for pixels, shift in [(tracks.left, 0.), (tracks.right, calibration.baseline_m)]:
        cv2.setRNGSeed(42)
        solved, rotation, translation, inliers = cv2.solvePnPRansac(
            tracks.points, pixels, calibration.intrinsic, None, iterationsCount=200,
            reprojectionError=1., confidence=.999, flags=cv2.SOLVEPNP_EPNP)
        if solved and inliers is not None:
            translation[0, 0] += shift
            parameters = np.r_[rotation.ravel(), translation.ravel()]
            yield parameters, stereo_inliers(parameters, tracks, calibration)


def estimate_motion(points, pixels, calibration, right_pixels):
    require_spatial_support(points, pixels)
    tracks = StereoTracks(points, pixels, right_pixels)
    candidates = list(pose_hypotheses(tracks, calibration))
    if not candidates:
        raise ValueError("robust stereo PnP failed")
    parameters, inliers = max(candidates, key=lambda item: len(item[1]))
    if len(inliers) < max(MIN_POINTS, .4*len(points)):
        raise ValueError("PnP lacks consistent four-view support")
    supported = StereoTracks(points[inliers], pixels[inliers], right_pixels[inliers])
    require_spatial_support(supported.points, supported.left)
    refined = least_squares(stereo_residual, parameters, args=(supported, calibration), loss="huber", f_scale=1., max_nfev=30)
    inliers = stereo_inliers(refined.x, tracks, calibration)
    if not refined.success or len(inliers) < max(MIN_POINTS, .4*len(points)):
        raise ValueError("refined stereo PnP failed geometric checks")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = cv2.Rodrigues(refined.x[:3])[0], refined.x[3:]
    return transform, len(inliers)


def stereo_motion(points, pixels, current, calibration):
    if len(points) < MIN_POINTS:
        raise ValueError("insufficient temporal flow correspondences")
    right_pixels, valid = checked_flow(current[0], current[1], pixels)
    valid &= np.abs(pixels[:, 1]-right_pixels[:, 1]) <= .5
    valid &= pixels[:, 0]-right_pixels[:, 0] > .75
    return estimate_motion(points[valid], pixels[valid], calibration, right_pixels[valid])


class FlowStereoOdometry:
    def __init__(self, calibration):
        self.calibration = calibration
        self.world_from_camera = np.eye(4)
        self.reference_image = self.reference_pixels = self.reference_points = None
        self.reference_right = None
        self.last_inliers = 0
        self.last_method = "initialization"

    def _motion(self, current):
        try:
            tracked, valid = checked_flow(self.reference_image, current[0], self.reference_pixels)
            result = stereo_motion(self.reference_points[valid], tracked[valid], current, self.calibration)
            return *result, "direct_subpixel_flow"
        except ValueError:
            result = reacquire_motion((self.reference_image, self.reference_right), current, self.calibration)
            return *result, "descriptor_seeded_subpixel_flow"

    def track(self, left, right):
        if left.ndim != 2 or left.dtype != np.uint8 or right.shape != left.shape or right.dtype != np.uint8:
            raise ValueError("rectified stereo inputs must be same-size grayscale uint8")
        pixels, points = stereo_points(left, right, self.calibration)
        pose, inliers, method = self.world_from_camera, 0, "initialization"
        if self.reference_image is not None:
            motion, inliers, method = self._motion((left, right))
            pose = self.world_from_camera @ np.linalg.inv(motion)
        self.world_from_camera, self.last_inliers, self.last_method = pose, inliers, method
        self.reference_image, self.reference_pixels, self.reference_points = left.copy(), pixels, points
        self.reference_right = right.copy()
        return pose @ np.linalg.inv(self.calibration.body_from_camera)
