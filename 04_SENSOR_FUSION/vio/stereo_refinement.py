"""Bounded stereo extrinsic self-refinement from raw-image correspondences.

Three small-angle corrections of the right-from-left camera rotation are
estimated at initialization by minimizing a robust calibrated Sampson cost on
undistorted normalized coordinates. Baseline vector, intrinsics and distortion
stay at the supplied calibration. No ground truth, IMU or rectified raster
enters the fit. The frontend's 0.5 px per-track vertical gate is unchanged.
Gates are declared here and copied into every evaluation freeze.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from .geometry import rotation_exp, rotation_log, skew

GATES = {"max_correction_deg": 1.0, "min_fit_pairs": 6, "min_fit_correspondences": 600,
         "min_held_out_correspondences": 300, "min_held_out_per_quadrant": 50,
         "held_out_median_limit_px": .25,
         "held_out_statistic": "max over 2x2 image quadrants of |median signed rectified y_left-y_right| (px)",
         "window_seconds": 5.0, "pair_stride": 5, "orb_features": 1500, "mutual_ratio": .75,
         "robust_scale_px": 1.0, "perturbation": "R_rl' = Exp(delta) @ R_rl, delta in right camera frame",
         "vertical_gate_px_unchanged": .5}
UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)


@dataclass(frozen=True)
class StereoGeometry:
    """Supplied raw calibration: right-from-left rotation/translation and both camera models."""
    left_intrinsic: np.ndarray
    left_distortion: np.ndarray
    right_intrinsic: np.ndarray
    right_distortion: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    size: tuple

    def __post_init__(self):
        for name in ["left_intrinsic", "right_intrinsic", "rotation"]:
            matrix = getattr(self, name)
            if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
                raise ValueError(f"{name} must be a finite 3x3 matrix")
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(self.rotation), 1):
            raise ValueError("right-from-left rotation must be proper orthonormal")
        if self.translation.shape != (3,) or not np.isfinite(self.translation).all() or np.linalg.norm(self.translation) <= 0:
            raise ValueError("right-from-left translation must be a finite nonzero 3-vector")
        if self.left_intrinsic[0, 0] <= 0 or self.right_intrinsic[0, 0] <= 0:
            raise ValueError("focal lengths must be positive")

    def rotated(self, correction):
        return rotation_exp(np.asarray(correction, dtype=float)) @ self.rotation


def raw_correspondences(left, right, detector=None, matcher=None):
    """Mutual-ratio ORB matches between a raw stereo pair, as (left_px, right_px)."""
    if left.ndim != 2 or left.dtype != np.uint8 or right.shape != left.shape or right.dtype != np.uint8:
        raise ValueError("raw stereo inputs must be same-size grayscale uint8")
    detector = detector or cv2.ORB_create(nfeatures=GATES["orb_features"], edgeThreshold=12, fastThreshold=8)
    matcher = matcher or cv2.BFMatcher(cv2.NORM_HAMMING)
    keys_l, desc_l = detector.detectAndCompute(left, None)
    keys_r, desc_r = detector.detectAndCompute(right, None)
    if desc_l is None or desc_r is None or min(len(desc_l), len(desc_r)) < 2:
        return np.empty((0, 2)), np.empty((0, 2))

    def ratio(source, target):
        passed = {}
        for pair in matcher.knnMatch(source, target, k=2):
            if len(pair) == 2 and pair[0].distance < GATES["mutual_ratio"]*pair[1].distance:
                passed[pair[0].queryIdx] = pair[0]
        return passed
    forward, backward = ratio(desc_l, desc_r), ratio(desc_r, desc_l)
    accepted = [a for _, a in sorted(forward.items())
                if a.trainIdx in backward and backward[a.trainIdx].trainIdx == a.queryIdx]
    return (np.array([keys_l[a.queryIdx].pt for a in accepted], dtype=float).reshape(-1, 2),
            np.array([keys_r[a.trainIdx].pt for a in accepted], dtype=float).reshape(-1, 2))


def normalized_coordinates(pixels, intrinsic, distortion):
    pixels = np.asarray(pixels, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise ValueError("pixel correspondences must be finite Nx2")
    if len(pixels) == 0:
        return np.empty((0, 2))
    return cv2.undistortPointsIter(pixels.reshape(-1, 1, 2), intrinsic, distortion, None, None, UNDISTORT_CRITERIA).reshape(-1, 2)


def sampson_pixels(correction, left_normalized, right_normalized, geometry):
    """Signed Sampson epipolar distance scaled to ideal left pixels."""
    essential = skew(geometry.translation) @ geometry.rotated(correction)
    hl = np.c_[left_normalized, np.ones(len(left_normalized))]
    hr = np.c_[right_normalized, np.ones(len(right_normalized))]
    right_lines, left_lines = hl @ essential.T, hr @ essential
    numerator = np.sum(hr*right_lines, axis=1)
    scale = np.sqrt(np.sum(right_lines[:, :2]**2+left_lines[:, :2]**2, axis=1))
    return geometry.left_intrinsic[0, 0]*numerator/scale


def fit_rotation_correction(left_normalized, right_normalized, geometry):
    """Robust three-parameter rotation fit; returns (delta_rad, optimizer report)."""
    left_normalized, right_normalized = np.asarray(left_normalized, dtype=float), np.asarray(right_normalized, dtype=float)
    if (left_normalized.shape != right_normalized.shape or left_normalized.ndim != 2 or left_normalized.shape[1] != 2
            or len(left_normalized) < 3 or not np.isfinite([left_normalized, right_normalized]).all()):
        raise ValueError("fit requires at least three finite normalized correspondence pairs of equal shape")
    solution = least_squares(sampson_pixels, np.zeros(3), args=(left_normalized, right_normalized, geometry),
                             loss="soft_l1", f_scale=GATES["robust_scale_px"], xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=200)
    if not np.isfinite(solution.x).all():
        raise ValueError("stereo self-refinement produced nonfinite parameters")
    return solution.x, {"success": bool(solution.success), "evaluations": int(solution.nfev),
                        "cost": float(solution.cost), "message": str(solution.message)}


def rectified_vertical(left_pixels, right_pixels, geometry, correction=None):
    """Sparse rectified y_left-y_right and disparity (px) under the rectification the frontend would use."""
    correction = np.zeros(3) if correction is None else correction
    r1, r2, p1, p2, *_ = cv2.stereoRectify(geometry.left_intrinsic, geometry.left_distortion, geometry.right_intrinsic,
                                           geometry.right_distortion, geometry.size, geometry.rotated(correction),
                                           geometry.translation, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    if len(left_pixels) == 0:
        return np.empty(0), np.empty(0)
    left = cv2.undistortPointsIter(np.asarray(left_pixels, float).reshape(-1, 1, 2), geometry.left_intrinsic,
                                   geometry.left_distortion, r1, p1, UNDISTORT_CRITERIA).reshape(-1, 2)
    right = cv2.undistortPointsIter(np.asarray(right_pixels, float).reshape(-1, 1, 2), geometry.right_intrinsic,
                                    geometry.right_distortion, r2, p2, UNDISTORT_CRITERIA).reshape(-1, 2)
    return left[:, 1]-right[:, 1], left[:, 0]-right[:, 0]


def quadrant_labels(left_pixels, size):
    """0..3 = (left/right half) + 2*(top/bottom half) by raw left pixel position."""
    left_pixels = np.asarray(left_pixels, dtype=float)
    return (left_pixels[:, 0] >= size[0]/2).astype(int) + 2*(left_pixels[:, 1] >= size[1]/2).astype(int)


def _summary(vertical, sampson, quadrants):
    if len(vertical) == 0:
        return {"count": 0}
    counts = [int(np.sum(quadrants == q)) for q in range(4)]
    medians = [float(np.median(vertical[quadrants == q])) if counts[q] else None for q in range(4)]
    evaluable = min(counts) >= GATES["min_held_out_per_quadrant"]
    return {"count": int(len(vertical)), "median_abs_vertical_px": float(np.median(np.abs(vertical))),
            "median_signed_vertical_px": float(np.median(vertical)),
            "quadrant_counts": counts, "quadrant_signed_medians_px": medians,
            "max_abs_quadrant_signed_median_px": float(max(abs(m) for m in medians)) if evaluable else None,
            "share_within_gate": float(np.mean(np.abs(vertical) <= GATES["vertical_gate_px_unchanged"])),
            "sampson_median_abs_px": float(np.median(np.abs(sampson))),
            "sampson_quantiles_05_50_95_px": np.quantile(sampson, [.05, .5, .95]).tolist()}


def _evaluate(subset, geometry, correction):
    left, right = subset
    vertical, _ = rectified_vertical(left, right, geometry, correction)
    sampson = sampson_pixels(np.zeros(3) if correction is None else correction,
                             normalized_coordinates(left, geometry.left_intrinsic, geometry.left_distortion),
                             normalized_coordinates(right, geometry.right_intrinsic, geometry.right_distortion), geometry)
    return _summary(vertical, sampson, quadrant_labels(left, geometry.size))


def refine_from_pairs(pairs, geometry):
    """Fit on even-indexed pairs, hold out odd-indexed pairs; apply the declared decision rule.

    `pairs` is a sequence of (timestamp_ns, left_image, right_image) already subsampled.
    Returns a JSON-serializable report; raises ValueError only when neither the refined nor
    the supplied geometry satisfies the held-out gate.
    """
    if not isinstance(geometry, StereoGeometry):
        raise ValueError("geometry must be a StereoGeometry")
    pairs = list(pairs)
    if not pairs:
        raise ValueError("stereo self-refinement needs at least one stereo pair")
    detector = cv2.ORB_create(nfeatures=GATES["orb_features"], edgeThreshold=12, fastThreshold=8)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    sets = {"fit": [], "held_out": []}
    stamps = {"fit": [], "held_out": []}
    per_pair = []
    for index, (stamp, left, right) in enumerate(pairs):
        if left.shape[::-1] != geometry.size:
            raise ValueError("image dimensions differ from calibration")
        role = "fit" if index % 2 == 0 else "held_out"
        left_px, right_px = raw_correspondences(left, right, detector, matcher)
        sets[role].append((left_px, right_px))
        stamps[role].append(int(stamp))
        per_pair.append({"timestamp_ns": int(stamp), "role": role, "matches": int(len(left_px))})
    subsets = {role: (np.concatenate([a for a, _ in items]) if items else np.empty((0, 2)),
                      np.concatenate([b for _, b in items]) if items else np.empty((0, 2)))
               for role, items in sets.items()}
    counts = {"fit_pairs": len(sets["fit"]), "held_out_pairs": len(sets["held_out"]),
              "fit_correspondences": int(len(subsets["fit"][0])), "held_out_correspondences": int(len(subsets["held_out"][0]))}
    report = {"method": "robust Sampson fit of three right-camera rotation corrections on raw ORB correspondences",
              "ground_truth_used": False, "imu_used": False, "baseline_intrinsics_distortion_fixed": True,
              "gates": dict(GATES), "counts": counts, "pairs": per_pair,
              "fit_timestamps_ns": stamps["fit"], "held_out_timestamps_ns": stamps["held_out"]}
    if (counts["fit_pairs"] < GATES["min_fit_pairs"] or counts["fit_correspondences"] < GATES["min_fit_correspondences"]
            or counts["held_out_correspondences"] < GATES["min_held_out_correspondences"]):
        report.update(status="NOT_EVALUATED", decision="insufficient pairs or correspondences; supplied calibration retained",
                      correction={"rotvec_rad": [0., 0., 0.], "degrees_per_axis": [0., 0., 0.], "magnitude_deg": 0.})
        return report
    left_n = normalized_coordinates(subsets["fit"][0], geometry.left_intrinsic, geometry.left_distortion)
    right_n = normalized_coordinates(subsets["fit"][1], geometry.right_intrinsic, geometry.right_distortion)
    delta, optimizer = fit_rotation_correction(left_n, right_n, geometry)
    magnitude = float(np.degrees(np.linalg.norm(delta)))
    report["correction"] = {"rotvec_rad": delta.tolist(), "degrees_per_axis": np.degrees(delta).tolist(), "magnitude_deg": magnitude}
    report["optimizer"] = optimizer
    report["supplied"] = {role: _evaluate(subsets[role], geometry, None) for role in ["fit", "held_out"]}
    report["refined"] = {role: _evaluate(subsets[role], geometry, delta) for role in ["fit", "held_out"]}
    supplied_median = report["supplied"]["held_out"]["max_abs_quadrant_signed_median_px"]
    refined_median = report["refined"]["held_out"]["max_abs_quadrant_signed_median_px"]
    if supplied_median is None or refined_median is None:
        report.update(status="NOT_EVALUATED", decision="a held-out image quadrant has too few correspondences; supplied calibration retained")
        report["correction"] = {"rotvec_rad": [0., 0., 0.], "degrees_per_axis": [0., 0., 0.], "magnitude_deg": 0.,
                                "fitted_but_not_applied_deg": np.degrees(delta).tolist()}
        return report
    limit = GATES["held_out_median_limit_px"]
    checks = {"magnitude_within_bound": magnitude <= GATES["max_correction_deg"],
              "refined_held_out_within_limit": refined_median <= limit,
              "refined_not_worse_than_supplied": refined_median <= supplied_median,
              "supplied_held_out_within_limit": supplied_median <= limit}
    report["checks"] = checks
    if checks["magnitude_within_bound"] and checks["refined_held_out_within_limit"] and checks["refined_not_worse_than_supplied"]:
        report.update(status="APPLIED", decision="refined rotation applied to rectification")
    elif checks["supplied_held_out_within_limit"]:
        report.update(status="RETAINED_SUPPLIED", decision="refinement rejected by a gate; supplied calibration already within limit")
    else:
        raise ValueError("stereo self-refinement rejected and supplied calibration fails the held-out check: "
                         f"correction {magnitude:.4f} deg, refined held-out quadrant offset {refined_median:.4f} px, "
                         f"supplied held-out quadrant offset {supplied_median:.4f} px, limit {limit} px")
    return report


def applied_correction(report):
    """Rotation vector to feed stereo_rectification, or None when nothing is applied."""
    return np.asarray(report["correction"]["rotvec_rad"], dtype=float) if report["status"] == "APPLIED" else None


def correction_between(rotation_a, rotation_b):
    """Right-frame rotation vector delta with rotation_b = Exp(delta) @ rotation_a."""
    return rotation_log(rotation_b @ rotation_a.T)
