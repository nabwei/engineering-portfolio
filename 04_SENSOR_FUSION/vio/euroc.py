"""EuRoC CSV/calibration adapter; camera timestamps are integer nanoseconds."""

import csv
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from .eskf import State
from .stereo import Calibration
from .stereo_refinement import StereoGeometry
from .timing import sample_interpolated_imu


def csv_rows(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return [row for row in csv.reader(stream) if row and not row[0].lstrip().startswith("#")]


def numeric_records(path, columns):
    rows = csv_rows(path)
    if any(len(row) != columns for row in rows) or len(rows) < 2:
        raise ValueError(f"expected at least two {columns}-column records in {path}")
    timestamps = np.array([int(row[0]) for row in rows], dtype=np.int64)
    samples = np.array([[float(entry) for entry in row[1:]] for row in rows])
    if np.any(np.diff(timestamps) <= 0) or not np.isfinite(samples).all():
        raise ValueError("records must have increasing timestamps and finite samples")
    return timestamps, samples


def camera_records(path):
    records = {}
    for row in csv_rows(path / "data.csv"):
        if len(row) != 2 or Path(row[1]).name != row[1]:
            raise ValueError("invalid camera CSV row or filename")
        timestamp = int(row[0])
        if timestamp in records:
            raise ValueError("duplicate camera timestamp")
        records[timestamp] = path / "data" / row[1]
    if list(records) != sorted(records):
        raise ValueError("camera timestamps must increase")
    return records


def sensor_configuration(path):
    configuration = yaml.safe_load(path.read_text(encoding="utf-8"))
    transform = np.array(configuration["T_BS"]["data"], dtype=float).reshape(4, 4)
    # Reuse the existing transform contract, including proper rotation.
    Calibration(np.eye(3), 1., transform)
    return configuration, transform


def camera_parameters(configuration):
    if configuration["camera_model"] != "pinhole" or configuration["distortion_model"] != "radial-tangential":
        raise ValueError("only EuRoC pinhole radial-tangential calibration is supported")
    fx, fy, cx, cy = configuration["intrinsics"]
    intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]], dtype=float)
    distortion = np.asarray(configuration["distortion_coefficients"], dtype=float)
    return intrinsic, distortion, tuple(configuration["resolution"])


@dataclass
class Rectification:
    calibration: Calibration
    left_maps: tuple
    right_maps: tuple
    size: tuple

    def images(self, left_path, right_path):
        raw = [cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) for path in [left_path, right_path]]
        if any(image is None or image.shape[::-1] != self.size for image in raw):
            raise ValueError("missing image or image dimensions differ from calibration")
        return tuple(cv2.remap(image, *maps, cv2.INTER_LINEAR) for image, maps in zip(raw, [self.left_maps, self.right_maps]))


def raw_stereo_geometry(mav0):
    """Supplied calibration as a StereoGeometry plus the left body extrinsic (no correction)."""
    left, body_left = sensor_configuration(mav0 / "cam0/sensor.yaml")
    right, body_right = sensor_configuration(mav0 / "cam1/sensor.yaml")
    kl, dl, size = camera_parameters(left)
    kr, dr, right_size = camera_parameters(right)
    if size != right_size:
        raise ValueError("stereo camera sizes must match")
    right_left = np.linalg.inv(body_right) @ body_left
    return StereoGeometry(kl, dl, kr, dr, right_left[:3, :3], right_left[:3, 3], size), body_left


def stereo_rectification(mav0, right_rotation_correction=None):
    """Rectification from the supplied calibration, optionally with a self-refined right rotation.

    The correction is a right-camera-frame rotation vector: R_rl' = Exp(delta) @ R_rl. The
    baseline vector, intrinsics, distortion and the LEFT camera's body extrinsic are unchanged.
    """
    geometry, body_left = raw_stereo_geometry(mav0)
    kl, dl, kr, dr, size = (geometry.left_intrinsic, geometry.left_distortion, geometry.right_intrinsic,
                            geometry.right_distortion, geometry.size)
    rotation = geometry.rotation
    if right_rotation_correction is not None:
        correction = np.asarray(right_rotation_correction, dtype=float)
        if correction.shape != (3,) or not np.isfinite(correction).all():
            raise ValueError("right rotation correction must be a finite rotation 3-vector")
        rotation = geometry.rotated(correction)
    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(kl, dl, kr, dr, size, rotation, geometry.translation, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    if abs(p2[1, 3]) > 1e-8 or p2[0, 3] >= 0:
        raise ValueError("frontend requires horizontal positive-disparity rectification")
    raw_rectified = np.eye(4)
    raw_rectified[:3, :3] = r1.T
    calibration = Calibration(p1[:3, :3], float(-p2[0, 3] / p2[0, 0]), body_left @ raw_rectified)
    left_maps = cv2.initUndistortRectifyMap(kl, dl, r1, p1, size, cv2.CV_32FC1)
    right_maps = cv2.initUndistortRectifyMap(kr, dr, r2, p2, size, cv2.CV_32FC1)
    return Rectification(calibration, left_maps, right_maps, size)


def ground_truth_state(target_ns, records):
    timestamps, samples = records
    interpolated = sample_interpolated_imu(target_ns, timestamps, samples)
    upper = min(max(int(np.searchsorted(timestamps, target_ns)), 1), len(timestamps)-1)
    pair = samples[upper-1:upper+1, 3:7]
    quaternions = pair[:, [1, 2, 3, 0]]
    span = int(timestamps[upper]) - int(timestamps[upper-1])
    weight = (int(target_ns) - int(timestamps[upper-1])) / span
    rotation = Slerp([0., 1.], Rotation.from_quat(quaternions))([weight]).as_matrix()[0]
    return State(interpolated[:3], interpolated[7:10], rotation, interpolated[13:16], interpolated[10:13])


def integrate_interval(estimator, interval_ns, imu_records):
    start, end = interval_ns
    timestamps, samples = imu_records
    if end <= start:
        raise ValueError("replay interval must increase")
    knots = [start, *timestamps[(timestamps > start) & (timestamps < end)].tolist(), end]
    for first, second in zip(knots, knots[1:]):
        midpoint = first + (second-first)//2
        measurement = sample_interpolated_imu(midpoint, timestamps, samples)
        estimator.predict(measurement[3:6], measurement[:3], (second-first)*1e-9)
