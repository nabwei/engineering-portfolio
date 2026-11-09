"""Configurable relative-pose VIO; ground truth is opened only by scoring."""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vio.eskf import Noise
from vio.euroc import (camera_records, ground_truth_state, integrate_interval,
                       numeric_records, raw_stereo_geometry, sensor_configuration, stereo_rectification)
from vio.geometry import rotation_log, trajectory_metrics
from vio.relative import RelativePoseFilter
from vio.replay_initialization import InitializationInputs, quiet_start, window_timestamps
from vio.stereo import StereoOdometry
from vio.stereo_refinement import GATES, applied_correction, refine_from_pairs

VISUAL_SIGMAS = (.01, .005)  # Translation meters, rotation radians.


@dataclass(frozen=True)
class ReplayConfiguration:
    tracker_factory: object = StereoOdometry
    observation_sigmas: tuple = VISUAL_SIGMAS
    initialize: object = quiet_start
    filter_factory: object = RelativePoseFilter
    refine_stereo: bool = False  # v2: stereo extrinsic self-refinement before initialization.


def refine_stereo_extrinsics(mav0, cameras, imu_times):
    """Run the declared self-refinement on the subsampled initialization window of raw pairs."""
    geometry, _ = raw_stereo_geometry(mav0)
    window = window_timestamps(cameras, imu_times)
    stride = int(GATES["pair_stride"])
    pairs = []
    for stamp in window[::stride]:
        images = [cv2.imread(str(camera[stamp]), cv2.IMREAD_GRAYSCALE) for camera in cameras]
        if any(image is None for image in images):
            raise ValueError("missing image inside the stereo self-refinement window")
        pairs.append((int(stamp), *images))
    report = refine_from_pairs(pairs, geometry)
    report["window"] = {"start_ns": int(window[0]), "end_ns": int(window[-1]), "pairs_in_window": int(len(window)),
                        "sampled_pairs": len(pairs), "stride": stride}
    return report


@dataclass
class StereoReplay:
    frames: tuple
    rectification: object
    tracker: object
    observation_sigmas: tuple = VISUAL_SIGMAS


def pose_matrix(state):
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = state.rotation, state.position
    return pose


def replay(mav0, end_ns=None, settings=None):
    cv2.setNumThreads(1)
    settings = settings or ReplayConfiguration()
    sigmas = np.asarray(settings.observation_sigmas)
    if sigmas.shape != (2,) or not np.isfinite(sigmas).all() or (sigmas <= 0).any():
        raise ValueError("visual observation sigmas must be two finite positive values (m, rad)")
    imu = numeric_records(mav0 / "imu0/data.csv", 7)
    rectification = stereo_rectification(mav0)
    configuration, body_imu = sensor_configuration(mav0 / "imu0/sensor.yaml")
    if not np.allclose(body_imu, np.eye(4)):
        raise ValueError("this adapter requires EuRoC body == IMU frame")
    noise = Noise(configuration["accelerometer_noise_density"], configuration["gyroscope_noise_density"],
                  configuration["accelerometer_random_walk"], configuration["gyroscope_random_walk"])
    cameras = camera_records(mav0 / "cam0"), camera_records(mav0 / "cam1")
    refinement = None
    if settings.refine_stereo:
        refinement = refine_stereo_extrinsics(mav0, cameras, imu[0])
        correction = applied_correction(refinement)
        if correction is not None:
            rectification = stereo_rectification(mav0, correction)
    initial = settings.initialize(InitializationInputs(imu, cameras, rectification, settings.tracker_factory))
    initialization = initial.report
    estimator = settings.filter_factory(initial.state, noise, initial.covariance)
    all_times = sorted(set(cameras[0]) & set(cameras[1]))
    times = [stamp for stamp in all_times if initial.first_replay_ns <= stamp <= imu[0][-1]
             and (end_ns is None or stamp <= end_ns)]
    if len(times) < 3:
        raise ValueError("fewer than three frames after autonomous initialization")
    if times[0] > initialization["end_ns"]:
        integrate_interval(estimator, (initialization["end_ns"], times[0]), imu)
    stream = StereoReplay((times, cameras), rectification, settings.tracker_factory(rectification.calibration), settings.observation_sigmas)
    trajectories, diagnostics = replay_frames(estimator, imu, stream)
    return {"initialization": initialization,
            "reset_policy": "no state-reset code path", "provenance": "UNVERIFIED_INPUT_FILES", "input_format": "EUROC_ASL",
            "visual_translation_sigma_m": float(sigmas[0]), "visual_rotation_sigma_rad": float(sigmas[1]),
            "image_pair_policy": ("every successful adjacent pair with explicit noisy-image-pose clone"
                                  if estimator.overlaps_frames else "disjoint successfully tracked image pairs"),
            "fusion_model": estimator.fusion_model,
            "frame_noise_beta": getattr(estimator, "frame_noise_beta", None),
            "stereo_refinement": refinement,
            "diagnostics": diagnostics}, trajectories


def replay_frames(estimator, imu, camera_stream):
    times, cameras = camera_stream.frames
    rectification, tracker = camera_stream.rectification, camera_stream.tracker
    visual_anchor = pose_matrix(estimator.state) @ rectification.calibration.body_from_camera
    previous_stamp, pair_start = times[0], None
    trajectory, diagnostics = [], []
    covariance = np.diag(np.repeat(np.square(camera_stream.observation_sigmas), 3))
    for index, stamp in enumerate(times):
        if stamp > previous_stamp:
            integrate_interval(estimator, (previous_stamp, stamp), imu)
        previous_stamp = stamp
        try:
            visual = visual_anchor @ tracker.track(*rectification.images(cameras[0][stamp], cameras[1][stamp]))
        except ValueError as failure:
            diagnostics.append({"timestamp_ns": stamp, "visual_failure": str(failure)})
            trajectory.append((stamp, pose_matrix(estimator.state), None))
            continue
        record = {"timestamp_ns": stamp, "inliers": tracker.last_inliers}
        method = getattr(tracker, "last_method", None)
        if method is not None:
            record["visual_method"] = method
        if pair_start is None:
            estimator.clone_current_pose()
            pair_start = visual
        else:
            relative = np.linalg.inv(pair_start) @ visual
            record["accepted"] = estimator.update_relative(relative[:3, 3], relative[:3, :3], covariance)
            record["nis"] = estimator.last_nis
            pair_start = visual if estimator.overlaps_frames else None
        record["min_covariance_eigenvalue"] = float(np.linalg.eigvalsh(estimator.covariance).min())
        record["accel_bias"] = estimator.state.accel_bias.tolist()
        record["gyro_bias"] = estimator.state.gyro_bias.tolist()
        diagnostics.append(record)
        trajectory.append((stamp, pose_matrix(estimator.state), visual))
        if index % 100 == 0:
            print(f"processed {index+1}/{len(times)} stereo frames", flush=True)
    return trajectory, diagnostics


def pose_metrics(timestamps, estimates, truth):
    metrics = trajectory_metrics(estimates[:, :3, 3], truth[:, :3, 3])
    metrics.pop("raw_rmse_m")  # Arbitrary initial world frame makes raw error meaningless.
    translation_errors, rotation_errors = [], []
    for first, stamp in enumerate(timestamps):
        second = int(np.searchsorted(timestamps, int(stamp)+1_000_000_000))
        if second >= len(timestamps) or timestamps[second]-stamp > 1_075_000_000:
            continue
        estimated_delta = np.linalg.inv(estimates[first]) @ estimates[second]
        truth_delta = np.linalg.inv(truth[first]) @ truth[second]
        error = np.linalg.inv(truth_delta) @ estimated_delta
        translation_errors.append(np.linalg.norm(error[:3, 3]))
        rotation_errors.append(np.linalg.norm(rotation_log(error[:3, :3]))*180/np.pi)
    metrics["rpe_1s_translation_rmse_m"] = float(np.sqrt(np.mean(np.square(translation_errors)))) if translation_errors else None
    metrics["rpe_1s_rotation_rmse_deg"] = float(np.sqrt(np.mean(np.square(rotation_errors)))) if rotation_errors else None
    metrics["rpe_pairs"] = len(translation_errors)
    return metrics


def score(trajectory, truth_path, development_end):
    truth_records = numeric_records(truth_path, 17)
    rows = [row for row in trajectory if truth_records[0][0] <= row[0] <= truth_records[0][-1]]
    timestamps = np.array([row[0] for row in rows], dtype=np.int64)
    estimated = np.array([row[1] for row in rows])
    visual = np.array([np.full((4, 4), np.nan) if row[2] is None else row[2] for row in rows])
    visual_available = np.isfinite(visual).all(axis=(1, 2))
    truth = np.array([pose_matrix(ground_truth_state(stamp, truth_records)) for stamp in timestamps])
    report = {}
    for name, selected in [("all", np.ones(len(rows), dtype=bool)),
                           ("development", timestamps <= development_end),
                           ("reserved", timestamps > development_end)]:
        if selected.sum() < 22:
            report[name] = {"status": "NOT RUN", "frames": int(selected.sum())}
            continue
        vio = pose_metrics(timestamps[selected], estimated[selected], truth[selected])
        common = selected & visual_available
        if common.sum() < 22:
            report[name] = {"status": "FAIL", "frames": int(selected.sum()), "vio": vio,
                            "reason": "insufficient visual baseline coverage"}
            continue
        baseline = pose_metrics(timestamps[common], visual[common], truth[common])
        paired_vio = pose_metrics(timestamps[common], estimated[common], truth[common])
        if vio["rpe_pairs"] == 0 or baseline["rpe_pairs"] == 0:
            report[name] = {"status": "NOT RUN", "reason": "no timestamp pairs spanning one second within 75 ms tolerance",
                            "frames": int(selected.sum()), "vio": vio, "stereo_only": baseline}
            continue
        passed = (vio["ate_se3_rmse_m"] <= 1 and vio["rpe_1s_translation_rmse_m"] <= .30
                  and vio["rpe_1s_rotation_rmse_deg"] <= 5
                  and paired_vio["ate_se3_rmse_m"] <= 1.2*baseline["ate_se3_rmse_m"])
        report[name] = {"status": "PASS" if passed else "FAIL", "frames": int(selected.sum()),
                        "vio": vio, "stereo_only": baseline, "vio_on_common_frames": paired_vio,
                        "visual_frames": int(common.sum()), "inertial_only_frames": int((selected & ~visual_available).sum())}
    return report


def development_boundary(mav0):
    # This labels the previously observed dataset prefix, outside estimation.
    with (mav0 / "state_groundtruth_estimate0/data.csv").open() as stream:
        truth_start = int(next(line for line in stream if not line.startswith("#")).split(",")[0])
    left, right = camera_records(mav0 / "cam0"), camera_records(mav0 / "cam1")
    original_prefix = sorted(stamp for stamp in set(left) & set(right) if stamp >= truth_start)
    if len(original_prefix) < 400:
        raise ValueError("qualification protocol requires at least 400 truth-overlapping camera timestamps")
    return original_prefix[399]


def save_trajectory(destination, trajectory):
    np.savez_compressed(destination / "trajectory.npz", timestamps=np.array([row[0] for row in trajectory], dtype=np.int64),
                        poses=np.array([row[1] for row in trajectory]),
                        visual=np.array([np.full((4, 4), np.nan) if row[2] is None else row[2] for row in trajectory]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, choices=[0, 400], default=400, help="0 means all frames, 400 is development prefix")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=False)
    boundary = development_boundary(arguments.mav0)
    report, trajectory = replay(arguments.mav0, boundary if arguments.frames else None)
    report["development_end_ns"] = boundary
    save_trajectory(arguments.output, trajectory)
    report["metrics"] = {"status": "NOT RUN", "reason": "scoring not yet completed"}
    (arguments.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["metrics"] = score(trajectory, arguments.mav0 / "state_groundtruth_estimate0/data.csv", boundary)
    (arguments.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))
    requested_segment = "development" if arguments.frames else "reserved"
    return 0 if report["metrics"][requested_segment]["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
