"""Bounded EuRoC replay with explicitly ground-truth-assisted initialization."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from vio.eskf import ErrorStateFilter, Noise
from vio.euroc import camera_records, ground_truth_state, integrate_interval, numeric_records, sensor_configuration, stereo_rectification
from vio.geometry import rotation_log, trajectory_metrics
from vio.recovery import RecoverySupervisor
from vio.stereo import StereoOdometry


def replay(mav0, frame_count, provenance, recovery_mode="baseline"):
    if recovery_mode not in ["baseline", "bounded"]:
        raise ValueError("recovery mode must be baseline or bounded")
    cv2.setNumThreads(1)
    rectification = stereo_rectification(mav0)
    imu = numeric_records(mav0 / "imu0/data.csv", 7)
    truth = numeric_records(mav0 / "state_groundtruth_estimate0/data.csv", 17)
    left, right = camera_records(mav0 / "cam0"), camera_records(mav0 / "cam1")
    imu_config, body_imu = sensor_configuration(mav0 / "imu0/sensor.yaml")
    if not np.allclose(body_imu, np.eye(4), atol=1e-8):
        raise ValueError("this adapter requires body frame equal to imu0 frame")
    times = [t for t in left if t in right and max(imu[0][0], truth[0][0]) <= t <= min(imu[0][-1], truth[0][-1])][:frame_count]
    if len(times) < 3:
        raise ValueError("fewer than three stereo frames covered by IMU and ground truth")
    initial = ground_truth_state(times[0], truth)
    anchor = np.eye(4)
    anchor[:3, :3], anchor[:3, 3] = initial.rotation, initial.position
    anchor = anchor @ rectification.calibration.body_from_camera
    noise = Noise(imu_config["accelerometer_noise_density"], imu_config["gyroscope_noise_density"],
                  imu_config["accelerometer_random_walk"], imu_config["gyroscope_random_walk"])
    estimator, tracker = ErrorStateFilter(initial, noise), StereoOdometry(rectification.calibration)
    recovery = RecoverySupervisor() if recovery_mode == "bounded" else None
    times, trajectories, report = replay_frames((estimator, tracker, anchor), (times, left, right), (rectification, imu, truth), recovery)
    report["provenance_user_asserted"] = provenance
    report["input_frame_counts"] = {"left": len(left), "right": len(right), "synchronized_pairs": len(set(left) & set(right)), "requested_prefix": frame_count}
    return times, trajectories, report


def replay_frames(estimation, cameras, sensors, recovery=None):
    estimator, tracker, anchor = estimation
    times, left, right = cameras
    rectification, imu, truth = sensors
    trajectories, visual_trajectories, failures, rejected, diagnostics = [], [], [], [], []
    accepted = 0
    covariance = np.diag([.05**2]*3 + [.02**2]*3)
    for index, timestamp in enumerate(times):
        if index:
            integrate_interval(estimator, (times[index-1], timestamp), imu)
        pose, accepted_update, decision = None, False, "VISUAL_FAILURE"
        try:
            pose = anchor @ tracker.track(*rectification.images(left[timestamp], right[timestamp]))
            if recovery is None:
                accepted_update = estimator.update_pose(pose[:3, 3], pose[:3, :3], covariance)
                decision = "ACCEPTED" if accepted_update else "REJECTED"
            else:
                decision = recovery.update(estimator, timestamp, pose[:3, 3], pose[:3, :3], covariance)
                accepted_update = decision == "ACCEPTED"
            accepted += int(accepted_update)
            if not accepted_update:
                rejected.append(timestamp)
        except ValueError as error:
            if index == 0:
                raise ValueError("first stereo frame must initialize the visual coordinate frame") from error
            failures.append({"timestamp_ns": timestamp, "reason": str(error)})
            if recovery is not None:
                recovery.missing()
        reference = ground_truth_state(timestamp, truth)
        trajectories.append(np.r_[estimator.state.position, reference.position])
        if pose is not None:
            visual_trajectories.append(np.r_[pose[:3, 3], reference.position])
        diagnostics.append({"time_s": (timestamp-times[0])*1e-9,
                            "pose_update_accepted": bool(accepted_update),
                            "update_decision": decision,
                            "pnp_inliers": tracker.last_inliers,
                            "visual_raw_position_error_m": None if pose is None else float(np.linalg.norm(pose[:3, 3]-reference.position)),
                            "visual_orientation_error_rad": None if pose is None else float(np.linalg.norm(rotation_log(reference.rotation.T @ pose[:3, :3]))),
                            "filter_raw_position_error_m": float(np.linalg.norm(estimator.state.position-reference.position)),
                            "filter_orientation_error_rad": float(np.linalg.norm(rotation_log(reference.rotation.T @ estimator.state.rotation))),
                            "accel_bias_norm": float(np.linalg.norm(estimator.state.accel_bias)),
                            "gyro_bias_norm": float(np.linalg.norm(estimator.state.gyro_bias))})
    trajectories = np.array(trajectories)
    visual_trajectories = np.array(visual_trajectories)
    report = {"initialization": "ground-truth position, attitude, velocity and biases at first frame; no later ground-truth pose updates",
              "frames": len(times), "duration_s": (times[-1]-times[0])*1e-9, "pose_updates_accepted": accepted,
              "visual_failures": failures, "metrics": trajectory_metrics(trajectories[:, :3], trajectories[:, 3:]),
              "pose_updates_rejected_timestamps_ns": rejected, "diagnostics": diagnostics,
              "standalone_visual_metrics": trajectory_metrics(visual_trajectories[:, :3], visual_trajectories[:, 3:]) if len(visual_trajectories) >= 3 else None,
              "recovery_mode": "baseline" if recovery is None else "bounded",
              "recovery_policy": None if recovery is None else asdict(recovery.policy),
              "recovery_events": [] if recovery is None else recovery.events,
              "covariance_min_eigenvalue": float(np.linalg.eigvalsh(estimator.covariance).min()),
              "limitations": "bounded prefix, loose coupling, fixed visual covariance, no autonomous initialization or full-sequence benchmark"}
    return times, trajectories, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--provenance", choices=["PUBLIC_EUROC", "SYNTHETIC_EUROC_FORMAT"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recovery", choices=["baseline", "bounded"], default="baseline")
    arguments = parser.parse_args()
    if arguments.frames < 3:
        parser.error("--frames must be at least 3")
    times, trajectories, report = replay(arguments.mav0, arguments.frames, arguments.provenance, arguments.recovery)
    arguments.output.mkdir(parents=True, exist_ok=True)
    (arguments.output / "metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    np.savetxt(arguments.output / "trajectory.csv", trajectories, delimiter=",", header="estimate_x,estimate_y,estimate_z,truth_x,truth_y,truth_z", comments="")
    (arguments.output / "timestamps_ns.json").write_text(json.dumps(times), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
