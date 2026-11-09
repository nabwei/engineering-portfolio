"""Predeclared V1-only observation-noise grid using frozen visual observations.

The original-noise replay must match actual image replay bit for bit before any
candidate is evaluated. Failed-camera frames and disjoint pairs are retained.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from run_relative_euroc import pose_matrix, save_trajectory, score
from vio.eskf import Noise
from vio.euroc import integrate_interval, numeric_records, sensor_configuration
from vio.initialization import quiet_initialization
from vio.relative import RelativePoseFilter

FACTORS = {"initial": (1., .5, .25), "extension": (1., .125, .0625)}


def load_development(trial):
    report = json.loads((trial / "report.json").read_text(encoding="utf-8"))
    if report.get("evaluation_role") != "ALL_V1_01_DEVELOPMENT_AFTER_ORB_RESERVED_FAILURE":
        raise ValueError("calibration requires explicitly observed V1 development evidence")
    with np.load(trial / "trajectory.npz", allow_pickle=False) as stored:
        arrays = {key: stored[key] for key in stored.files}
    if len(arrays["timestamps"]) != len(report["diagnostics"]):
        raise ValueError("cache and diagnostic lengths differ")
    if not np.array_equal(arrays["timestamps"], [row["timestamp_ns"] for row in report["diagnostics"]]):
        raise ValueError("cache timestamps differ from diagnostics")
    return report, arrays


def initial_filter(mav0, first_stamp, factory=RelativePoseFilter):
    imu = numeric_records(mav0 / "imu0/data.csv", 7)
    state, initialization = quiet_initialization(imu)
    configuration, extrinsic = sensor_configuration(mav0 / "imu0/sensor.yaml")
    if not np.allclose(extrinsic, np.eye(4)):
        raise ValueError("cached adapter requires body == IMU")
    noise = Noise(configuration["accelerometer_noise_density"], configuration["gyroscope_noise_density"],
                  configuration["accelerometer_random_walk"], configuration["gyroscope_random_walk"])
    estimator = factory(state, noise)
    if first_stamp > initialization["end_ns"]:
        integrate_interval(estimator, (initialization["end_ns"], first_stamp), imu)
    return estimator, imu


def recorded_update(estimator, visual, pending, covariance):
    if pending is None:
        estimator.clone_current_pose()
        return visual, {}
    relative = np.linalg.inv(pending) @ visual
    accepted = estimator.update_relative(relative[:3, 3], relative[:3, :3], covariance)
    return None, {"accepted": accepted, "nis": estimator.last_nis}


def diagnostic(estimator, original, update):
    row = {key: original[key] for key in ("timestamp_ns", "inliers", "visual_method") if key in original}
    row.update(update)
    row["min_covariance_eigenvalue"] = float(np.linalg.eigvalsh(estimator.covariance).min())
    row["accel_bias"], row["gyro_bias"] = estimator.state.accel_bias.tolist(), estimator.state.gyro_bias.tolist()
    return row


def cached_replay(mav0, cache, sigmas):
    report, arrays = cache
    stamps = arrays["timestamps"]
    estimator, imu = initial_filter(mav0, int(stamps[0]))
    covariance = np.diag(np.repeat(np.square(sigmas), 3))
    previous, pending, trajectory, diagnostics = int(stamps[0]), None, [], []
    for stamp, visual, original in zip(stamps, arrays["visual"], report["diagnostics"]):
        stamp = int(stamp)
        if stamp > previous:
            integrate_interval(estimator, (previous, stamp), imu)
        previous = stamp
        if not np.isfinite(visual).all():
            diagnostics.append(dict(original))
            trajectory.append((stamp, pose_matrix(estimator.state), None))
            continue
        pending, update = recorded_update(estimator, visual, pending, covariance)
        diagnostics.append(diagnostic(estimator, original, update))
        trajectory.append((stamp, pose_matrix(estimator.state), visual))
    return trajectory, diagnostics


def evaluate_candidate(arguments, cache, factor):
    original, arrays = cache
    sigmas = (original["visual_translation_sigma_m"]*factor, original["visual_rotation_sigma_rad"]*factor)
    trajectory, diagnostics = cached_replay(arguments.mav0, cache, sigmas)
    metrics = score(trajectory, arguments.mav0 / "state_groundtruth_estimate0/data.csv", original["development_end_ns"])
    report = dict(original, diagnostics=diagnostics, metrics=metrics, noise_multiplier=factor,
                  visual_translation_sigma_m=sigmas[0], visual_rotation_sigma_rad=sigmas[1])
    output = arguments.output / f"factor_{factor:g}"
    output.mkdir()
    save_trajectory(output, trajectory)
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if factor == 1.:
        checks = {"poses": bool(np.array_equal(np.array([row[1] for row in trajectory]), arrays["poses"])),
                  "diagnostics": diagnostics == original["diagnostics"], "metrics": metrics == original["metrics"]}
        (arguments.output / "baseline_exact_match.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
        if not all(checks.values()):
            raise RuntimeError("cached baseline differs from actual image run; calibration aborted")
    nis = [row["nis"] for row in diagnostics if "nis" in row]
    statistics = {"accepted": sum(row.get("accepted", False) for row in diagnostics),
                  "rejected": sum(row.get("accepted") is False for row in diagnostics),
                  "nis_mean": float(np.mean(nis)), "nis_p95": float(np.quantile(nis, .95)), "nis_max": float(max(nis)),
                  "visual_failures": sum("visual_failure" in row for row in diagnostics),
                  "descriptor_reacquisitions": [row for row in diagnostics if row.get("visual_method") == "descriptor_seeded_subpixel_flow"]}
    return {"factor": factor, "sigmas": sigmas, "diagnostic_summary": statistics, **metrics["development"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--grid", choices=FACTORS, default="initial")
    arguments = parser.parse_args()
    cache = load_development(arguments.trial)
    arguments.output.mkdir(parents=True, exist_ok=False)
    inputs = [arguments.trial / "trajectory.npz", arguments.trial / "report.json", Path(__file__)]
    manifest = {"factors": FACTORS[arguments.grid], "role": "V1 DEVELOPMENT ONLY", "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}}
    (arguments.output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    candidates = [evaluate_candidate(arguments, cache, factor) for factor in FACTORS[arguments.grid]]
    passed = [row for row in candidates if row["status"] == "PASS"]
    winner = min(passed, key=lambda row: row["vio"]["ate_se3_rmse_m"]) if passed else None
    result = {"status": "PASS" if winner else "FAIL", "selected": winner, "candidates": candidates}
    (arguments.output / "selection.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if winner else 1


if __name__ == "__main__":
    raise SystemExit(main())
