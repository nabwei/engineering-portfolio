import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vio.eskf import ErrorStateFilter, State
from vio.geometry import trajectory_metrics
from vio.stereo import StereoOdometry
from vio.synthetic import Scene


def simulate():
    scene = Scene()
    tracker = StereoOdometry(scene.calibration)
    velocity = np.array([0.12, 0.025, 0.])
    estimator = ErrorStateFilter(State(velocity=velocity.copy()))
    inertial = ErrorStateFilter(State(velocity=velocity.copy()))
    random = np.random.default_rng(42)
    history, visual, truth = [], [], []
    accepted, failed = 0, 0
    measurement_covariance = np.diag([0.03**2]*3 + [0.01**2]*3)
    for step in range(301):
        if step:
            acceleration = np.array([0.015, -0.01, 9.80665]) + random.normal(0, 0.01, 3)
            gyro = random.normal(0, 0.0005, 3)
            estimator.predict(acceleration, gyro, 0.01)
            inertial.predict(acceleration, gyro, 0.01)
        if step % 10:
            continue
        position = velocity*step*0.01
        left, right = scene.stereo(position)
        try:
            pose = tracker.track(left, right)
        except ValueError:
            failed += 1
            continue
        accepted += int(estimator.update_pose(pose[:3, 3], pose[:3, :3], measurement_covariance))
        history.append(estimator.state.position.copy())
        visual.append(pose[:3, 3])
        truth.append(position)
    return np.asarray(history), np.asarray(visual), np.asarray(truth), {
        "provenance": "SYNTHETIC image-driven stereo + IMU; no public benchmark",
        "pose_updates_accepted": accepted, "visual_failures": failed,
        "inertial_final_error_m": float(np.linalg.norm(inertial.state.position - velocity*3)),
        "covariance_min_eigenvalue": float(np.linalg.eigvalsh(estimator.covariance).min()),
        "assumptions": ["known initial velocity and attitude", "rectified calibrated stereo",
                        "loose coupling ignores temporal visual-error correlation"]}


def save_demo(destination):
    destination.mkdir(parents=True, exist_ok=True)
    estimate, visual, truth, report = simulate()
    report["fused"] = trajectory_metrics(estimate, truth)
    report["visual"] = trajectory_metrics(visual, truth)
    (destination/"metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    np.savetxt(destination/"trajectory.csv", np.c_[truth, estimate, visual], delimiter=",",
               header="truth_x,truth_y,truth_z,fused_x,fused_y,fused_z,vo_x,vo_y,vo_z", comments="")
    figure, axis = plt.subplots(figsize=(8, 4))
    for trajectory, label in [(truth, "truth"), (visual, "stereo VO"), (estimate, "ESKF")]:
        axis.plot(trajectory[:, 0], trajectory[:, 1], label=label)
    axis.set(xlabel="x [m]", ylabel="y [m]", title="Synthetic image-driven stereo-inertial replay")
    axis.legend()
    figure.tight_layout()
    figure.savefig(destination/"trajectory.png", dpi=160)
    cv2.imwrite(str(destination/"synthetic_left.png"), Scene().stereo(np.zeros(3))[0])
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("../08_TEST_RESULTS/sensor_fusion/demo"))
    save_demo(parser.parse_args().output)
