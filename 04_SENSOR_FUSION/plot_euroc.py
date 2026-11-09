"""Plot a saved replay and quantify how much the recorded reference moved."""

import argparse
import json

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import pyplot as plt


def plot_replay(directory):
    trajectory = np.loadtxt(directory / "trajectory.csv", delimiter=",", skiprows=1)
    timestamps = json.loads((directory / "timestamps_ns.json").read_text())
    elapsed = np.array([value-timestamps[0] for value in timestamps])*1e-9
    estimate, truth = trajectory[:, :3], trajectory[:, 3:]
    error = np.linalg.norm(estimate-truth, axis=1)
    motion = {"ground_truth_path_length_m": float(np.linalg.norm(np.diff(truth, axis=0), axis=1).sum()),
              "ground_truth_axis_span_m": np.ptp(truth, axis=0).tolist(),
              "raw_position_error_max_m": float(error.max()),
              "raw_position_error_final_m": float(error[-1])}
    (directory / "motion.json").write_text(json.dumps(motion, indent=2), encoding="utf-8")
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(truth[:, 0], truth[:, 1], label="Recorded ground truth")
    axes[0].plot(estimate[:, 0], estimate[:, 1], label="Stereo + IMU estimate")
    axes[0].set(xlabel="World x (m)", ylabel="World y (m)", title="Unaligned XY trajectory", aspect="equal")
    axes[0].legend()
    axes[1].plot(elapsed, error, label="Fused estimate")
    report = json.loads((directory / "metrics.json").read_text())
    if "diagnostics" in report:
        visual_error = [entry["visual_raw_position_error_m"] for entry in report["diagnostics"]]
        axes[1].plot(elapsed, visual_error, label="Standalone visual odometry")
    axes[1].legend()
    axes[1].set(xlabel="Time from first stereo pair (s)", ylabel="Raw position error (m)", title="No alignment or scale adjustment")
    for axis in axes:
        axis.grid(alpha=.3)
    figure.suptitle("Bounded EuRoC mirror replay — ground-truth-assisted initialization")
    figure.tight_layout()
    figure.savefig(directory / "trajectory.png", dpi=140)
    plt.close(figure)
    print(json.dumps(motion, indent=2))


if __name__ == "__main__":
    from pathlib import Path

    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    plot_replay(parser.parse_args().directory)
