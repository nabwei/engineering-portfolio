"""Plot actual SLAM occupancy and odometry in their separate coordinate frames."""

import argparse
import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D


def visualize(directory):
    metadata = json.loads((directory / "slam_map.yaml").read_text())
    samples = json.loads((directory / "trajectory.json").read_text())
    acceptance = json.loads((directory / "acceptance.json").read_text())
    occupancy = plt.imread(directory / "slam_map.pgm")
    height, width = occupancy.shape
    resolution = metadata["resolution"]
    origin_x, origin_y, yaw = metadata["origin"]
    figure, axes = plt.subplots(1, 2, figsize=(11, 5), layout="constrained")
    transform = Affine2D().rotate(yaw).translate(origin_x, origin_y) + axes[0].transData
    axes[0].imshow(
        occupancy,
        cmap="gray",
        vmin=0,
        vmax=255,
        origin="upper",
        extent=[0, width * resolution, 0, height * resolution],
        transform=transform,
    )
    axes[0].set_xlim(origin_x, origin_x + width * resolution)
    axes[0].set_ylim(origin_y, origin_y + height * resolution)
    axes[0].set(
        title="SLAM occupancy — map frame",
        xlabel="map x [m]",
        ylabel="map y [m]",
        aspect="equal",
    )
    positions_x, positions_y = (
        [sample["x"] for sample in samples],
        [sample["y"] for sample in samples],
    )
    axes[1].plot(positions_x, positions_y, color="#137d91", linewidth=2)
    axes[1].scatter([positions_x[0]], [positions_y[0]], color="#398147", label="Start")
    axes[1].scatter(
        [positions_x[-1]], [positions_y[-1]], color="#d16b36", label="Finish"
    )
    axes[1].set(
        title="Recorded odometry — odom frame",
        xlabel="odom x [m]",
        ylabel="odom y [m]",
        aspect="equal",
    )
    axes[1].grid(alpha=0.2)
    axes[1].legend()
    figure.suptitle(
        f"ROS 2 navigation / {acceptance['backend']} backend\nGoal error {acceptance['goal_error_m']:.3f} m · sampled clearance ≥ {acceptance['minimum_sampled_lidar_clearance_m']:.3f} m",
        fontsize=13,
    )
    figure.savefig(directory / "navigation_evidence.png", dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    visualize(parser.parse_args().directory)
