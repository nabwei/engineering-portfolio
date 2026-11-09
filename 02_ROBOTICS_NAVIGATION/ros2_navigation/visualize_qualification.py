"""Plot the measured world trajectory and separate map/error evidence."""

import argparse
import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def visualize(directory):
    report = json.loads((directory / "acceptance.json").read_text())
    truth = json.loads((directory / "ground_truth.json").read_text())
    metadata = json.loads((directory / "slam_map.yaml").read_text())
    occupancy = plt.imread(directory / "slam_map.pgm")
    figure, axes = plt.subplots(1, 3, figsize=(15, 5), layout="constrained")
    axes[0].add_patch(Rectangle((-2.95, -2.95), 5.9, 5.9, fill=False, color="#555555"))
    axes[0].add_patch(Rectangle((-0.25, 1.25), 0.5, 0.5, color="#555555"))
    axes[0].plot(
        [pose["x"] for pose in truth],
        [pose["y"] for pose in truth],
        color="#187988",
        linewidth=1.5,
    )
    axes[0].set(
        title="Named simulator truth — world frame",
        xlabel="world x [m]",
        ylabel="world y [m]",
        aspect="equal",
        xlim=(-3.2, 3.2),
        ylim=(-3.2, 3.2),
    )
    origin_x, origin_y, yaw = metadata["origin"]
    if abs(yaw) > 1e-8:
        raise ValueError("qualification figure expects an axis-aligned SLAM grid")
    height, width = occupancy.shape
    axes[1].imshow(
        occupancy,
        cmap="gray",
        origin="upper",
        vmin=0,
        vmax=255,
        extent=(
            origin_x,
            origin_x + width * metadata["resolution"],
            origin_y,
            origin_y + height * metadata["resolution"],
        ),
    )
    axes[1].set(
        title="SLAM occupancy — map frame",
        xlabel="map x [m]",
        ylabel="map y [m]",
        aspect="equal",
    )
    indices = range(1, len(report["goals"]) + 1)
    axes[2].plot(
        indices,
        [goal["slam_goal_error_m"] for goal in report["goals"]],
        "o-",
        label="SLAM estimate",
    )
    axes[2].plot(
        indices,
        [goal["aligned_simulator_goal_error_m"] for goal in report["goals"]],
        "s-",
        label="Aligned simulator truth",
    )
    axes[2].axhline(0.3, color="#436f9c", linestyle="--", label="SLAM bound")
    axes[2].axhline(0.35, color="#bc7840", linestyle=":", label="Truth bound")
    axes[2].set(
        title="Endpoint errors — separate references",
        xlabel="Sequential goal",
        ylabel="Position error [m]",
        ylim=(0, 0.39),
        xticks=list(indices),
    )
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.2)
    figure.suptitle(
        f"{report['status']} — Actual Gazebo + SLAM + Nav2: {len(report['goals'])} goals / {report['simulated_seconds']:.2f} simulated s / {report['odometry_distance_m']:.2f} m\nMinimum sampled LiDAR clearance {report['minimum_sampled_lidar_clearance_m']:.3f} m; peak body tilt {report['peak_body_tilt_rad']:.4f} rad",
        fontsize=13,
    )
    figure.savefig(directory / "qualification_overview.png", dpi=180)
    plt.close(figure)
    if "aligned_pose_position_error_m" in report["goals"][0]:
        figure, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
        for axis, field, limit, unit, title in (
            (axes[0], "aligned_pose_position_error_m", 0.05, "m", "Position vector"),
            (axes[1], "aligned_pose_yaw_error_rad", 0.10, "rad", "Wrapped yaw"),
        ):
            axis.plot(indices, [goal[field] for goal in report["goals"]], "o-")
            axis.axhline(limit, color="#bc7840", linestyle="--", label="Declared limit")
            axis.set(
                title=title,
                xlabel="Sequential goal",
                ylabel=f"Disagreement [{unit}]",
                ylim=(
                    0,
                    max(
                        limit * 1.2, max(goal[field] for goal in report["goals"]) * 1.1
                    ),
                ),
                xticks=list(indices),
            )
            axis.grid(alpha=0.2)
            axis.legend()
        figure.suptitle(
            "Estimated pose versus native simulator truth\nOne initial SE(2) alignment, timestamp-paired poses"
        )
        figure.savefig(directory / "pose_agreement.png", dpi=180)
        plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    visualize(parser.parse_args().directory)
