"""Summarize saved simulator evidence and visualize reconstruction failure regions."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from geometry import sample_surface


def analyze(directory):
    reconstruction = json.loads((directory / "replay/reconstruction.json").read_text())
    repeated = json.loads((directory / "replay_repeat/reconstruction.json").read_text())
    frames = json.loads((directory / "camera/frames.json").read_text())
    reached = json.loads((directory / "reached_waypoints.json").read_text())
    mission = json.loads((directory / "mission.json").read_text())
    cloud = np.loadtxt(directory / "replay/gazebo_reconstruction.ply", skiprows=7)
    with np.load(directory / "blade_mesh.npz") as mesh:
        truth = sample_surface(mesh["vertices"], mesh["faces"])
    distances = cKDTree(cloud).query(truth)[0]
    height_bins = np.linspace(0.2, 3.2, 7)
    height_coverage = []
    for bottom, top in zip(height_bins[:-1], height_bins[1:]):
        selected = (truth[:, 2] >= bottom) & (truth[:, 2] < top)
        height_coverage.append({"height_m": [float(bottom), float(top)], "samples": int(selected.sum()),
                                "coverage_4cm": float(np.mean(distances[selected] <= 0.04))})
    replay_equal = reconstruction["ply_sha256"] == repeated["ply_sha256"]
    if not replay_equal:
        raise RuntimeError("Recorded depth replay is not deterministic")
    report = {"recorded_replay": "PASS", "replay_ply_sha256": reconstruction["ply_sha256"],
              "maximum_camera_pose_alignment_s": max(frame["alignment_error_s"] for frame in frames),
              "maximum_estimated_arrival_error_m": max(float(np.linalg.norm(np.subtract(item["target_ned_m"], item["measured_ned_m"]))) for item in reached),
              "coverage_by_height": height_coverage,
              "limitation": "Low-resolution simulated depth; exact known extrinsics; known target crop; no RGB-only reconstruction"}
    (directory / "analysis.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    figure = plt.figure(figsize=(14, 5), layout="constrained")
    route_axes = figure.add_subplot(131)
    target = np.asarray(mission["waypoints"])
    measured = np.asarray([item["measured_ned_m"] for item in reached])
    route_axes.plot(target[:, 1], target[:, 0], "o-", ms=3, label="Planned ENU XY")
    route_axes.plot(measured[:, 1], measured[:, 0], ".", label="PX4 estimated arrival")
    route_axes.plot(5, 0, "kx", ms=12, label="Blade center")
    route_axes.set(xlabel="East [m]", ylabel="North [m]", title="64 actual SITL waypoint arrivals")
    route_axes.axis("equal")
    route_axes.legend(fontsize=8)
    cloud_axes = figure.add_subplot(132, projection="3d")
    cloud_axes.scatter(*cloud[::3].T, s=0.2, c="steelblue", label="Reconstructed depth")
    missing = truth[distances > 0.04]
    cloud_axes.scatter(*missing.T, s=1, c="orangered", label="Surface farther than 4 cm")
    cloud_axes.set(xlabel="Target X [m]", ylabel="Target Y [m]", zlabel="Z [m]",
                   title=f"Surface coverage: {100 * reconstruction['coverage_within_4cm']:.2f}%")
    cloud_axes.set_box_aspect((0.6, 0.35, 3))
    cloud_axes.legend(fontsize=7)
    image_axes = figure.add_subplot(133)
    with np.load(directory / "camera/frame_004.npz") as frame:
        image_axes.imshow(np.ma.masked_invalid(frame["depth"]), vmin=0.15, vmax=4, cmap="viridis")
    image_axes.set(title="Actual Gazebo metric depth: frame 4", xlabel="Pixel column", ylabel="Pixel row")
    figure.savefig(directory / "mission_evidence.png", dpi=160)
    plt.close(figure)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mission_directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.mission_directory), indent=2))
