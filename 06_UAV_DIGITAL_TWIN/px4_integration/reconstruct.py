"""Replay Gazebo metric depth using recorded simulator truth extrinsics, not visual odometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from geometry import sample_surface

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "inspection"))
from reconstruction import voxel_fuse, write_ply  # noqa: E402 -- reuse existing Project L functions


def backproject_metric(depth, position, quaternion, calibration):
    height, width = depth.shape
    if (width, height) != (calibration["width"], calibration["height"]):
        raise ValueError("Depth dimensions do not match archived calibration")
    focal = width / (2 * np.tan(calibration["horizontal_fov_rad"] / 2))
    rows, columns = np.indices(depth.shape)
    valid = np.isfinite(depth) & (depth > calibration["near_m"]) & (depth < calibration["far_m"])
    ranges = depth[valid]
    # Gazebo camera link uses forward X, left Y, up Z; image rows increase downward.
    points = np.stack([ranges, -(columns[valid] + 0.5 - width / 2) * ranges / focal,
                       -(rows[valid] + 0.5 - height / 2) * ranges / focal], axis=-1)
    model_points = Rotation.from_quat(calibration["quaternion_xyzw"]).apply(points) + calibration["position_model_m"]
    return Rotation.from_quat(quaternion).apply(model_points) + position


def reconstruct(camera_directory, output):
    files = sorted(camera_directory.glob("frame_*.npz"))
    if not files:
        raise ValueError("No recorded Gazebo frames")
    calibration = json.loads((camera_directory.parent / "camera_calibration.json").read_text())
    with np.load(camera_directory.parent / "blade_mesh.npz") as geometry:
        vertices, faces = geometry["vertices"], geometry["faces"]
    low, high = vertices.min(axis=0) - 0.04, vertices.max(axis=0) + 0.04
    clouds = []
    for path in files:
        with np.load(path) as frame:
            cloud = backproject_metric(frame["depth"], frame["position_enu"], frame["quaternion_xyzw"], calibration)
        cloud -= [5, 0, 0]
        # Known simulator target bounding box: this is ground-truth-assisted segmentation.
        selected = ((cloud > low) & (cloud < high)).all(axis=1)
        clouds.append(cloud[selected])
    cloud = voxel_fuse(np.concatenate(clouds))
    if not len(cloud):
        raise ValueError("Recorded camera frames contain no target points")
    truth = sample_surface(vertices, faces)
    nearest = cKDTree(cloud).query(truth)[0]
    output.mkdir(parents=True, exist_ok=True)
    ply = output / "gazebo_reconstruction.ply"
    write_ply(ply, cloud)
    report = {"frames": len(files), "points": len(cloud),
              "coverage_within_4cm": float(np.mean(nearest <= 0.04)),
              "surface_samples": len(truth), "surface_distance_p95_m": float(np.quantile(nearest, 0.95)),
              "ply_sha256": hashlib.sha256(ply.read_bytes()).hexdigest(),
              "method": "metric simulated depth + simulator truth camera pose + known target crop",
              "not_claimed": ["image-only structure from motion", "real inspection accuracy"]}
    (output / "reconstruction.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(reconstruct(args.camera_directory, args.output), indent=2))
