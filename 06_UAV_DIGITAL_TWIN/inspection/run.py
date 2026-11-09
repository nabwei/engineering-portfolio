import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
import pybullet as bullet
from scipy.spatial import cKDTree
from reconstruction import capture, voxel_fuse, write_ply
from scene import advance_to, coverage_path, create_scene, surface_samples


def execute(output):
    output.mkdir(parents=True, exist_ok=True)
    scene = create_scene()
    frames, clouds, tick = [], [], 0
    try:
        for index, waypoint in enumerate(coverage_path()):
            position, tick = advance_to(scene, waypoint, tick)
            image, cloud = capture(scene, position)
            if index in [0, 4, 8, 20, 36, 52]:
                Image.fromarray(image.astype("uint8")).save(
                    output / f"camera_{index:03d}.png"
                )
            frames.append(
                {
                    "sequence": index,
                    "tick": tick,
                    "time_s": tick / 240,
                    "position_m": position.tolist(),
                    "points": len(cloud),
                }
            )
            clouds.append(cloud)
        cloud = voxel_fuse(np.concatenate(clouds))
        distances, _ = cKDTree(cloud).query(surface_samples())
        coverage = float(np.mean(distances < 0.04))
        report = {
            "status": "PASS" if coverage >= 0.85 else "FAIL",
            "coverage_fraction": coverage,
            "coverage_definition": "fraction of 12000 seeded area-uniform reference surface samples within 4 cm of backprojected cloud",
            "reference_to_cloud_median_m": float(np.median(distances)),
            "cloud_points": len(cloud),
            "frames": len(frames),
            "simulation_seconds": tick / 240,
            "plant": "PyBullet force-controlled point mass, no PX4 or rotor model",
            "reconstruction": "simulated calibrated depth, known camera poses; not monocular SfM",
        }
        write_ply(output / "reconstruction.ply", cloud)
        (output / "mission_replay.json").write_text(json.dumps(frames, indent=2))
        (output / "coverage.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
        if report["status"] != "PASS":
            raise RuntimeError("coverage below acceptance threshold")
    finally:
        bullet.disconnect(scene[0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
