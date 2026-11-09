"""Resolve the actual nested SDF sensor transform instead of assuming a root offset."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess


def resolve(source, scene, output):
    import sdformat14

    environment = dict(os.environ)
    environment["SDF_PATH"] = str(source / "Tools/simulation/gz/models")
    environment["GZ_SIM_RESOURCE_PATH"] = environment["SDF_PATH"]
    completed = subprocess.run(["gz", "sdf", "-p", str(scene / "models/portfolio_x500/model.sdf")],
                               env=environment, capture_output=True, text=True, check=True, timeout=30)
    root = sdformat14.Root()
    root.load_sdf_string(completed.stdout)
    sensor = root.model().link_by_name("portfolio_camera").sensor_by_name("depth")
    pose = sensor.semantic_pose().resolve("__model__")
    camera = sensor.camera_sensor()
    calibration = {"position_model_m": [pose.pos().x(), pose.pos().y(), pose.pos().z()],
                   "quaternion_xyzw": [pose.rot().x(), pose.rot().y(), pose.rot().z(), pose.rot().w()],
                   "width": camera.image_width(), "height": camera.image_height(),
                   "horizontal_fov_rad": camera.horizontal_fov().radian(),
                   "near_m": camera.near_clip(), "far_m": camera.far_clip(),
                   "method": "libsdformat14 sensor semantic pose resolved to __model__",
                   "resolved_sdf_sha256": hashlib.sha256(completed.stdout.encode()).hexdigest()}
    output.mkdir(parents=True, exist_ok=True)
    (output / "camera_calibration.json").write_text(json.dumps(calibration, indent=2), encoding="utf-8")
    (output / "resolved_camera_model.sdf").write_text(completed.stdout, encoding="utf-8")
    (output / "camera_resolution.log").write_text(completed.stderr, encoding="utf-8")
    shutil.copyfile(source / "Tools/simulation/gz/LICENSE", output / "PX4_GAZEBO_MODELS_LICENSE")
    return calibration


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(resolve(args.px4_source, args.scene, args.output), indent=2))
