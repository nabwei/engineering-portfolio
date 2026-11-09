"""Fail explicitly when the requested ROS 2 / Gazebo / Nav2 stack is unavailable."""

import importlib.util
import json
import shutil
import subprocess


def inspect_environment():
    report = {name: shutil.which(name) for name in ["ros2", "gz", "colcon"]}
    report["rclpy"] = importlib.util.find_spec("rclpy") is not None
    if report["ros2"]:
        packages = subprocess.check_output(
            ["ros2", "pkg", "list"], text=True
        ).splitlines()
        report["required_packages"] = {
            name: name in packages
            for name in [
                "nav2_bringup",
                "slam_toolbox",
                "ros_gz_sim",
                "robot_state_publisher",
            ]
        }
    report["status"] = (
        "PASS"
        if all(report.get(name) for name in ["ros2", "gz", "colcon", "rclpy"])
        and all(report.get("required_packages", {"missing": False}).values())
        else "BLOCKED"
    )
    return report


if __name__ == "__main__":
    environment = inspect_environment()
    print(json.dumps(environment, indent=2))
    raise SystemExit(0 if environment["status"] == "PASS" else 2)
