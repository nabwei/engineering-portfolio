"""Derive and save the tested Nav2 configuration from the installed Jazzy package."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def save_configuration(destination):
    source = (
        Path(get_package_share_directory("nav2_bringup"))
        / "params"
        / "nav2_params.yaml"
    )
    settings = yaml.safe_load(source.read_text())
    # Recorded software-rendered WSL runs can exceed the upstream 20 ms action handshake.
    settings["bt_navigator"]["ros__parameters"]["default_server_timeout"] = 2000
    controller = settings["controller_server"]["ros__parameters"]
    controller["goal_checker_plugins"] = ["general_goal_checker"]
    controller["general_goal_checker"] = {
        "plugin": "nav2_controller::SimpleGoalChecker",
        "stateful": True,
        "xy_goal_tolerance": 0.25,
        "yaw_goal_tolerance": 0.25,
    }
    controller["FollowPath"] = {
        "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "desired_linear_vel": 0.25,
        "lookahead_dist": 0.4,
        "use_velocity_scaled_lookahead_dist": False,
        "use_rotate_to_heading": True,
        "rotate_to_heading_angular_vel": 0.35,
        "max_angular_accel": 0.4,
        "transform_tolerance": 0.3,
    }
    controller["progress_checker"]["required_movement_radius"] = 0.2
    controller["progress_checker"]["movement_time_allowance"] = 20.0
    monitor = settings["collision_monitor"]["ros__parameters"]
    monitor["base_frame_id"] = "base_link"
    monitor["scan"]["min_height"] = -0.1
    smoother = settings["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [0.3, 0.0, 0.5]
    smoother["min_velocity"] = [-0.3, 0.0, -0.5]
    smoother["max_accel"] = [0.5, 0.0, 0.5]
    smoother["max_decel"] = [-0.5, 0.0, -0.5]
    destination.write_text(yaml.safe_dump(settings, sort_keys=False))
