"""Exercise actual Nav2 planner failure codes with controlled map/TF inputs."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

from run_integration import stop_group
from test_navigation import wait_for, save_map


def enclosed_map(node):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.resolution = 0.05
    grid.info.width = grid.info.height = 120
    grid.info.origin.position.x = grid.info.origin.position.y = -3.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 14400
    for row in range(120):
        for column in range(120):
            x, y = -3 + column * 0.05, -3 + row * 0.05
            outer_wall = row in (0, 119) or column in (0, 119)
            ring = (0.4 <= x <= 1.6 and 0.4 <= y <= 1.6) and not (
                0.6 < x < 1.4 and 0.6 < y < 1.4
            )
            if outer_wall or ring:
                grid.data[row * 120 + column] = 100
    return grid


def planner_configuration(path):
    settings = {
        "planner_server": {
            "ros__parameters": {
                "planner_plugins": ["GridBased"],
                "GridBased": {
                    "plugin": "nav2_navfn_planner::NavfnPlanner",
                    "tolerance": 0.0,
                    "allow_unknown": False,
                },
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "global_frame": "map",
                    "robot_base_frame": "base_link",
                    "update_frequency": 5.0,
                    "publish_frequency": 5.0,
                    "robot_radius": 0.2,
                    "resolution": 0.05,
                    "track_unknown_space": True,
                    "plugins": ["static_layer", "inflation_layer"],
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_subscribe_transient_local": True,
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "inflation_radius": 0.25,
                    },
                }
            }
        },
    }
    path.write_text(yaml.safe_dump(settings))


def change_state(node, transition):
    client = node.create_client(ChangeState, "/planner_server/change_state")
    if not client.wait_for_service(timeout_sec=15):
        raise TimeoutError("planner lifecycle service missing")
    request = ChangeState.Request()
    request.transition.id = transition
    response = client.call_async(request)
    wait_for(node, response.done, 30)
    if not response.result().success:
        raise AssertionError(f"planner transition {transition} failed")
    node.destroy_client(client)


def transform(node, child, age):
    stamped = TransformStamped()
    stamped.header.frame_id = "map"
    stamped.child_frame_id = child
    stamped.header.stamp = (node.get_clock().now() - Duration(seconds=age)).to_msg()
    stamped.transform.rotation.w = 1.0
    return stamped


def publish_robot_transform(node, broadcaster):
    stamped = transform(node, "base_link", 0.0)
    stamped.transform.translation.x = stamped.transform.translation.y = -1.0
    broadcaster.sendTransform(stamped)


def characterize_latest_robot_transform(node, planner, timer):
    timer.cancel()
    deadline = time.monotonic() + 1.0
    wait_for(node, lambda: time.monotonic() > deadline, 2)
    response = request_path(node, planner, ("map", -1, 1))
    assert (
        response.status == GoalStatus.STATUS_SUCCEEDED
        and response.result.error_code == 0
    )
    timer.reset()
    deadline = time.monotonic() + 0.5
    wait_for(node, lambda: time.monotonic() > deadline, 2)
    recovered = request_path(node, planner, ("map", -1, 1))
    assert (
        recovered.status == GoalStatus.STATUS_SUCCEEDED
        and recovered.result.error_code == 0
    )
    return {
        "scenario": "upstream_planner_latest_transform_policy",
        "outage_error_code": response.result.error_code,
        "limitation": "planner accepts cached latest robot pose; velocity_guard enforces freshness independently",
        "recovered_path_poses": len(recovered.result.path.poses),
    }


def request_path(node, planner, target):
    frame, x, y = target
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = frame
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x, request.goal.pose.position.y = float(x), float(y)
    request.goal.pose.orientation.w = 1.0
    request.planner_id = "GridBased"
    sent = planner.send_goal_async(request)
    wait_for(node, sent.done, 5)
    if not sent.result().accepted:
        raise AssertionError("planner refused action before checking input")
    finished = sent.result().get_result_async()
    wait_for(node, finished.done, 15)
    return finished.result()


def verify_scenarios(node, planner, broadcaster):
    outcomes = []
    for name, target, expected in [
        ("reachable_control", ("map", -1, 1), 0),
        ("enclosed_free_goal", ("map", 1, 1), 208),
        ("missing_goal_transform", ("missing_goal_frame", -1, 1), 202),
        ("stale_goal_transform", ("stale_goal_frame", -1, 1), 202),
    ]:
        broadcaster.sendTransform(transform(node, "stale_goal_frame", 30.0))
        response = request_path(node, planner, target)
        expected_status = (
            GoalStatus.STATUS_SUCCEEDED if expected == 0 else GoalStatus.STATUS_ABORTED
        )
        assert response.status == expected_status, (name, response.status)
        assert response.result.error_code == expected, (
            name,
            response.result.error_code,
        )
        outcomes.append(
            {
                "scenario": name,
                "action_status": response.status,
                "error_code": response.result.error_code,
                "path_poses": len(response.result.path.poses),
            }
        )
    return outcomes


def run_checks(node, output):
    publisher = node.create_publisher(
        OccupancyGrid,
        "/map",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    grid = enclosed_map(node)
    save_map(grid, output)
    publisher.publish(grid)
    dynamic = TransformBroadcaster(node)
    robot_timer = node.create_timer(
        0.02, lambda: publish_robot_transform(node, dynamic)
    )
    change_state(node, Transition.TRANSITION_CONFIGURE)
    change_state(node, Transition.TRANSITION_ACTIVATE)
    planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    assert planner.wait_for_server(timeout_sec=10)
    outcomes = verify_scenarios(node, planner, dynamic)
    timer = node.create_timer(
        0.02, lambda: dynamic.sendTransform(transform(node, "fresh_goal_frame", 0.0))
    )
    deadline = time.monotonic() + 0.5
    wait_for(node, lambda: time.monotonic() > deadline, 2)
    response = request_path(node, planner, ("fresh_goal_frame", -1, 1))
    assert (
        response.status == GoalStatus.STATUS_SUCCEEDED
        and response.result.error_code == 0
    )
    outcomes.append(
        {
            "scenario": "fresh_transform_control",
            "error_code": 0,
            "path_poses": len(response.result.path.poses),
        }
    )
    node.destroy_timer(timer)
    outcomes.append(characterize_latest_robot_transform(node, planner, robot_timer))
    node.destroy_timer(robot_timer)
    change_state(node, Transition.TRANSITION_DEACTIVATE)
    change_state(node, Transition.TRANSITION_CLEANUP)
    (output / "acceptance.json").write_text(
        json.dumps({"status": "PASS", "scenarios": outcomes}, indent=2)
    )
    print(json.dumps(outcomes), flush=True)


def execute(output):
    output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = "188"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    configuration = output / "planner.yaml"
    planner_configuration(configuration)
    rclpy.init()
    node = rclpy.create_node("portfolio_planner_qualification")
    with (output / "planner.log").open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nav2_planner",
                "planner_server",
                "--ros-args",
                "--params-file",
                str(configuration.resolve()),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            run_checks(node, output)
        finally:
            stop_group(process)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
