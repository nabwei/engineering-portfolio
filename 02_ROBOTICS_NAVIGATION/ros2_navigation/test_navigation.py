"""Executable ROS integration acceptance; launch the stack before running this file."""

import json
import math
import time
from pathlib import Path
import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException


def wait_for(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError("ROS integration condition timed out")
        rclpy.spin_once(node, timeout_sec=0.1)


def save_map(grid, directory):
    pixels = bytearray()
    for row in range(grid.info.height - 1, -1, -1):
        for occupancy in grid.data[row * grid.info.width : (row + 1) * grid.info.width]:
            pixels.append(205 if occupancy < 0 else round(254 * (1 - occupancy / 100)))
    header = f"P5\n{grid.info.width} {grid.info.height}\n255\n".encode()
    (directory / "slam_map.pgm").write_bytes(header + pixels)
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )
    metadata = {
        "image": "slam_map.pgm",
        "resolution": grid.info.resolution,
        "origin": [origin.position.x, origin.position.y, yaw],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (directory / "slam_map.yaml").write_text(json.dumps(metadata, indent=2))


def shutdown_managed_nodes(node):
    for manager in ["lifecycle_manager_navigation", "slam_lifecycle_manager"]:
        client = node.create_client(ManageLifecycleNodes, f"/{manager}/manage_nodes")
        if not client.wait_for_service(timeout_sec=5):
            raise TimeoutError(f"{manager} shutdown service unavailable")
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        response = client.call_async(request)
        wait_for(node, response.done, 15)
        if not response.result().success:
            raise AssertionError(f"{manager} refused managed shutdown")
        node.destroy_client(client)


def wait_for_active_nodes(node):
    for name in [
        "controller_server",
        "planner_server",
        "bt_navigator",
        "velocity_smoother",
        "collision_monitor",
    ]:
        client = node.create_client(GetState, f"/{name}/get_state")
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError(f"{name} lifecycle service unavailable")
        deadline = time.monotonic() + 40
        while True:
            response = client.call_async(GetState.Request())
            wait_for(node, response.done, 5)
            if response.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                break
            if time.monotonic() > deadline:
                raise TimeoutError(f"{name} did not become active")
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_client(client)


def reject_outside_map_goal(node):
    planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not planner.wait_for_server(timeout_sec=5):
        raise TimeoutError("planner action unavailable")
    request = ComputePathToPose.Goal()
    request.goal.header.frame_id = "map"
    request.goal.header.stamp = node.get_clock().now().to_msg()
    request.goal.pose.position.x = 10000.0
    request.goal.pose.orientation.w = 1.0
    request.planner_id = "GridBased"
    sent = planner.send_goal_async(request)
    wait_for(node, sent.done, 5)
    if not sent.result().accepted:
        raise AssertionError("planner action rejected before validating boundary")
    finished = sent.result().get_result_async()
    wait_for(node, finished.done, 15)
    response = finished.result()
    if (
        response.status != GoalStatus.STATUS_ABORTED
        or response.result.error_code != ComputePathToPose.Result.GOAL_OUTSIDE_MAP
    ):
        raise AssertionError(
            f"outside-map goal returned {response.status}/{response.result.error_code}"
        )
    planner.destroy()


def execute(output, backend):
    rclpy.init()
    node = rclpy.create_node(
        "portfolio_navigation_acceptance",
        parameter_overrides=[Parameter("use_sim_time", value=backend == "gazebo")],
    )
    scans, odometry, maps = [], [], []
    subscriptions = [
        node.create_subscription(
            LaserScan, "/scan", scans.append, qos_profile_sensor_data
        ),
        node.create_subscription(Odometry, "/odom", odometry.append, 10),
        node.create_subscription(
            OccupancyGrid,
            "/map",
            maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    navigator = ActionClient(node, NavigateToPose, "navigate_to_pose")
    try:
        wait_for(node, lambda: scans and odometry and maps, 60)
        if (
            len(scans[-1].ranges) != 361
            or not any(cell == 0 for cell in maps[-1].data)
            or not any(cell > 50 for cell in maps[-1].data)
        ):
            raise AssertionError("scan or SLAM map contract failed")
        transform = None
        deadline = time.monotonic() + 20
        while transform is None:
            try:
                transform = buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                buffer.lookup_transform("base_link", "laser", rclpy.time.Time())
            except TransformException:
                if time.monotonic() > deadline:
                    raise TimeoutError("TF tree incomplete")
                rclpy.spin_once(node, timeout_sec=0.1)
        if not navigator.wait_for_server(timeout_sec=30):
            raise TimeoutError("Nav2 action server unavailable")
        wait_for_active_nodes(node)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = transform.transform.translation.x + 1.2
        goal.pose.pose.position.y = transform.transform.translation.y + 0.3
        goal.pose.pose.orientation.w = 1.0
        sent = navigator.send_goal_async(goal)
        wait_for(node, sent.done, 10)
        if not sent.result().accepted:
            raise AssertionError("Nav2 rejected test goal")
        finished = sent.result().get_result_async()
        wait_for(node, finished.done, 90)
        if finished.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise AssertionError(f"Nav2 goal status {finished.result().status}")
        final_pose = buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        ).transform.translation
        goal_error = math.hypot(
            final_pose.x - goal.pose.pose.position.x,
            final_pose.y - goal.pose.pose.position.y,
        )
        clearance = min(
            distance
            for scan in scans
            for distance in scan.ranges
            if math.isfinite(distance) and distance >= scan.range_min
        )
        if goal_error > 0.3 or clearance < 0.22:
            raise AssertionError(
                f"goal error {goal_error} or sampled clearance {clearance} violates contract"
            )
        reject_outside_map_goal(node)
        shutdown_managed_nodes(node)
        report = {
            "status": "PASS",
            "scan_messages": len(scans),
            "odom_messages": len(odometry),
            "slam_map_cells": len(maps[-1].data),
            "nav2_goal_status": finished.result().status,
            "backend": backend,
            "goal_error_m": goal_error,
            "minimum_sampled_lidar_clearance_m": clearance,
            "managed_shutdown": "PASS",
            "outside_map_goal_rejection": "PASS (GOAL_OUTSIDE_MAP=204)",
        }
        output.write_text(json.dumps(report, indent=2))
        save_map(maps[-1], output.parent)
        samples = [
            {
                "time_s": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                "x": message.pose.pose.position.x,
                "y": message.pose.pose.position.y,
            }
            for message in odometry
        ]
        (output.parent / "trajectory.json").write_text(json.dumps(samples, indent=2))
        print(json.dumps(report))
    finally:
        del subscriptions, listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--backend", choices=["core", "gazebo"], required=True)
    arguments = parser.parse_args()
    execute(arguments.output, arguments.backend)
