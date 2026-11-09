"""Ten-goal physical simulation qualification against a running Gazebo stack."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from tf2_ros import Buffer, TransformListener

from test_navigation import (
    wait_for,
    wait_for_active_nodes,
    save_map,
    shutdown_managed_nodes,
)
from test_physics_balance import attitude
from pose_metrics import aligned_pose_error
from qualification_report import (
    preserve_acceptance_failure,
    write_and_require_acceptance,
)


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def yaw(rotation):
    return math.atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y**2 + rotation.z**2),
    )


class MissionEvidence:
    def __init__(self, node):
        self.scans, self.odometry, self.maps, self.truth = [], [], [], []
        self.subscriptions = [
            node.create_subscription(
                LaserScan, "/scan", self.scans.append, qos_profile_sensor_data
            ),
            node.create_subscription(Odometry, "/odom", self.odometry.append, 100),
            node.create_subscription(
                OccupancyGrid,
                "/map",
                self.maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        self.gazebo = GazeboNode()
        self.gazebo.subscribe(
            Pose_V, "/world/portfolio_room/dynamic_pose/info", self.capture_truth
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def capture_truth(self, message):
        for pose in message.pose:
            if pose.name == "portfolio_robot":
                stamped = TransformStamped()
                stamped.header.stamp.sec = message.header.stamp.sec
                stamped.header.stamp.nanosec = message.header.stamp.nsec
                stamped.header.frame_id = "world"
                stamped.child_frame_id = pose.name
                for axis in ("x", "y", "z"):
                    setattr(
                        stamped.transform.translation,
                        axis,
                        getattr(pose.position, axis),
                    )
                for axis in ("x", "y", "z", "w"):
                    setattr(
                        stamped.transform.rotation,
                        axis,
                        getattr(pose.orientation, axis),
                    )
                self.truth.append(stamped)

    def map_pose(self):
        return self.buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        ).transform

    def paired_poses(self, node):
        estimated = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        stamp = stamp_seconds(estimated)
        wait_for(node, lambda: stamp_seconds(self.truth[-1]) >= stamp, 2)
        truth = min(self.truth, key=lambda sample: abs(stamp_seconds(sample) - stamp))
        skew = abs(stamp_seconds(truth) - stamp)
        assert skew <= 0.02, f"pose timestamp skew {skew} s exceeds 0.02 s"
        return estimated, truth, skew

    def save(self, output):
        save_map(self.maps[-1], output)
        samples = [
            {
                "time_s": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
            }
            for msg in self.odometry
        ]
        (output / "trajectory.json").write_text(json.dumps(samples))
        truth = [
            {
                "time_s": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "x": msg.transform.translation.x,
                "y": msg.transform.translation.y,
                "yaw": yaw(msg.transform.rotation),
                "roll_rad": attitude(msg.transform.rotation)[0],
                "pitch_rad": attitude(msg.transform.rotation)[1],
            }
            for msg in self.truth
        ]
        (output / "ground_truth.json").write_text(json.dumps(truth))


def navigate(node, navigator, target):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = target
    goal.pose.pose.orientation.w = 1.0
    started = time.monotonic()
    sent = navigator.send_goal_async(goal)
    wait_for(node, sent.done, 10)
    assert sent.result().accepted, "Nav2 rejected endurance goal"
    finished = sent.result().get_result_async()
    wait_for(node, finished.done, 90)
    assert finished.result().status == GoalStatus.STATUS_SUCCEEDED, (
        finished.result().status
    )
    return time.monotonic() - started


def map_to_world_alignment(node, evidence):
    estimated_stamp, truth_stamp, _ = evidence.paired_poses(node)
    estimated, truth = estimated_stamp.transform, truth_stamp.transform
    angle = yaw(truth.rotation) - yaw(estimated.rotation)
    cosine, sine = math.cos(angle), math.sin(angle)
    tx = (
        truth.translation.x
        - cosine * estimated.translation.x
        + sine * estimated.translation.y
    )
    ty = (
        truth.translation.y
        - sine * estimated.translation.x
        - cosine * estimated.translation.y
    )
    return cosine, sine, tx, ty


def goal_errors(node, evidence, target, alignment):
    estimated_stamp, truth_stamp, skew = evidence.paired_poses(node)
    estimated = estimated_stamp.transform.translation
    truth = truth_stamp.transform.translation
    estimated_pose = (estimated.x, estimated.y, yaw(estimated_stamp.transform.rotation))
    truth_pose = (truth.x, truth.y, yaw(truth_stamp.transform.rotation))
    position_error, heading_error = aligned_pose_error(
        estimated_pose, truth_pose, alignment
    )
    cosine, sine, tx, ty = alignment
    wx = cosine * target[0] - sine * target[1] + tx
    wy = sine * target[0] + cosine * target[1] + ty
    return {
        "slam_goal_error_m": math.hypot(
            estimated.x - target[0], estimated.y - target[1]
        ),
        "aligned_simulator_goal_error_m": math.hypot(truth.x - wx, truth.y - wy),
        "aligned_pose_position_error_m": position_error,
        "aligned_pose_yaw_error_rad": heading_error,
        "estimated_pose_map_x_y_yaw": estimated_pose,
        "simulator_pose_world_x_y_yaw": truth_pose,
        "estimated_timestamp_s": stamp_seconds(estimated_stamp),
        "simulator_timestamp_s": stamp_seconds(truth_stamp),
        "pose_timestamp_skew_s": skew,
    }


def run_route(node, evidence, output):
    navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    assert navigator.wait_for_server(timeout_sec=30)
    wait_for_active_nodes(node)
    alignment = map_to_world_alignment(node, evidence)
    (output / "alignment.json").write_text(
        json.dumps(
            {
                "cosine": alignment[0],
                "sine": alignment[1],
                "translation_x_m": alignment[2],
                "translation_y_m": alignment[3],
                "policy": "one stationary initial SE(2) alignment; never fitted per goal",
            },
            indent=2,
        )
    )
    targets = [(1.2, 0.3), (1.2, -1.2), (-1.2, -1.2), (-1.2, 0.3), (0.0, 0.0)] * 2
    goals = []
    for index, target in enumerate(targets):
        duration = navigate(node, navigator, target)
        errors = goal_errors(node, evidence, target, alignment)
        errors["pose_agreement_pass"] = (
            errors["aligned_pose_position_error_m"] <= 0.05
            and errors["aligned_pose_yaw_error_rad"] <= 0.10
        )
        goals.append(
            {"index": index, "target_map_m": target, "wall_seconds": duration, **errors}
        )
        (output / "goals.json").write_text(json.dumps(goals, indent=2))
        evidence.save(output)
        print(json.dumps(goals[-1]), flush=True)
        assert errors["slam_goal_error_m"] <= 0.3, goals[-1]
        assert errors["aligned_simulator_goal_error_m"] <= 0.35, goals[-1]
    return goals


def mission_metrics(evidence):
    peak_tilt = max(
        (
            max(abs(angle) for angle in attitude(msg.transform.rotation))
            for msg in evidence.truth
        ),
        default=None,
    )
    poses = [message.pose.pose.position for message in evidence.odometry]
    distance = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))
    stamps = [
        msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        for msg in evidence.odometry
    ]
    clearance = min(
        (
            distance
            for scan in evidence.scans
            for distance in scan.ranges
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
        ),
        default=None,
    )
    return {
        "peak_body_tilt_rad": peak_tilt,
        "simulated_seconds": stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0,
        "odometry_distance_m": distance,
        "minimum_sampled_lidar_clearance_m": clearance,
        "scan_messages": len(evidence.scans),
        "odom_messages": len(evidence.odometry),
        "ground_truth_messages": len(evidence.truth),
    }


def execute(output):
    rclpy.init()
    node = rclpy.create_node(
        "portfolio_endurance",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    evidence = MissionEvidence(node)
    try:
        with preserve_acceptance_failure(output):
            wait_for(
                node,
                lambda: evidence.scans
                and evidence.odometry
                and evidence.maps
                and evidence.truth,
                80,
            )
            wait_for(
                node,
                lambda: evidence.buffer.can_transform(
                    "map", "base_link", rclpy.time.Time()
                ),
                20,
            )
            goals = run_route(node, evidence, output)
            metrics = mission_metrics(evidence)
            write_and_require_acceptance(output, goals, metrics)
            print(json.dumps(metrics), flush=True)
    finally:
        if evidence.maps and evidence.odometry:
            evidence.save(output)
        try:
            shutdown_managed_nodes(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
