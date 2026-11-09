"""Compare two real ROS bag replays with each recorded topic's ordered CDR bytes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import time

import rclpy
import rosbag2_py
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

from run_integration import stop_group


class StreamDigest:
    def __init__(self):
        self.count = 0
        self.digest = hashlib.sha256()

    def append(self, serialized):
        self.digest.update(struct.pack("<Q", len(serialized)))
        self.digest.update(serialized)
        self.count += 1

    def summary(self):
        return {"count": self.count, "ordered_cdr_sha256": self.digest.hexdigest()}


def recorded_streams(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    digests = {topic: StreamDigest() for topic in types}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        digests[topic].append(serialized)
    expected = {topic: digest.summary() for topic, digest in digests.items()}
    required = {"/scan", "/odom", "/tf", "/tf_static", "/clock", "/cmd_vel_safe"}
    assert required <= types.keys(), types
    assert all(summary["count"] > 0 for summary in expected.values()), expected
    return types, expected


def write_replay_qos(topics, output):
    path = output / "replay_qos.yaml"
    settings = {
        topic: {
            "reliability": "reliable",
            "durability": "volatile",
            "history": "keep_last",
            "depth": 20000,
        }
        for topic in topics
    }
    path.write_text(yaml.safe_dump(settings))
    return path


def receive_replay(node, player, digests, expected):
    deadline = time.monotonic() + 300
    while player.poll() is None:
        if time.monotonic() > deadline:
            raise TimeoutError("rosbag playback exceeded 300 seconds")
        rclpy.spin_once(node, timeout_sec=0.05)
    assert player.returncode == 0, player.returncode
    deadline = time.monotonic() + 10
    while any(
        digests[topic].count < summary["count"] for topic, summary in expected.items()
    ):
        if time.monotonic() > deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.05)


def replay_once(bag, output, types, expected):
    output.mkdir(parents=True, exist_ok=False)
    qos_path = write_replay_qos(types, output)
    rclpy.init()
    node = rclpy.create_node("portfolio_replay_verification")
    digests = {topic: StreamDigest() for topic in types}
    qos = QoSProfile(
        depth=20000,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscriptions = [
        node.create_subscription(
            get_message(message_type), topic, digests[topic].append, qos, raw=True
        )
        for topic, message_type in types.items()
    ]
    with (output / "player.log").open("w") as log:
        player = subprocess.Popen(
            [
                "ros2",
                "bag",
                "play",
                str(bag),
                "--rate",
                "3",
                "--delay",
                "2",
                "--disable-keyboard-controls",
                "--wait-for-all-acked",
                "10000",
                "--qos-profile-overrides-path",
                str(qos_path),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            receive_replay(node, player, digests, expected)
        finally:
            stop_group(player)
            del subscriptions
            node.destroy_node()
            rclpy.shutdown()
    received = {topic: digest.summary() for topic, digest in digests.items()}
    report = {
        "status": "PASS" if received == expected else "FAIL",
        "received": received,
        "expected": expected,
        "comparison": "per-topic ordered raw CDR; not cross-topic callback order",
    }
    (output / "acceptance.json").write_text(json.dumps(report, indent=2))
    assert received == expected, report
    print(
        json.dumps({"run": output.name, "status": "PASS", "received": received}),
        flush=True,
    )


def execute(bag, output):
    os.environ["ROS_DOMAIN_ID"] = "189"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    types, expected = recorded_streams(bag)
    for index in (1, 2):
        replay_once(bag, output / f"replay_{index}", types, expected)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    execute(arguments.bag, arguments.output)
