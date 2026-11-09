"""Capture Gazebo metric depth and time-matched simulator truth poses."""
from collections import deque
import json
from pathlib import Path
import threading
import time

from gz.msgs10.image_pb2 import Image, R_FLOAT32
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
import numpy as np


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nsec * 1e-9


class DepthRecorder:
    def __init__(self, output):
        self.output = Path(output) / "camera"
        self.output.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.image = None
        self.poses = deque(maxlen=1024)
        self.last_pose_received = -float("inf")
        self.records = []
        self.node = Node()
        self.node.subscribe(Image, "/portfolio/depth", self.on_image)
        self.node.subscribe(Pose_V, "/world/portfolio_inspection/dynamic_pose/info", self.on_pose)

    def on_image(self, message):
        if message.pixel_format_type != R_FLOAT32 or message.step != message.width * 4:
            return
        with self.lock:
            self.image = (stamp(message), np.frombuffer(message.data, dtype="<f4").copy().reshape(message.height, message.width), time.monotonic())

    def on_pose(self, message):
        for pose in message.pose:
            if pose.name == "portfolio_x500_7":
                item = (stamp(message), [pose.position.x, pose.position.y, pose.position.z],
                        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                with self.lock:
                    self.poses.append(item)
                    self.last_pose_received = time.monotonic()

    def snapshot(self, sequence):
        with self.lock:
            if self.image is None or not self.poses:
                raise RuntimeError("No Gazebo depth image or model truth pose received")
            image_time, depth, received = self.image
            pose_time, position, quaternion = min(self.poses, key=lambda item: abs(item[0] - image_time))
            latest_time = self.poses[-1][0]
            pose_received = self.last_pose_received
        if time.monotonic() - min(received, pose_received) > 1.0:
            raise RuntimeError("Camera stream or pose stream stopped")
        if latest_time - image_time > 0.5 or abs(pose_time - image_time) > 0.03:
            raise RuntimeError("Camera image is stale or lacks a pose within 30 ms")
        np.savez_compressed(self.output / f"frame_{sequence:03}.npz", depth=depth,
                            position_enu=position, quaternion_xyzw=quaternion)
        self.records.append({"sequence": sequence, "image_sim_s": image_time,
                             "pose_sim_s": pose_time, "alignment_error_s": abs(pose_time - image_time)})
        (self.output / "frames.json").write_text(json.dumps(self.records, indent=2), encoding="utf-8")

    def close(self):
        self.node.unsubscribe("/portfolio/depth")
        self.node.unsubscribe("/world/portfolio_inspection/dynamic_pose/info")
