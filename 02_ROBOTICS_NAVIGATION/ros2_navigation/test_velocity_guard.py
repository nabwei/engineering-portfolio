"""Real ROS topics exercise the compiled C++ watchdog and scan stop behavior."""

import math
import os
import signal
import subprocess
import time
import unittest
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class VelocityGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("velocity_guard_test")
        cls.commands = cls.node.create_publisher(Twist, "cmd_vel", 10)
        cls.scans = cls.node.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )
        cls.received = []
        cls.transform_age = 0.0
        cls.transforms = cls.node.create_publisher(TFMessage, "tf", 100)
        cls.subscription = cls.node.create_subscription(
            Twist, "cmd_vel_safe", cls.received.append, 10
        )
        cls.process = subprocess.Popen(
            ["ros2", "run", "portfolio_navigation", "velocity_guard"],
            start_new_session=True,
        )
        cls.addClassCleanup(cls.stop_process)
        deadline = time.monotonic() + 10
        while (
            cls.commands.get_subscription_count() == 0
            or cls.scans.get_subscription_count() == 0
            or cls.transforms.get_subscription_count() == 0
        ):
            if time.monotonic() > deadline:
                raise TimeoutError("guard ROS subscriptions did not appear")
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def stop_process(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
            try:
                cls.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(cls.process.pid, signal.SIGKILL)
                cls.process.wait(timeout=5)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def emit(self, distance, duration, age=0.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            command = Twist()
            command.linear.x = 0.7
            command.angular.z = 2.0
            self.commands.publish(command)
            self.publish_transform()
            if distance is not None:
                scan = LaserScan()
                scan.header.stamp = (
                    self.node.get_clock().now() - Duration(seconds=age)
                ).to_msg()
                scan.angle_min = -math.pi / 2
                scan.angle_increment = math.pi / 180
                scan.range_min, scan.range_max = 0.05, 10.0
                scan.ranges = [distance] * 181
                self.scans.publish(scan)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_transform(self):
        if self.transform_age is None:
            return
        transform = TransformStamped()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.header.stamp = (
            self.node.get_clock().now() - Duration(seconds=self.transform_age)
        ).to_msg()
        transform.transform.rotation.w = 1.0
        self.transforms.publish(TFMessage(transforms=[transform]))

    def test_clear_scan_clamps_speed_then_obstacle_and_dropout_stop(self):
        self.emit(2.0, 0.5)
        self.assertAlmostEqual(self.received[-1].linear.x, 0.3)
        self.assertAlmostEqual(self.received[-1].angular.z, 1.2)
        self.emit(0.2, 0.5)
        self.assertEqual(self.received[-1].linear.x, 0)
        self.emit(2.0, 0.5)
        self.assertAlmostEqual(self.received[-1].linear.x, 0.3)
        self.emit(None, 0.6)
        self.assertEqual(self.received[-1].linear.x, 0)
        for age in [1.0, -1.0]:
            with self.subTest(timestamp_age=age):
                self.emit(2.0, 0.3)
                self.emit(2.0, 0.4, age)
                self.assertEqual(self.received[-1].linear.x, 0)

    def test_missing_stale_future_robot_transform_stops_and_recovers(self):
        for transform_age in [None, 1.0, -1.0]:
            with self.subTest(transform_age=transform_age):
                self.transform_age = 0.0
                self.emit(2.0, 0.5)
                self.assertAlmostEqual(self.received[-1].linear.x, 0.3)
                self.transform_age = transform_age
                self.emit(2.0, 0.6)
                self.assertTrue(
                    all(
                        command.linear.x == 0 and command.angular.z == 0
                        for command in self.received[-5:]
                    )
                )
        self.transform_age = 0.0
        self.emit(2.0, 0.5)
        self.assertAlmostEqual(self.received[-1].linear.x, 0.3)


if __name__ == "__main__":
    unittest.main()
