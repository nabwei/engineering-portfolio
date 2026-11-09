"""Independent native Gazebo turn/stop test for rigid-body roll and pitch."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from run_integration import stop_group


def attitude(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def speed_near(poses, stamp):
    """Average native displacement velocity over a 0.1-second centred window."""
    start = min(poses, key=lambda pose: abs(pose["time_s"] - stamp + 0.05))
    end = min(poses, key=lambda pose: abs(pose["time_s"] - stamp - 0.05))
    duration = end["time_s"] - start["time_s"]
    if duration <= 0:
        raise ValueError("insufficient native samples for a velocity window")
    return math.hypot(end["x_m"] - start["x_m"], end["y_m"] - start["y_m"]) / duration


def probe(output):
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist

    poses = []

    def capture(message):
        for pose in message.pose:
            if pose.name == "portfolio_robot":
                roll, pitch = attitude(pose.orientation)
                poses.append(
                    {
                        "time_s": message.header.stamp.sec
                        + message.header.stamp.nsec / 1e9,
                        "roll_rad": roll,
                        "pitch_rad": pitch,
                        "z_m": pose.position.z,
                        "x_m": pose.position.x,
                        "y_m": pose.position.y,
                        "yaw_rad": math.atan2(
                            2
                            * (
                                pose.orientation.w * pose.orientation.z
                                + pose.orientation.x * pose.orientation.y
                            ),
                            1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2),
                        ),
                    }
                )

    transport = Node()
    transport.subscribe(Pose_V, "/world/portfolio_room/dynamic_pose/info", capture)
    publisher = transport.advertise("/cmd_vel_safe", Twist)
    deadline = time.monotonic() + 120
    epoch = None
    while epoch is None or poses[-1]["time_s"] - epoch < 17:
        if time.monotonic() > deadline:
            raise TimeoutError("native Gazebo balance probe timed out")
        if poses and epoch is None:
            epoch = poses[-1]["time_s"]
        stamp = poses[-1]["time_s"] - epoch if epoch is not None else 0
        command = Twist()
        if 2 <= stamp < 7:
            command.linear.x = 0.3
        elif 7 <= stamp < 15:
            command.angular.z = -0.8
        publisher.publish(command)
        time.sleep(0.02)
    publisher.publish(Twist())
    peak = max(max(abs(pose["roll_rad"]), abs(pose["pitch_rad"])) for pose in poses)
    distance = sum(
        math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
        for a, b in zip(poses, poses[1:])
    )
    turn = sum(
        abs(
            math.atan2(
                math.sin(b["yaw_rad"] - a["yaw_rad"]),
                math.cos(b["yaw_rad"] - a["yaw_rad"]),
            )
        )
        for a, b in zip(poses, poses[1:])
    )
    cruise = [pose for pose in poses if 4 <= pose["time_s"] - epoch <= 6]
    cruise_speed = (
        (
            math.hypot(
                cruise[-1]["x_m"] - cruise[0]["x_m"],
                cruise[-1]["y_m"] - cruise[0]["y_m"],
            )
            / (cruise[-1]["time_s"] - cruise[0]["time_s"])
        )
        if len(cruise) > 1
        else 0
    )
    rising_speed = speed_near(poses, epoch + 2.3)
    braking_speed = speed_near(poses, epoch + 7.3)
    passed = (
        peak <= 0.2
        and distance >= 0.8
        and turn >= 3
        and abs(turn - 6.4) <= 0.3
        and 0.25 <= cruise_speed <= 0.35
        and 0.08 <= rising_speed <= 0.22
        and 0.08 <= braking_speed <= 0.22
    )
    (output / "poses.json").write_text(json.dumps(poses))
    report = {
        "status": "PASS" if passed else "FAIL",
        "peak_tilt_rad": peak,
        "criterion_max_abs_roll_or_pitch_rad": 0.2,
        "forward_command_m_s": 0.3,
        "turn_command_rad_s": -0.8,
        "native_distance_m": distance,
        "native_absolute_turn_rad": turn,
        "expected_integrated_turn_rad": 6.4,
        "integrated_turn_tolerance_rad": 0.3,
        "native_cruise_speed_m_s": cruise_speed,
        "native_rising_speed_at_2_3_s_m_s": rising_speed,
        "native_braking_speed_at_7_3_s_m_s": braking_speed,
        "start_simulation_time_s": epoch,
        "simulated_seconds": poses[-1]["time_s"] - epoch,
    }
    (output / "acceptance.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    assert passed, report


def execute(output):
    output.mkdir(parents=True, exist_ok=False)
    os.environ["GZ_PARTITION"] = "portfolio_balance_" + uuid.uuid4().hex
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    world = Path(__file__).parent / "worlds" / "inspection_room.sdf"
    with (output / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            probe(output)
        finally:
            stop_group(process)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
