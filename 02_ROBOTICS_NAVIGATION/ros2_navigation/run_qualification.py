"""Bound one recorded Gazebo endurance run and clean up only owned processes."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid
from ament_index_python.packages import get_package_share_directory

from configure_navigation import save_configuration
from run_integration import stop_group
from qualification_report import archive_matching_world
from qualification_processes import recorded_processes


def start_logged(stack, command, destination, environment):
    logfile = stack.enter_context(destination.open("w"))
    child = subprocess.Popen(
        command,
        stdout=logfile,
        stderr=subprocess.STDOUT,
        env=environment,
        start_new_session=True,
    )
    stack.callback(stop_group, child)
    return child


def launch_command(configuration, slam_params_file=None):
    """Launch argv; the SLAM file is passed through only when explicitly requested."""
    command = [
        "ros2",
        "launch",
        "portfolio_navigation",
        "gazebo_navigation.launch.py",
        "params_file:=" + str(configuration.resolve()),
    ]
    if slam_params_file is not None:
        command.append("slam_params_file:=" + str(Path(slam_params_file).resolve()))
    return command


def record_slam_configuration(output, slam_params_file):
    """Save which SLAM parameter file the mission used (launch default v1 when None)."""
    report = {"launch_argument_passed": slam_params_file is not None}
    if slam_params_file is not None:
        source = Path(slam_params_file).resolve()
        content = source.read_bytes()
        (output / "slam_params.yaml").write_bytes(content)
        report.update(
            path=str(source), sha256=hashlib.sha256(content).hexdigest()
        )
    else:
        report["note"] = (
            "no slam_params_file argument; launch default config/slam_params_v1.yaml"
        )
    (output / "slam_configuration.json").write_text(json.dumps(report, indent=2))


def run(output, slam_params_file=None):
    output.mkdir(parents=True, exist_ok=False)
    environment = dict(
        os.environ,
        ROS_DOMAIN_ID="187",
        ROS_LOCALHOST_ONLY="1",
        GZ_PARTITION="portfolio_qualification_" + uuid.uuid4().hex,
    )
    configuration = output / "nav2_params.yaml"
    save_configuration(configuration)
    record_slam_configuration(output, slam_params_file)
    installed_world = (
        Path(get_package_share_directory("portfolio_navigation"))
        / "worlds"
        / "inspection_room.sdf"
    )
    source_world = Path(__file__).parent / "worlds" / "inspection_room.sdf"
    archive_matching_world(source_world, installed_world, output)
    with recorded_processes(output) as stack:
        start_logged(
            stack,
            launch_command(configuration, slam_params_file),
            output / "stack.log",
            environment,
        )
        start_logged(
            stack,
            [
                "ros2",
                "bag",
                "record",
                "-s",
                "sqlite3",
                "-o",
                str(output / "mission_bag"),
                "--disable-keyboard-controls",
                "--use-sim-time",
                "--topics",
                "/scan",
                "/odom",
                "/tf",
                "/tf_static",
                "/clock",
                "/cmd_vel_safe",
                "/lookahead_point",
                "/received_global_plan",
            ],
            output / "recorder.log",
            environment,
        )
        client = start_logged(
            stack,
            [
                sys.executable,
                str(Path(__file__).with_name("test_endurance.py")),
                "--output",
                str(output),
            ],
            output / "acceptance.log",
            environment,
        )
        returncode = client.wait(timeout=720)
        if returncode:
            raise RuntimeError(
                f"endurance acceptance exited {returncode}; see acceptance.log"
            )
    shutdown = json.loads((output / "shutdown.json").read_text())
    if shutdown["stack_crash_detected"]:
        raise RuntimeError("stack process crashed; inspect stack.log")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--slam-params-file",
        type=Path,
        default=None,
        help="SLAM Toolbox parameter YAML passed to the launch as slam_params_file "
        "(default: none, so the launch uses its v1 default)",
    )
    arguments = parser.parse_args()
    run(arguments.output, arguments.slam_params_file)
