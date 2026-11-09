"""Launch and test one isolated ROS/Gazebo stack, preserving process logs."""

import argparse
import os
import signal
import subprocess
import sys
import time
import json
import uuid
from pathlib import Path
from configure_navigation import save_configuration


def stop_group(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=5)


def acceptance_run(command, environment, output):
    try:
        completed = subprocess.run(
            command, env=environment, capture_output=True, text=True, timeout=220
        )
    except subprocess.TimeoutExpired as error:
        captured = (
            b"FAIL: acceptance timed out\n"
            + (error.stdout or b"")
            + (error.stderr or b"")
        )
        (output / "acceptance.log").write_text(captured.decode(errors="replace"))
        raise
    (output / "acceptance.log").write_text(completed.stdout + completed.stderr)
    print(completed.stdout + completed.stderr)
    if completed.returncode:
        raise SystemExit(completed.returncode)


def execute(arguments):
    arguments.output.mkdir(parents=True, exist_ok=True)
    environment = dict(
        os.environ,
        ROS_DOMAIN_ID="187",
        ROS_LOCALHOST_ONLY="1",
        GZ_PARTITION="portfolio_" + uuid.uuid4().hex,
    )
    filename = (
        "gazebo_navigation.launch.py"
        if arguments.backend == "gazebo"
        else "navigation.launch.py"
    )
    configuration = arguments.output / "nav2_params.yaml"
    save_configuration(configuration)
    with (arguments.output / "stack.log").open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "portfolio_navigation",
                filename,
                "params_file:=" + str(configuration.resolve()),
            ],
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            command = [
                sys.executable,
                str(Path(__file__).with_name("test_navigation.py")),
                "--output",
                str(arguments.output / "acceptance.json"),
                "--backend",
                arguments.backend,
            ]
            acceptance_run(command, environment, arguments.output)
        finally:
            stop_group(process)
    crashed = "process has died" in (arguments.output / "stack.log").read_text()
    (arguments.output / "shutdown.json").write_text(
        json.dumps({"status": "FAIL" if crashed else "PASS"})
    )
    if crashed:
        raise RuntimeError("a stack process crashed; inspect stack.log")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=["core", "gazebo"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args())
