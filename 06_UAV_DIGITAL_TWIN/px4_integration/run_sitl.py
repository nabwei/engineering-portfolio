"""Run one isolated, local PX4/Gazebo process pair and record actual flight evidence."""
import argparse
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time

from pymavlink import mavutil

from offboard import OffboardFlight
from processes import cleanup, stop_group


def run(source, scene, output, limit, camera=False):
    source, scene, output = source.resolve(), scene.resolve(), output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runtime = output / "runtime"
    runtime.mkdir(exist_ok=True)
    build = source / "build/px4_sitl_default"
    binary = build / "bin/px4"
    if not binary.is_file():
        raise FileNotFoundError(f"Build PX4 first: {binary}")
    mission = json.loads((scene / "mission.json").read_text())
    for name in ("mission.json", "blade_mesh.npz"):
        shutil.copyfile(scene / name, output / name)
    waypoints = mission["waypoints"][:limit]
    # Fixed loopback port and instance; arbitrary remote vehicles are deliberately unsupported.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 14547))
    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14547", source_system=250)
    environment = dict(os.environ)
    environment.update({
        "GZ_PARTITION": f"portfolio_px4_{os.getpid()}", "GZ_IP": "127.0.0.1",
        "HEADLESS": "1", "PX4_GZ_STANDALONE": "1", "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": "gz_x500", "PX4_GZ_WORLD": "portfolio_inspection",
        "PX4_GZ_MODELS": str(source / "Tools/simulation/gz/models"),
        "GZ_SIM_SERVER_CONFIG_PATH": str(scene / "server.config"),
        "GZ_SIM_RESOURCE_PATH": str(source / "Tools/simulation/gz/models") + ":" + str(scene),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": str(build / "src/modules/simulation/gz_plugins") + ":" + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    })
    if camera:
        environment.update(PX4_SIM_MODEL="gz_portfolio_x500", PX4_GZ_MODELS=str(scene / "models"))
    # Transport subscriptions must share this runner's private Gazebo partition.
    os.environ["GZ_PARTITION"] = environment["GZ_PARTITION"]
    os.environ["GZ_IP"] = "127.0.0.1"
    recorder = None
    flight = OffboardFlight(connection, output)
    simulator = autopilot = None
    started = time.monotonic()
    report = {"status": "FAIL", "kind": "PX4 SITL + Gazebo software simulation",
              "requested_waypoints": len(waypoints), "complete_route": limit >= len(mission["waypoints"]),
              "hardware_validated": False}
    try:
        if camera:
            from camera import DepthRecorder
            from resolve_camera import resolve
            resolve(source, scene, output)
            recorder = DepthRecorder(output)
        with (output / "gazebo.log").open("w") as gzlog, (output / "px4.log").open("w") as pxlog:
            simulator = subprocess.Popen(["gz", "sim", "-r", "-s", "--headless-rendering", str(scene / "portfolio_inspection.sdf")],
                stdout=gzlog, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
            autopilot = subprocess.Popen([str(binary), "-d", "-i", "7", "-w", str(runtime), str(build / "etc")],
                stdout=pxlog, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if simulator.poll() is not None or autopilot.poll() is not None:
                    raise RuntimeError("Simulator/autopilot exited before initialization; inspect saved logs")
                flight.receive()
                if flight.heartbeat and flight.position is not None:
                    break
                time.sleep(0.1)
            else:
                raise TimeoutError("No local PX4 heartbeat and position within 90 seconds")
            connection.target_system = flight.heartbeat.get_srcSystem()
            connection.target_component = flight.heartbeat.get_srcComponent()
            if connection.target_system != 8:
                raise RuntimeError(f"Unexpected system ID for owned instance 7: {connection.target_system}")
            reached = flight.fly(waypoints, mission["transit_altitude_m"], recorder.snapshot if recorder else None)
            report.update(status="PASS", reached_waypoints=len(reached), landed_disarmed=True)
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        original_error = sys.exc_info()[0] is not None
        errors = cleanup([
            ("save telemetry", flight.save),
            ("stop autopilot", lambda: stop_group(autopilot)),
            ("stop simulator", lambda: stop_group(simulator)),
            ("close camera", lambda: recorder.close() if recorder else None),
            ("close MAVLink", connection.close),
        ])
        report["wall_seconds"] = time.monotonic() - started
        report["telemetry_samples"] = len(flight.telemetry)
        if errors:
            report.update(status="FAIL", cleanup_errors=errors)
        errors.extend(cleanup([("save result", lambda: (output / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8"))]))
        if errors and not original_error:
            raise RuntimeError("; ".join(errors))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--waypoints", type=int, default=64, choices=range(1, 65))
    parser.add_argument("--camera", action="store_true", help="Capture actual Gazebo depth at each waypoint")
    args = parser.parse_args()
    run(args.px4_source, args.scene, args.output, args.waypoints, args.camera)
