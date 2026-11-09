"""Bounded local-only MAVLink control for a runner-owned PX4 SITL instance."""
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil


class OffboardFlight:
    def __init__(self, connection, output):
        self.connection = connection
        self.output = Path(output)
        self.position = None
        self.last_position_received = -math.inf
        self.last_position_boot_ms = None
        self.velocity = None
        self.heartbeat = None
        self.acks = {}
        self.telemetry = []
        self.status_text = []
        self.last_heartbeat_sent = 0.0
        self.started = time.monotonic()

    def receive(self):
        while message := self.connection.recv_match(blocking=False):
            kind = message.get_type()
            if kind == "LOCAL_POSITION_NED":
                if self.last_position_boot_ms is not None:
                    delta = (message.time_boot_ms - self.last_position_boot_ms) & 0xffffffff
                    if not 0 < delta < 0x80000000:
                        continue
                self.last_position_boot_ms = message.time_boot_ms
                self.position = [message.x, message.y, message.z]
                self.last_position_received = time.monotonic()
                self.velocity = [message.vx, message.vy, message.vz]
                self.telemetry.append({"time_boot_ms": message.time_boot_ms,
                    "position_ned_m": self.position, "velocity_ned_mps": self.velocity})
            elif kind == "HEARTBEAT":
                self.heartbeat = message
            elif kind == "COMMAND_ACK":
                self.acks[message.command] = message.result
            elif kind == "STATUSTEXT":
                self.status_text.append(message.text)

    def send_setpoint(self, target):
        north, east, down = target
        yaw = math.atan2(5.0 - east, -north)
        milliseconds = int((time.monotonic() - self.started) * 1000) & 0xffffffff
        # Position and yaw are active; velocity, acceleration and yaw-rate are ignored.
        mask = 8 | 16 | 32 | 64 | 128 | 256 | 2048
        self.connection.mav.set_position_target_local_ned_send(milliseconds,
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, north, east, down,
            0, 0, 0, 0, 0, 0, yaw, 0)

    def tick(self, target):
        now = time.monotonic()
        if now - self.last_heartbeat_sent >= 1:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self.last_heartbeat_sent = now
        self.send_setpoint(target)
        self.receive()
        time.sleep(0.05)

    def stream(self, target, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.tick(target)

    def command(self, command, parameters, target):
        self.acks.pop(command, None)
        padded = list(parameters) + [0] * (7 - len(parameters))
        self.connection.mav.command_long_send(self.connection.target_system,
            self.connection.target_component, command, 0, *padded)
        deadline = time.monotonic() + 5
        while command not in self.acks and time.monotonic() < deadline:
            self.tick(target)
        if self.acks.get(command) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"SITL command {command} rejected/timed out: {self.acks.get(command)}; {self.status_text[-8:]}")

    def reach(self, target, timeout=30):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            self.tick(target)
            fresh = time.monotonic() - self.last_position_received < 0.5
            close = fresh and self.position is not None and math.dist(self.position, target) < 0.18
            slow = self.velocity is not None and math.hypot(*self.velocity) < 0.2
            stable_since = (stable_since or time.monotonic()) if close and slow else None
            if stable_since is not None and time.monotonic() - stable_since > 0.4:
                return list(self.position)
        raise TimeoutError(f"SITL waypoint not reached: target={target}, measured={self.position}")

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "telemetry.json").write_text(json.dumps(self.telemetry, indent=2), encoding="utf-8")
        (self.output / "status_text.json").write_text(json.dumps(self.status_text, indent=2), encoding="utf-8")

    def fly(self, waypoints, transit_altitude, capture=None):
        # Cross above the target before descending onto its far-side first waypoint.
        takeoff = [0, 0, -transit_altitude]
        self.stream(takeoff, 5)
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1, 6, 0], takeoff)
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1], takeoff)
        reached = []
        try:
            self.reach(takeoff)
            self.reach([*waypoints[0][:2], -transit_altitude])
            for sequence, target in enumerate(waypoints):
                measured = self.reach(target)
                if capture is not None:
                    self.stream(target, 1)
                    capture(sequence)
                reached.append({"sequence": sequence, "target_ned_m": target, "measured_ned_m": measured})
                print(f"Reached SITL waypoint {sequence + 1}/{len(waypoints)}", flush=True)
            self.reach([*waypoints[-1][:2], -transit_altitude])
            self.reach(takeoff)
            self.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [], takeoff)
            deadline = time.monotonic() + 40
            while time.monotonic() < deadline:
                self.receive()
                if self.heartbeat and not (self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    return reached
                time.sleep(0.1)
            raise TimeoutError("SITL did not disarm after landing")
        finally:
            self.save()
            (self.output / "reached_waypoints.json").write_text(json.dumps(reached, indent=2), encoding="utf-8")
