from types import SimpleNamespace

import pytest
from pymavlink import mavutil

import offboard
from offboard import OffboardFlight


class Sender:
    def __init__(self):
        self.setpoints = []

    def set_position_target_local_ned_send(self, *fields):
        self.setpoints.append(fields)

    def heartbeat_send(self, *fields):
        pass

    def command_long_send(self, *fields):
        pass


def flight(tmp_path):
    connection = SimpleNamespace(mav=Sender(), target_system=8, target_component=1,
                                 recv_match=lambda **kwargs: None)
    return OffboardFlight(connection, tmp_path)


def fake_clock(monkeypatch):
    elapsed = [0.0]
    monkeypatch.setattr(offboard.time, "monotonic", lambda: elapsed[0])
    monkeypatch.setattr(offboard.time, "sleep", lambda duration: elapsed.__setitem__(0, elapsed[0] + duration))


def test_setpoint_activates_only_local_position_and_yaw(tmp_path):
    controller = flight(tmp_path)
    controller.send_setpoint([1, 6, -2])
    fields = controller.connection.mav.setpoints[-1]
    assert fields[1:4] == (8, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
    assert fields[4] == 2552
    assert fields[5:8] == (1, 6, -2)
    assert fields[-2] == pytest.approx(-3 * 3.141592653589793 / 4)


def test_stale_position_does_not_count_as_waypoint_arrival(tmp_path, monkeypatch):
    fake_clock(monkeypatch)
    controller = flight(tmp_path)
    controller.position, controller.velocity = [0, 0, -1], [0, 0, 0]
    controller.last_position_received = -1
    with pytest.raises(TimeoutError, match="not reached"):
        controller.reach([0, 0, -1], timeout=1)


def test_missing_command_acknowledgement_fails_boundedly(tmp_path, monkeypatch):
    fake_clock(monkeypatch)
    controller = flight(tmp_path)
    with pytest.raises(RuntimeError, match="rejected/timed out"):
        controller.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1], [0, 0, -1])
    assert offboard.time.monotonic() < 5.1


def test_repeated_measurement_timestamp_does_not_refresh_position_age(tmp_path, monkeypatch):
    fake_clock(monkeypatch)
    controller = flight(tmp_path)
    message = SimpleNamespace(get_type=lambda: "LOCAL_POSITION_NED", time_boot_ms=42,
                              x=0, y=0, z=-1, vx=0, vy=0, vz=0)
    for _ in range(2):
        messages = iter([message, None])
        controller.connection.recv_match = lambda **kwargs: next(messages)
        controller.receive()
        offboard.time.sleep(1)
    assert controller.last_position_received == 0
    assert len(controller.telemetry) == 1


def test_existing_output_is_rejected_before_starting_resources(tmp_path):
    from run_sitl import run
    marker = tmp_path / "prior_frame.txt"
    marker.write_text("old evidence")
    with pytest.raises(FileExistsError):
        run(tmp_path / "missing_source", tmp_path / "missing_scene", tmp_path, 1)
    assert marker.read_text() == "old evidence"
