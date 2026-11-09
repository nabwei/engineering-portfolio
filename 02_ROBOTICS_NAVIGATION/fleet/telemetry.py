"""Validated MQTT payload storage in an indexed SQLite time-series table."""

import json
import math
import sqlite3


def connect_store(path):
    database = sqlite3.connect(path)
    database.execute(
        "CREATE TABLE IF NOT EXISTS samples (robot TEXT, sequence INTEGER, sent_ns INTEGER, received_ns INTEGER, x REAL, y REAL, battery REAL, PRIMARY KEY(robot,sequence))"
    )
    database.execute("CREATE INDEX IF NOT EXISTS sample_time ON samples(received_ns)")
    return database


def decode(payload):
    sample = json.loads(payload)
    if not isinstance(sample, dict) or set(sample) != {
        "robot",
        "sequence",
        "sent_ns",
        "x",
        "y",
        "battery",
    }:
        raise ValueError("payload fields do not match schema")
    if (
        not isinstance(sample["robot"], str)
        or not sample["robot"]
        or len(sample["robot"]) > 80
    ):
        raise ValueError("invalid robot identifier")
    for field in ["sequence", "sent_ns"]:
        if type(sample[field]) is not int or not 0 <= sample[field] <= 2**63 - 1:
            raise ValueError(f"invalid {field}")
    for field in ["x", "y", "battery"]:
        if type(sample[field]) not in (int, float) or not math.isfinite(sample[field]):
            raise ValueError(f"invalid {field}")
    if not 0 <= sample["battery"] <= 100:
        raise ValueError("battery outside [0,100]")
    return sample


def store_sample(database, sample, received_ns):
    if received_ns < sample["sent_ns"]:
        raise ValueError("receive timestamp precedes send: clock alignment invalid")
    database.execute(
        "INSERT OR IGNORE INTO samples VALUES (?,?,?,?,?,?,?)",
        (
            sample["robot"],
            sample["sequence"],
            sample["sent_ns"],
            received_ns,
            sample["x"],
            sample["y"],
            sample["battery"],
        ),
    )
    database.commit()


def metrics(database, expected_per_robot):
    robots = {}
    for robot, expected in expected_per_robot.items():
        samples = database.execute(
            "SELECT sequence, received_ns-sent_ns FROM samples WHERE robot=? ORDER BY sequence",
            (robot,),
        ).fetchall()
        delays = sorted(sample[1] / 1e6 for sample in samples)
        received = {sample[0] for sample in samples}
        robots[robot] = {
            "received": len(samples),
            "expected": expected,
            "missing_sequences": sorted(set(range(expected)) - received),
            "mean_delay_ms": sum(delays) / len(delays) if delays else None,
            "max_delay_ms": max(delays) if delays else None,
        }
    return robots
