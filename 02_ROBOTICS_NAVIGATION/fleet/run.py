"""Start a loopback-only broker, inject producer loss/delay, persist and report."""

import argparse
import html
import json
import socket
import subprocess
import time
from pathlib import Path
import paho.mqtt.client as mqtt
from telemetry import connect_store, decode, metrics, store_sample


def wait_until(client, predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError("MQTT handshake or delivery timed out")
        client.loop(timeout=0.02)


def free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def connect_client(client, port):
    for _ in range(100):
        try:
            client.connect("127.0.0.1", port)
            return
        except ConnectionRefusedError:
            time.sleep(0.02)
    raise TimeoutError("local Mosquitto did not start")


def publish_fleet(client):
    for sequence in range(20):
        for robot in ["alpha", "beta", "gamma"]:
            if robot == "beta" and sequence % 5 == 0:
                continue
            sample = {
                "robot": robot,
                "sequence": sequence,
                "sent_ns": time.time_ns(),
                "x": sequence * 0.1,
                "y": {"alpha": 0, "beta": 1, "gamma": 2}[robot],
                "battery": 100 - sequence * 0.1,
            }
            if robot == "gamma":
                time.sleep(0.015)
            receipt = client.publish(
                f"portfolio/fleet/{robot}", json.dumps(sample), qos=1
            )
            wait_until(client, receipt.is_published)


def dashboard(output, report):
    rows = "".join(
        f"<tr><td>{html.escape(robot)}</td><td>{stats['received']}/{stats['expected']}</td><td>{stats['mean_delay_ms']:.2f}</td><td>{stats['missing_sequences']}</td></tr>"
        for robot, stats in report.items()
    )
    document = f'<!doctype html><meta charset="utf-8"><title>Fleet telemetry evidence</title><style>body{{background:#102030;color:#eef;font:18px system-ui;margin:50px}}td,th{{padding:18px;border-bottom:1px solid #456}}h1{{color:#5de0ca}}</style><h1>Fleet telemetry — local MQTT run</h1><p>Actual loopback Mosquitto delivery; indexed SQLite time series. Producer-injected loss and delay.</p><table><tr><th>Robot</th><th>Received / expected</th><th>Mean delay (ms)</th><th>Missing sequences</th></tr>{rows}</table><p>Software timestamps on one host; no hardware timing claim.</p>'
    (output / "dashboard.html").write_text(document, encoding="utf-8")


def execute(output):
    output.mkdir(parents=True, exist_ok=True)
    database_path = output / "telemetry.sqlite"
    if database_path.exists():
        raise FileExistsError(
            "use a fresh output directory to preserve earlier measurements"
        )
    database = connect_store(database_path)
    port = free_port()
    log = (output / "mosquitto.log").open("w")
    broker = subprocess.Popen(
        ["mosquitto", "-p", str(port), "-v"], stdout=log, stderr=subprocess.STDOUT
    )
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, client_id="portfolio-local-validation"
    )
    subscribed = []
    client.on_subscribe = lambda *arguments: subscribed.append(True)
    client.on_message = lambda client, userdata, message: store_sample(
        database, decode(message.payload), time.time_ns()
    )
    try:
        connect_client(client, port)
        client.subscribe("portfolio/fleet/+", qos=1)
        wait_until(client, lambda: bool(subscribed))
        publish_fleet(client)
        wait_until(
            client,
            lambda: database.execute("SELECT COUNT(*) FROM samples").fetchone()[0]
            == 56,
        )
        report = metrics(database, {robot: 20 for robot in ["alpha", "beta", "gamma"]})
        if (
            report["beta"]["missing_sequences"] != [0, 5, 10, 15]
            or report["gamma"]["mean_delay_ms"] < 15
        ):
            raise AssertionError("injected fault not detected")
        (output / "metrics.json").write_text(json.dumps(report, indent=2))
        dashboard(output, report)
        print(json.dumps({"status": "PASS", "metrics": report}))
    finally:
        client.disconnect()
        broker.terminate()
        broker.wait(timeout=5)
        database.close()
        log.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
