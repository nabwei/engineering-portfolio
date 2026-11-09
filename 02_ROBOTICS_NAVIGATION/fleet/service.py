"""Local MQTT collector and HTTP monitoring endpoint backed by SQLite."""

import argparse
import json
import logging
import sqlite3
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import paho.mqtt.client as mqtt
from telemetry import connect_store, decode, store_sample


def snapshot(database_path):
    with sqlite3.connect(database_path) as database:
        rows = database.execute(
            "SELECT robot,COUNT(*),MAX(sequence),MAX(received_ns),AVG(received_ns-sent_ns)/1000000.0 FROM samples GROUP BY robot"
        ).fetchall()
    now = time.time_ns()
    return [
        {
            "robot": robot,
            "received": count,
            "sequence_gaps": last + 1 - count,
            "age_seconds": (now - received) / 1e9,
            "mean_delay_ms": delay,
        }
        for robot, count, last, received, delay in rows
    ]


def request_handler(database_path):
    class DashboardHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/metrics":
                content = json.dumps(snapshot(database_path)).encode()
                mime = "application/json"
            elif self.path == "/":
                content = b'<!doctype html><meta charset="utf-8"><title>Live fleet monitor</title><style>body{font:18px system-ui;margin:40px;background:#102030;color:white}pre{background:#1b3544;padding:20px}</style><h1>Live fleet telemetry</h1><p>Loopback collector; software timestamps. Metrics refresh each second.</p><pre id="metrics"></pre><script>async function refresh(){let r=await fetch("/metrics");document.getElementById("metrics").textContent=JSON.stringify(await r.json(),null,2)}setInterval(refresh,1000);refresh()</script>'
                mime = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

    return DashboardHandler


def serve(arguments):
    database = connect_store(arguments.database)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def received(client, userdata, message):
        try:
            sample = decode(message.payload)
            store_sample(database, sample, time.time_ns())
        except (ValueError, UnicodeDecodeError) as error:
            logging.warning("Rejected telemetry: %s", error)

    client.on_message = received
    client.on_connect = (
        lambda client, userdata, flags, reason, properties: client.subscribe(
            "portfolio/fleet/+", qos=1
        )
        if not reason.is_failure
        else logging.error("MQTT connection rejected: %s", reason)
    )
    client.enable_logger()
    server = ThreadingHTTPServer(
        ("127.0.0.1", arguments.http_port), request_handler(arguments.database)
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    try:
        client.connect(arguments.broker_host, arguments.broker_port)
        server_thread.start()
        client.loop_forever()
    finally:
        client.disconnect()
        if server_thread.is_alive():
            server.shutdown()
            server_thread.join(timeout=5)
        server.server_close()
        database.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--broker-host", default="127.0.0.1")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--http-port", type=int, default=8088)
    parser.add_argument("--database", type=Path, required=True)
    logging.basicConfig(level=logging.INFO)
    serve(parser.parse_args())
