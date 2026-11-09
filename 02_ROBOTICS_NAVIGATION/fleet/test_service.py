import json
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError
import paho.mqtt.client as mqtt
from run import free_port, connect_client, wait_until


class LiveCollectorTests(unittest.TestCase):
    def test_real_mqtt_message_reaches_http_dashboard(self):
        with tempfile.TemporaryDirectory() as directory:
            broker_port, http_port = free_port(), free_port()
            broker = subprocess.Popen(
                ["mosquitto", "-p", str(broker_port)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            service = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).with_name("service.py")),
                    "--broker-port",
                    str(broker_port),
                    "--http-port",
                    str(http_port),
                    "--database",
                    str(Path(directory) / "live.sqlite"),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            publisher = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            try:
                connect_client(publisher, broker_port)
                deadline = time.monotonic() + 8
                observed = []
                while not observed:
                    if time.monotonic() > deadline:
                        self.fail("live collector did not expose a received message")
                    sample = {
                        "robot": "testbot",
                        "sequence": 0,
                        "sent_ns": time.time_ns(),
                        "x": 1,
                        "y": 2,
                        "battery": 95,
                    }
                    receipt = publisher.publish(
                        "portfolio/fleet/testbot", json.dumps(sample), qos=1
                    )
                    wait_until(publisher, receipt.is_published)
                    try:
                        with urlopen(
                            f"http://127.0.0.1:{http_port}/metrics", timeout=1
                        ) as response:
                            observed = json.load(response)
                    except URLError:
                        time.sleep(0.05)
                self.assertEqual(observed[0]["robot"], "testbot")
                self.assertEqual(observed[0]["received"], 1)
                self.assertEqual(observed[0]["sequence_gaps"], 0)
            finally:
                publisher.disconnect()
                service.terminate()
                service.wait(timeout=5)
                broker.terminate()
                broker.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
