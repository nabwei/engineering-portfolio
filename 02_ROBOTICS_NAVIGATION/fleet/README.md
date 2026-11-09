# K — Fleet telemetry with local MQTT

Three simulated producers publish JSON through an actual loopback Mosquitto broker using QoS 1. A Paho subscriber validates payloads and stores them in a SQLite table keyed by `(robot, sequence)` and indexed by receive timestamp. SQLite is used as a small time-series store; this does not claim InfluxDB/Timescale deployment or production-scale performance.

```sh
../../.venv/bin/python -m unittest -v test_fleet
../../.venv/bin/python run.py --output ../../08_TEST_RESULTS/robotics/fleet
```

Run under Ubuntu/WSL with Mosquitto available on PATH, Python Paho MQTT 2.1.0, and a fresh output directory. The runner starts a local broker on an ephemeral loopback port and terminates it in `finally`. It does not install a permanent service or use a public broker. A preexisting database is rejected to preserve previous evidence.

The test intentionally omits beta sequences 0,5,10,15 and delays gamma messages by 15 ms before publication. These are **producer-injected loss and delay**, not an operating-system network fault emulator. Actual MQTT delivery and database persistence are exercised. Delay measures one-host wall-clock software timestamps; clock reversal is rejected. No PTP or hardware timestamp claim is made.

Outputs: `telemetry.sqlite`, broker log, metrics JSON with received counts/sequence gaps/mean and maximum delay, and `dashboard.html` showing that run. The dashboard is a saved monitoring snapshot, not a continuously updating web service. Unit tests exercise duplicate/out-of-order messages, gap detection, invalid numeric payloads, and invalid time ordering.

For continuous collection with an already-running local broker, run:

```sh
../../.venv/bin/python service.py --database live.sqlite --broker-port 1883 --http-port 8088
```

Open `http://127.0.0.1:8088/` for a live dashboard, or `/metrics` for JSON. HTTP binds only to loopback. The collector logs malformed messages and broker events, stores accepted messages, and reports per-robot message counts, gaps, age and mean software delay. Sequence numbers must start at zero and continue monotonically across producer restarts; choose a new robot/session identifier when restarting the sequence. The database parent directory must exist. This local development service has no authentication and is not configured for external exposure.

`../../.venv/bin/python -m unittest -v test_fleet test_service` verifies the real MQTT-to-SQLite-to-HTTP path with temporary broker/service processes and checks duplicate suppression. The service is separate from the bounded demonstration runner so a real producer can continuously publish without regenerating the synthetic test sequence.

Reference: [Eclipse Paho Python client documentation](https://eclipse.dev/paho/clients/python/docs/).

## Actual packet fault injection

`test_netem.sh` creates a task-owned Linux network namespace, enables only its isolated loopback, and applies `tc netem` with 5 ms delay and 5% packet loss there. Broker and clients run inside that namespace; the host loopback and other interfaces are untouched. A trap removes the namespace after execution. Run as root with a fresh output directory:

```sh
bash test_netem.sh ../../08_TEST_RESULTS/robotics/fleet_netem
```

The script saves `qdisc.json`, requires actual dropped packets in the kernel counters, and reruns the bounded MQTT delivery test. TCP/MQTT recovery is expected to preserve all intended messages despite packet drops; the separate producer omissions remain detectable. This tests network packet loss and delay in software, not radio-link or hardware behavior. The recorded run dropped 32 packets and passed; exact loss counts/latency are stochastic. Evidence is in `../../08_TEST_RESULTS/robotics/k_netem.log` and `fleet_netem/`.
