#!/usr/bin/env bash
set -euo pipefail

if [[ $(id -u) != 0 ]]; then
  printf '%s\n' 'BLOCKED: an isolated network namespace requires root/CAP_NET_ADMIN.' >&2
  exit 2
fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
portfolio=$(cd -- "$script_dir/../.." && pwd)
output=${1:?Provide a fresh output directory}
namespace="portfolio-fleet-$$"
mkdir -p -- "$output"
ip netns add "$namespace"
trap 'ip netns delete "$namespace"' EXIT
ip netns exec "$namespace" ip link set lo up
ip netns exec "$namespace" tc qdisc add dev lo root netem delay 5ms loss 5%
ip netns exec "$namespace" "$portfolio/.venv/bin/python" "$script_dir/run.py" --output "$output/run"
ip netns exec "$namespace" tc -s -j qdisc show dev lo > "$output/qdisc.json"
"$portfolio/.venv/bin/python" - "$output/qdisc.json" <<'PY'
import json
import sys
from pathlib import Path
queues = json.loads(Path(sys.argv[1]).read_text())
dropped = sum(queue.get('drops', 0) for queue in queues)
if dropped <= 0:
    raise AssertionError('netem run did not demonstrate an actual dropped packet')
print(f'PASS isolated loopback netem: {dropped} packets dropped; TCP/MQTT delivery validated')
PY
