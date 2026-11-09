#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
sudo apt-get update
sudo apt-get install -y --no-install-recommends git build-essential cmake ghdl iverilog python3-venv python3-dev mosquitto linuxptp iproute2
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-linux.txt
printf 'Environment prepared. Run .venv/bin/python 00_MASTER_CONTROL/run_validation.py\n'
