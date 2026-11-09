#!/usr/bin/env bash
set -eo pipefail
if [ "$#" -ne 3 ]; then
  echo "usage: build_px4.sh PX4_SOURCE PYTHON_VENV MICRO_CDR_MIRROR" >&2
  exit 2
fi
px4_source=$(realpath "$1")
python_environment=$(realpath "$2")
microcdr_mirror=$(realpath "$3")
source /opt/ros/jazzy/setup.bash
source "$python_environment/bin/activate"
python "$(dirname "$0")/pin_opticalflow.py" "$px4_source"
export GZ_DISTRO=harmonic
# The pinned local Git mirror avoids this host's WSL GitHub DNS failure.
export GIT_CONFIG_COUNT=2
export GIT_CONFIG_KEY_0="url.file://$microcdr_mirror.insteadOf"
export GIT_CONFIG_VALUE_0="https://github.com/eProsima/Micro-CDR.git"
export GIT_CONFIG_KEY_1="protocol.file.allow"
export GIT_CONFIG_VALUE_1="always"
cmake -S "$px4_source" -B "$px4_source/build/px4_sitl_default" -G Ninja \
  -DCONFIG=px4_sitl_default -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="$python_environment/bin/python"
cmake --build "$px4_source/build/px4_sitl_default" --parallel 2
