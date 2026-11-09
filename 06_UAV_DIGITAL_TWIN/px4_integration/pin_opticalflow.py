"""Pin PX4 v1.17.0's otherwise floating OpticalFlow ExternalProject dependency."""
import argparse
from pathlib import Path
import subprocess

PX4_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
OPTICALFLOW_COMMIT = "8758bd5567aadf2f8ab5deee97701d2c495d0756"


def pin(source):
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != PX4_COMMIT:
        raise ValueError(f"Unsupported PX4 source revision: {revision}")
    path = source / "src/modules/simulation/gz_plugins/optical_flow/optical_flow.cmake"
    content = path.read_text(encoding="utf-8")
    old, new = "GIT_TAG master", f"GIT_TAG {OPTICALFLOW_COMMIT}"
    if new in content and old not in content:
        return
    if content.count(old) != 1:
        raise ValueError("Unexpected upstream OpticalFlow definition; inspect before modifying")
    path.write_text(content.replace(old, new), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("px4_source", type=Path)
    args = parser.parse_args()
    pin(args.px4_source)
    print(f"PX4 {PX4_COMMIT}; OpticalFlow {OPTICALFLOW_COMMIT}")
