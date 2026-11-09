"""Evaluate the separate flow frontend on already observed V1_01 development data.

The historical ORB reserved-segment failure remains unchanged. This runner never
labels any V1 frames as newly reserved and retains the original numerical gates.
"""

import argparse
import hashlib
import json
from pathlib import Path

from run_relative_euroc import ReplayConfiguration, development_boundary, replay, save_trajectory, score
from vio.flow_stereo import FlowStereoOdometry
from vio.noisy_frame import NoisyFramePoseFilter
from vio.relative import RelativePoseFilter
from vio.replay_initialization import moving_start, quiet_start


def run_development(mav0, output, frames, settings=None):
    output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).parent / "run_relative_euroc.py"]
    sources += list((Path(__file__).parent / "vio").glob("*.py"))
    hashes = {str(path.relative_to(Path(__file__).parent)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (output / "source_sha256.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    prefix_end = development_boundary(mav0) if frames else None
    settings = settings or ReplayConfiguration(FlowStereoOdometry)
    report, trajectory = replay(mav0, prefix_end, settings)
    report.update(frontend="Subpixel forward-backward LK stereo/temporal + robust refined PnP",
                  evaluation_role="ALL_V1_01_DEVELOPMENT_AFTER_ORB_RESERVED_FAILURE",
                  development_end_ns=int(trajectory[-1][0]),
                  metrics={"status": "NOT RUN", "reason": "scoring not yet completed"})
    save_trajectory(output, trajectory)
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["metrics"] = score(trajectory, mav0 / "state_groundtruth_estimate0/data.csv", report["development_end_ns"])
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))
    return 0 if report["metrics"]["development"]["status"] == "PASS" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, choices=[0, 400], default=400, help="0 means all V1 development frames")
    parser.add_argument("--initialization", choices=["quiet", "moving"], default="quiet")
    parser.add_argument("--fusion", choices=["disjoint", "noisy"], default="disjoint",
                        help="noisy is the explicitly experimental shared-frame candidate, not quality-approved")
    parser.add_argument("--stereo-refinement", choices=["off", "on"], default="off",
                        help="on selects the v2 stereo extrinsic self-refinement; off reproduces historical runs")
    arguments = parser.parse_args()
    initialize = {"quiet": quiet_start, "moving": moving_start}[arguments.initialization]
    fusion = {"disjoint": RelativePoseFilter, "noisy": NoisyFramePoseFilter}[arguments.fusion]
    settings = ReplayConfiguration(FlowStereoOdometry, initialize=initialize, filter_factory=fusion,
                                   refine_stereo=arguments.stereo_refinement == "on")
    return run_development(arguments.mav0, arguments.output, arguments.frames, settings)


if __name__ == "__main__":
    raise SystemExit(main())
