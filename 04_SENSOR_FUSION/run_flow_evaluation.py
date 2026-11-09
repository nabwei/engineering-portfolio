"""Evaluate a frozen flow frontend on a wholly reserved recorded sequence.

All truth-overlapping estimates after autonomous quiet or moving initialization belong to the
reserved segment. This runner does not designate a 400-frame development prefix.
"""

import argparse
import hashlib
import json
from pathlib import Path

from run_relative_euroc import ReplayConfiguration, replay, save_trajectory, score
from vio.flow_stereo import FlowStereoOdometry
from vio.noisy_frame import BETA, NoisyFramePoseFilter
from vio.replay_initialization import automatic_start
from vio.stereo_refinement import GATES

REQUIRED_SOURCE = {"run_flow_evaluation.py", "run_relative_euroc.py", "vio/flow_stereo.py",
                   "vio/relative.py", "vio/eskf.py", "vio/initialization.py", "vio/euroc.py",
                   "vio/geometry.py", "vio/stereo.py", "vio/timing.py",
                   "vio/replay_initialization.py", "vio/dynamic_initialization.py", "vio/noisy_frame.py",
                   "vio/stereo_refinement.py"}
ALGORITHM_VERSION = "v2_stereo_self_refinement"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_frozen_source(path):
    freeze = json.loads(path.read_text(encoding="utf-8"))
    root = Path(__file__).resolve().parent
    if not REQUIRED_SOURCE.issubset(freeze.get("source_sha256", {})):
        raise ValueError("freeze manifest omits required estimator or runner source")
    for name, expected in freeze["source_sha256"].items():
        source = (root / name).resolve()
        if not source.is_relative_to(root) or digest(source) != expected:
            raise ValueError(f"frozen source mismatch: {name}")
    if freeze.get("observation_sigmas") != [.01, .005]:
        raise ValueError("this experiment freezes observation sigmas at 0.01 m and 0.005 rad")
    if freeze.get("fusion_model") != "noisy_frame_pose" or freeze.get("frame_noise_beta") != BETA:
        raise ValueError("freeze must identify the fixed-beta noisy-frame experimental model")
    if freeze.get("algorithm_version") != ALGORITHM_VERSION or freeze.get("stereo_refinement") != GATES:
        raise ValueError("freeze must declare the v2 algorithm version and the exact stereo self-refinement gates")
    return freeze


def record_inputs(arguments, freeze):
    metadata = {"evaluation_role": "WHOLE_SEQUENCE_RESERVED_AFTER_V1_DEVELOPMENT",
                "source_freeze": freeze, "freeze_sha256": digest(arguments.freeze),
                "acquisition_manifest_sha256": digest(arguments.acquisition_manifest),
                "acquisition_manifest_path": str(arguments.acquisition_manifest.resolve())}
    files = [arguments.mav0 / sensor / name for sensor in ["cam0", "cam1", "imu0"]
             for name in ["data.csv", "sensor.yaml"]]
    files.append(arguments.mav0 / "state_groundtruth_estimate0/data.csv")
    metadata["input_metadata_sha256"] = {str(path.relative_to(arguments.mav0)): digest(path) for path in files}
    arguments.output.mkdir(parents=True, exist_ok=False)
    (arguments.output / "experiment_manifest.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def run_reserved(arguments):
    freeze = verify_frozen_source(arguments.freeze)
    record_inputs(arguments, freeze)
    settings = ReplayConfiguration(FlowStereoOdometry, tuple(freeze["observation_sigmas"]), automatic_start,
                                   NoisyFramePoseFilter, refine_stereo=True)
    try:
        report, trajectory = replay(arguments.mav0, settings=settings)
    except Exception as failure:
        # At this run boundary, preserve unexpected estimator/OpenCV failures
        # as evidence and re-raise the original exception. Never convert them
        # into successful results or catch process interrupts (BaseException).
        failed = {"status": "FAIL", "stage": "estimation_or_initialization",
                  "failure_type": type(failure).__name__, "failure": str(failure),
                  "metrics": {"status": "NOT RUN", "reason": "estimator did not complete"},
                  "ground_truth_states_scored": False, "source_freeze_sha256": digest(arguments.freeze)}
        (arguments.output/"report.json").write_text(json.dumps(failed, indent=2), encoding="utf-8")
        raise
    report.update(frontend="Subpixel flow with descriptor reacquisition and four-view refined PnP",
                  algorithm_version=ALGORITHM_VERSION,
                  evaluation_role="WHOLE_SEQUENCE_RESERVED_AFTER_V1_DEVELOPMENT",
                  candidate_status="EXPERIMENTAL_NOT_QUALITY_APPROVED",
                  development_end_ns=int(trajectory[0][0])-1,
                  metrics={"status": "NOT RUN", "reason": "scoring not yet completed"})
    save_trajectory(arguments.output, trajectory)
    destination = arguments.output / "report.json"
    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["metrics"] = score(trajectory, arguments.mav0 / "state_groundtruth_estimate0/data.csv", report["development_end_ns"])
    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))
    return 0 if report["metrics"]["reserved"]["status"] == "PASS" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--freeze", type=Path, required=True, help="Pre-evaluation source and parameter freeze JSON")
    parser.add_argument("--acquisition-manifest", type=Path, required=True)
    return run_reserved(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
