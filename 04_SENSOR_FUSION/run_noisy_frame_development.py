"""One fixed V1 development experiment with adjacent noisy-image-pose updates."""

import argparse
import hashlib
import json
from functools import partial
from pathlib import Path

import numpy as np

from calibrate_flow_covariance import diagnostic, initial_filter, load_development
from run_relative_euroc import pose_matrix, save_trajectory, score
from vio.euroc import integrate_interval
from vio.noisy_frame import BETA, NoisyFramePoseFilter


def cached_adjacent_replay(mav0, cache, observation_sigmas=(.01, .005)):
    original, arrays = cache
    covariance = np.diag(np.repeat(np.square(observation_sigmas), 3))
    factory = partial(NoisyFramePoseFilter, pair_covariance=covariance)
    estimator, imu = initial_filter(mav0, int(arrays["timestamps"][0]), factory)
    previous_stamp, previous_image = int(arrays["timestamps"][0]), None
    trajectory, diagnostics = [], []
    for stamp, visual, old in zip(arrays["timestamps"], arrays["visual"], original["diagnostics"]):
        stamp = int(stamp)
        if stamp > previous_stamp:
            integrate_interval(estimator, (previous_stamp, stamp), imu)
        previous_stamp = stamp
        if not np.isfinite(visual).all():
            trajectory.append((stamp, pose_matrix(estimator.state), None))
            diagnostics.append(dict(old))
            continue
        update = update_image(estimator, previous_image, visual, covariance)
        previous_image = visual
        diagnostics.append(diagnostic(estimator, old, update))
        trajectory.append((stamp, pose_matrix(estimator.state), visual))
    return trajectory, diagnostics


def update_image(estimator, previous, current, covariance):
    if previous is None:
        estimator.clone_current_pose()
        return {}
    relative = np.linalg.inv(previous) @ current
    accepted = estimator.update_relative(relative[:3, 3], relative[:3, :3], covariance)
    return {"accepted": accepted, "nis": estimator.last_nis}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    cache = load_development(arguments.trial)
    arguments.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).parent / "calibrate_flow_covariance.py"]
    sources += list((Path(__file__).parent / "vio").glob("*.py"))
    manifest = {"beta": BETA, "pair_sigmas": [.01, .005], "role": "V1 DEVELOPMENT ONLY",
                "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
                "input_sha256": hashlib.sha256((arguments.trial / "trajectory.npz").read_bytes()).hexdigest()}
    (arguments.output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    report = evaluate(arguments, cache)
    return 0 if report["metrics"]["development"]["status"] == "PASS" else 1


def evaluate(arguments, cache, observation_sigmas=(.01, .005)):
    trajectory, diagnostics = cached_adjacent_replay(arguments.mav0, cache, observation_sigmas)
    report = dict(cache[0], diagnostics=diagnostics, image_pair_policy="every successful adjacent pair with explicit noisy-image-pose clone",
                  frame_noise_beta=BETA, covariance_model="iid image-pose approximation plus independent pair residual",
                  visual_translation_sigma_m=observation_sigmas[0], visual_rotation_sigma_rad=observation_sigmas[1],
                  metrics={"status": "NOT RUN", "reason": "scoring not yet completed"})
    save_trajectory(arguments.output, trajectory)
    path = arguments.output / "report.json"
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["metrics"] = score(trajectory, arguments.mav0 / "state_groundtruth_estimate0/data.csv", report["development_end_ns"])
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))
    return report


if __name__ == "__main__":
    raise SystemExit(main())
