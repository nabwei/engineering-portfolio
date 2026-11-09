"""Independent synthetic fault scenarios; reference positions never enter recovery."""

import argparse
import json
from pathlib import Path

import numpy as np

from vio.eskf import ErrorStateFilter, State
from vio.recovery import RecoverySupervisor


def scenario(acceleration_fault, seed, recovery_enabled):
    random = np.random.default_rng(seed)
    estimator = ErrorStateFilter(State(velocity=np.array([.4, .1, 0.])))
    recovery = RecoverySupervisor() if recovery_enabled else None
    covariance = np.diag([.05**2]*3+[.02**2]*3)
    errors, visual_errors = [], []
    accepted = 0
    for frame in range(121):
        time = frame*.05
        if frame:
            for step in range(5):
                midpoint = time-.05+(step+.5)*.01
                acceleration = [acceleration_fault if midpoint >= 2 else 0., -.05*np.sin(.5*midpoint), 9.80665]
                estimator.predict(acceleration, [0., 0., 0.], .01)
        # Independent generative trajectory and correlated visual drift model.
        truth = np.array([.4*time, .2*np.sin(.5*time), 0.])
        observed = truth+np.array([0., .04*time, 0.])+random.normal(0., .002, 3)
        if recovery is None:
            accepted += int(estimator.update_pose(observed, np.eye(3), covariance))
        else:
            accepted += int(recovery.update(estimator, frame*50_000_000, observed, np.eye(3), covariance) == "ACCEPTED")
        # Truth is used only here for the independent scoring step.
        errors.append(float(np.linalg.norm(estimator.state.position-truth)))
        visual_errors.append(float(np.linalg.norm(observed-truth)))
    return {"raw_rmse_m": float(np.sqrt(np.mean(np.square(errors)))),
            "maximum_error_m": max(errors), "final_error_m": errors[-1],
            "visual_raw_rmse_m": float(np.sqrt(np.mean(np.square(visual_errors)))),
            "pose_updates_accepted": accepted, "recovery_events": [] if recovery is None else recovery.events,
            "position_errors_m": errors}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    results = []
    for fault, seed in [(1., 37), (2., 83), (3., 101)]:
        baseline, bounded = scenario(fault, seed, False), scenario(fault, seed, True)
        results.append({"accelerometer_step_fault_m_s2": fault, "seed": seed,
                        "baseline": baseline, "bounded_recovery": bounded,
                        "status": "PASS" if bounded["raw_rmse_m"] <= baseline["raw_rmse_m"] else "FAIL"})
    report = {"provenance": "SYNTHETIC_FAULT_SCENARIOS", "duration_s": 6,
              "fault_onset_s": 2, "visual_drift_m_s": .04, "scenarios": results,
              "criterion": "fixed-policy recovery does not worsen raw RMSE in these independently generated faults; not a universal accuracy bound"}
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    for result in results:
        print(result["accelerometer_step_fault_m_s2"], result["status"], result["baseline"]["raw_rmse_m"], result["bounded_recovery"]["raw_rmse_m"])
    if any(result["status"] != "PASS" for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
