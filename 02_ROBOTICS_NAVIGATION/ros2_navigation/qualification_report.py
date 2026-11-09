"""Preflight model identity and persist qualification outcomes before rejection."""

import hashlib
import json
import math
import sys
from contextlib import contextmanager


def archive_matching_world(source, installed, output):
    try:
        source_bytes = source.read_bytes()
        installed_bytes = installed.read_bytes()
    except OSError as failure:
        report = {
            "status": "FAIL",
            "source_path": str(source),
            "installed_path": str(installed),
            "failure_type": type(failure).__name__,
            "failure": str(failure),
        }
        (output / "world_preflight.json").write_text(json.dumps(report, indent=2))
        raise
    source_hash = hashlib.sha256(source_bytes).hexdigest()
    installed_hash = hashlib.sha256(installed_bytes).hexdigest()
    matched = source_hash == installed_hash
    report = {
        "status": "PASS" if matched else "FAIL",
        "source_path": str(source),
        "installed_path": str(installed),
        "source_sha256": source_hash,
        "installed_sha256": installed_hash,
    }
    (output / "world_preflight.json").write_text(json.dumps(report, indent=2))
    if not matched:
        raise RuntimeError(
            "installed world differs from source; rebuild before qualification"
        )
    (output / "inspection_room.sdf").write_bytes(installed_bytes)
    return report


def in_range(value, lower, upper):
    return value is not None and math.isfinite(value) and lower <= value <= upper


ROUTE_GOALS = 10

# Three independent verdicts are written into every acceptance.json so the
# original navigation scope and the added 5 cm / 0.10 rad agreement target are
# never reported through one conflated flag. `status` is unchanged and remains
# the strict combined verdict; the other two keys only make its parts visible.
STATUS_DEFINITIONS = {
    "status": (
        "PASS only when all ten goals completed, every completed goal passed the "
        "added 0.05 m / 0.10 rad pose-agreement target, and all four mission "
        "metric checks passed; otherwise FAIL (the combined follow-up contract "
        "verdict, which raises after the report is written)."
    ),
    "original_scope_status": (
        "PASS only when all ten goals completed and all four mission metric "
        "checks (body tilt, duration, distance, clearance) passed; the per-goal "
        "endpoint limits (0.3 m estimated, 0.35 m aligned simulator truth) are "
        "asserted in the route before a goal is recorded, so every recorded goal "
        "already met them; otherwise FAIL."
    ),
    "pose_agreement_status": (
        "PASS when ten goals completed and every goal has pose_agreement_pass "
        "true; FAIL when any completed goal has pose_agreement_pass false, "
        "missing or non-boolean, or more than ten goals were supplied; NOT RUN when fewer than ten goals completed "
        "and none of them failed (including zero completed goals)."
    ),
}


def classify_pose_agreement(goals):
    # A completed goal whose pose_agreement_pass is absent or not a boolean is a
    # FAIL, not a crash: the route always writes the key before appending, so a
    # missing key means the evidence is not a verified pass.
    if not goals:
        return "NOT RUN"
    if any(
        not isinstance(goal, dict) or goal.get("pose_agreement_pass") is not True
        for goal in goals
    ):
        return "FAIL"
    if len(goals) > ROUTE_GOALS:
        return "FAIL"
    return "PASS" if len(goals) == ROUTE_GOALS else "NOT RUN"


def save_partial_failure(output, failure):
    report, corrupt_report = read_saved_evidence(output / "acceptance.json", dict)
    corrupt = [corrupt_report] if corrupt_report else []
    if report is None:
        goals, corrupt_goals = read_saved_evidence(output / "goals.json", list)
        if corrupt_goals:
            corrupt.append(corrupt_goals)
        goals = goals if goals is not None else []
        report = {
            "status": "FAIL",
            "original_scope_status": "FAIL",
            "pose_agreement_status": classify_pose_agreement(goals),
            "navigation_actions": "FAIL",
            "mission_metrics": "NOT RUN",
            "status_definitions": STATUS_DEFINITIONS,
            "goals": goals,
        }
    if corrupt:
        report.setdefault("corrupt_evidence", []).extend(corrupt)
    report["status"] = "FAIL"
    # An existing valid report keeps the statuses it already recorded; the two
    # scope keys are only filled in when an older report never wrote them.
    report.setdefault("original_scope_status", "FAIL")
    if "pose_agreement_status" not in report:
        saved_goals = report.get("goals", [])
        report["pose_agreement_status"] = (
            classify_pose_agreement(saved_goals) if isinstance(saved_goals, list) else "FAIL"
        )
    report.setdefault("status_definitions", STATUS_DEFINITIONS)
    report.setdefault("failure_type", type(failure).__name__)
    report.setdefault("failure", str(failure))
    (output / "acceptance.json").write_text(json.dumps(report, indent=2))


def read_saved_evidence(path, expected_type):
    if not path.exists():
        return None, None
    raw = path.read_bytes()
    try:
        parsed = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as failure:
        reason = f"{type(failure).__name__}: {failure}"
    else:
        if isinstance(parsed, expected_type):
            return parsed, None
        reason = f"expected JSON {expected_type.__name__}, got {type(parsed).__name__}"
    # Preserve interrupted writer bytes before replacing the report with valid JSON.
    preserved = path.with_name(
        path.name + ".corrupt-" + hashlib.sha256(raw).hexdigest()
    )
    preserved.write_bytes(raw)
    return None, {"source": path.name, "reason": reason, "preserved_as": preserved.name}


@contextmanager
def preserve_acceptance_failure(output):
    try:
        yield
    finally:
        failure = sys.exc_info()[1]
        if failure is not None:
            save_partial_failure(output, failure)


def write_and_require_acceptance(output, goals, metrics):
    nonfinite = [
        key
        for key, value in metrics.items()
        if isinstance(value, (int, float)) and not math.isfinite(value)
    ]
    metrics = {
        key: None if key in nonfinite else value for key, value in metrics.items()
    }
    checks = {
        "body_tilt": in_range(metrics["peak_body_tilt_rad"], 0, 0.2),
        "duration": in_range(metrics["simulated_seconds"], 120, math.inf),
        "distance": in_range(metrics["odometry_distance_m"], 12, math.inf),
        "clearance": in_range(
            metrics["minimum_sampled_lidar_clearance_m"], 0.22, math.inf
        ),
    }
    navigation_passed = len(goals) == ROUTE_GOALS
    pose_status = classify_pose_agreement(goals)
    pose_passed = pose_status == "PASS"
    # Original scope: ten completed goals plus the four metric checks. Endpoint
    # errors (<=0.3 m estimated, <=0.35 m aligned simulator truth) are asserted
    # per goal in test_endurance.run_route before the goal is appended, so any
    # goal present here already satisfied them.
    original_scope_passed = navigation_passed and all(checks.values())
    # Strict combined verdict, unchanged: navigation AND pose agreement AND checks.
    passed = navigation_passed and pose_passed and all(checks.values())
    report = {
        **metrics,
        "status": "PASS" if passed else "FAIL",
        "original_scope_status": "PASS" if original_scope_passed else "FAIL",
        "pose_agreement_status": pose_status,
        "navigation_actions": "PASS" if navigation_passed else "FAIL",
        "mission_metric_checks": checks,
        "nonfinite_metrics": nonfinite,
        "criteria": {"pose_position_error_m": 0.05, "pose_yaw_error_rad": 0.10},
        "status_definitions": STATUS_DEFINITIONS,
        "goals": goals,
    }
    (output / "acceptance.json").write_text(
        json.dumps(report, indent=2, allow_nan=False)
    )
    if not passed:
        raise AssertionError("qualification criteria failed; see saved acceptance.json")
    return report
