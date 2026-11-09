"""Real-file evidence tests; no ROS server or simulator is needed."""

import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from qualification_report import (
    archive_matching_world,
    preserve_acceptance_failure,
    write_and_require_acceptance,
)


class QualificationEvidenceTests(unittest.TestCase):
    def test_corrupt_report_preserves_raw_bytes_and_original_failure(self):
        cases = [
            ("acceptance.json", b'{"status":'),
            ("acceptance.json", b"\xff"),
            ("acceptance.json", b"[]"),
            ("goals.json", b'[{"index":'),
        ]
        for name, raw in cases:
            with (
                self.subTest(name=name, raw=raw),
                tempfile.TemporaryDirectory() as directory,
            ):
                output = Path(directory)
                (output / name).write_bytes(raw)
                if name == "acceptance.json":
                    (output / "goals.json").write_text('[{"index": 0}]')
                failure = TimeoutError("navigation timed out")
                with self.assertRaises(TimeoutError) as raised:
                    with preserve_acceptance_failure(output):
                        raise failure
                self.assertIs(raised.exception, failure)
                report = json.loads((output / "acceptance.json").read_text())
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["failure"], str(failure))
                self.assertEqual(
                    report["goals"], [{"index": 0}] if name == "acceptance.json" else []
                )
                corruption = report["corrupt_evidence"][0]
                self.assertEqual(corruption["source"], name)
                self.assertEqual(
                    (output / corruption["preserved_as"]).read_bytes(), raw
                )

    def test_missing_world_preserves_failed_preflight(self):
        for missing in ("source", "installed"):
            with (
                self.subTest(missing=missing),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                source, installed = root / "source.sdf", root / "installed.sdf"
                (installed if missing == "source" else source).write_bytes(b"world")
                with self.assertRaises(FileNotFoundError):
                    archive_matching_world(source, installed, root)
                report = json.loads((root / "world_preflight.json").read_text())
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["failure_type"], "FileNotFoundError")
                self.assertIn(missing + ".sdf", report["failure"])

    def test_route_failures_keep_partial_trace_and_propagate(self):
        cases = [
            ([], AssertionError("Nav2 rejected endurance goal")),
            ([{"index": 0, "slam_goal_error_m": 0.31}], AssertionError("endpoint")),
            ([{"index": 0}], TimeoutError("ROS integration condition timed out")),
        ]
        for goals, failure in cases:
            with (
                self.subTest(reason=str(failure)),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                (root / "goals.json").write_text(json.dumps(goals))
                with self.assertRaises(type(failure)) as raised:
                    with preserve_acceptance_failure(root):
                        raise failure
                self.assertIs(raised.exception, failure)
                report = json.loads((root / "acceptance.json").read_text())
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["goals"], goals)
                self.assertEqual(report["failure"], str(failure))

    def test_stale_install_rejected_and_matching_model_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, installed = root / "source.sdf", root / "installed.sdf"
            source.write_bytes(b"<world>current</world>")
            installed.write_bytes(b"<world>stale</world>")
            with self.assertRaisesRegex(RuntimeError, "rebuild"):
                archive_matching_world(source, installed, root)
            failed = json.loads((root / "world_preflight.json").read_text())
            self.assertEqual(failed["status"], "FAIL")
            self.assertNotEqual(failed["source_sha256"], failed["installed_sha256"])
            self.assertFalse((root / "inspection_room.sdf").exists())
            installed.write_bytes(source.read_bytes())
            report = archive_matching_world(source, installed, root)
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(
                (root / "inspection_room.sdf").read_bytes(), source.read_bytes()
            )
            self.assertEqual(
                report["source_sha256"], hashlib.sha256(source.read_bytes()).hexdigest()
            )

    def test_each_failed_metric_keeps_report_and_full_goal_trace(self):
        valid = {
            "peak_body_tilt_rad": 0.05,
            "simulated_seconds": 133.0,
            "odometry_distance_m": 13.0,
            "minimum_sampled_lidar_clearance_m": 1.0,
        }
        failures = [
            ("peak_body_tilt_rad", 0.21, "body_tilt"),
            ("simulated_seconds", 119.0, "duration"),
            ("odometry_distance_m", 11.9, "distance"),
            ("minimum_sampled_lidar_clearance_m", 0.21, "clearance"),
            ("minimum_sampled_lidar_clearance_m", None, "clearance"),
            ("peak_body_tilt_rad", math.nan, "body_tilt"),
        ]
        goals = [{"index": i, "pose_agreement_pass": True} for i in range(10)]
        for field, bad_value, check in failures:
            with (
                self.subTest(field=field, value=bad_value),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                    write_and_require_acceptance(
                        root, goals, {**valid, field: bad_value}
                    )
                report = json.loads((root / "acceptance.json").read_text())
                self.assertEqual(report["status"], "FAIL")
                self.assertFalse(report["mission_metric_checks"][check])
                if isinstance(bad_value, float) and not math.isfinite(bad_value):
                    self.assertIsNone(report[field])
                    self.assertIn(field, report["nonfinite_metrics"])
                else:
                    self.assertEqual(report[field], bad_value)
                self.assertEqual(report["goals"], goals)
        with tempfile.TemporaryDirectory() as directory:
            report = write_and_require_acceptance(Path(directory), goals, valid)
            self.assertEqual(report["status"], "PASS")
            self.assertTrue(all(report["mission_metric_checks"].values()))
        for incomplete in (
            goals[:4],
            goals[:-1] + [{"index": 9, "pose_agreement_pass": False}],
        ):
            with (
                self.subTest(goals=incomplete),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                    write_and_require_acceptance(root, incomplete, valid)
                report = json.loads((root / "acceptance.json").read_text())
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["goals"], incomplete)
                self.assertTrue(all(report["mission_metric_checks"].values()))


VALID_METRICS = {
    "peak_body_tilt_rad": 0.05,
    "simulated_seconds": 133.0,
    "odometry_distance_m": 13.0,
    "minimum_sampled_lidar_clearance_m": 1.0,
}


def passing_goals(count):
    return [{"index": i, "pose_agreement_pass": True} for i in range(count)]


def saved_report(root):
    return json.loads((root / "acceptance.json").read_text(encoding="utf-8"))


class StatusClassificationTests(unittest.TestCase):
    """Each combination is checked on the acceptance.json read back from disk."""

    def test_navigation_pass_pose_fail_reports_original_scope_pass(self):
        goals = passing_goals(9) + [{"index": 9, "pose_agreement_pass": False}]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                write_and_require_acceptance(root, goals, VALID_METRICS)
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "PASS")
            self.assertEqual(report["pose_agreement_status"], "FAIL")
            self.assertEqual(report["navigation_actions"], "PASS")
            self.assertTrue(all(report["mission_metric_checks"].values()))
            self.assertEqual(report["goals"], goals)
            self.assertNotIn("failure", report)

    def test_all_criteria_pass_reports_three_pass_statuses(self):
        goals = passing_goals(10)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            returned = write_and_require_acceptance(root, goals, VALID_METRICS)
            report = saved_report(root)
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(report["original_scope_status"], "PASS")
            self.assertEqual(report["pose_agreement_status"], "PASS")
            self.assertEqual(report, returned)
            self.assertEqual(
                set(report["status_definitions"]),
                {"status", "original_scope_status", "pose_agreement_status"},
            )
            self.assertEqual(
                report["criteria"],
                {"pose_position_error_m": 0.05, "pose_yaw_error_rad": 0.10},
            )

    def test_incomplete_route_reports_pose_agreement_not_run(self):
        goals = passing_goals(4)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                write_and_require_acceptance(root, goals, VALID_METRICS)
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "FAIL")
            self.assertEqual(report["pose_agreement_status"], "NOT RUN")
            self.assertEqual(report["navigation_actions"], "FAIL")
            self.assertTrue(all(report["mission_metric_checks"].values()))
            self.assertEqual(len(report["goals"]), 4)

    def test_stale_metric_metadata_cannot_override_computed_failure(self):
        metrics = {**VALID_METRICS, "odometry_distance_m": 11.9,
                   "status": "PASS", "original_scope_status": "PASS",
                   "pose_agreement_status": "FAIL", "navigation_actions": "FAIL",
                   "goals": [], "mission_metric_checks": {"distance": True}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(AssertionError):
                write_and_require_acceptance(root, passing_goals(10), metrics)
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "FAIL")
            self.assertEqual(report["pose_agreement_status"], "PASS")
            self.assertEqual(report["navigation_actions"], "PASS")
            self.assertFalse(report["mission_metric_checks"]["distance"])
            self.assertEqual(len(report["goals"]), 10)

    def test_failed_metric_reports_original_scope_fail_with_pose_pass(self):
        goals = passing_goals(10)
        metrics = {**VALID_METRICS, "odometry_distance_m": 11.9}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                write_and_require_acceptance(root, goals, metrics)
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "FAIL")
            self.assertEqual(report["pose_agreement_status"], "PASS")
            self.assertEqual(report["navigation_actions"], "PASS")
            self.assertFalse(report["mission_metric_checks"]["distance"])
            self.assertEqual(report["odometry_distance_m"], 11.9)

    def test_missing_pose_key_fails_pose_agreement_without_crash(self):
        cases = [
            ("missing", {"index": 5}),
            ("non_boolean", {"index": 5, "pose_agreement_pass": "yes"}),
            ("null", {"index": 5, "pose_agreement_pass": None}),
        ]
        for name, malformed in cases:
            goals = passing_goals(10)
            goals[5] = malformed
            with (
                self.subTest(case=name),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                with self.assertRaisesRegex(AssertionError, "saved acceptance"):
                    write_and_require_acceptance(root, goals, VALID_METRICS)
                report = saved_report(root)
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["original_scope_status"], "PASS")
                self.assertEqual(report["pose_agreement_status"], "FAIL")
                self.assertEqual(report["goals"][5], malformed)

    def test_fatal_failure_without_report_derives_statuses_from_goals(self):
        cases = [
            ("no_reports", None, "NOT RUN"),
            ("empty_goals", [], "NOT RUN"),
            (
                "failed_pose_goal",
                passing_goals(2) + [{"index": 2, "pose_agreement_pass": False}],
                "FAIL",
            ),
            ("partial_all_passing", passing_goals(3), "NOT RUN"),
            ("overfull_route", passing_goals(11), "FAIL"),
        ]
        for name, goals, expected_pose in cases:
            with (
                self.subTest(case=name),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                if goals is not None:
                    (root / "goals.json").write_text(json.dumps(goals))
                failure = TimeoutError("ROS integration condition timed out")
                with self.assertRaises(TimeoutError):
                    with preserve_acceptance_failure(root):
                        raise failure
                report = saved_report(root)
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["original_scope_status"], "FAIL")
                self.assertEqual(report["pose_agreement_status"], expected_pose)
                self.assertEqual(report["failure"], str(failure))
                self.assertEqual(report["failure_type"], "TimeoutError")
                self.assertEqual(report["goals"], goals or [])
                self.assertEqual(report["mission_metrics"], "NOT RUN")
                self.assertIn("pose_agreement_status", report["status_definitions"])

    def test_existing_report_statuses_survive_fatal_failure(self):
        goals = passing_goals(9) + [{"index": 9, "pose_agreement_pass": False}]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(AssertionError):
                write_and_require_acceptance(root, goals, VALID_METRICS)
            before = saved_report(root)
            failure = RuntimeError("lifecycle shutdown failed after evaluation")
            with self.assertRaises(RuntimeError):
                with preserve_acceptance_failure(root):
                    raise failure
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "PASS")
            self.assertEqual(report["pose_agreement_status"], "FAIL")
            self.assertEqual(report["failure"], str(failure))
            self.assertEqual(report["failure_type"], "RuntimeError")
            self.assertEqual(report["goals"], goals)
            self.assertEqual(
                report["mission_metric_checks"], before["mission_metric_checks"]
            )
            self.assertNotIn("failure", before)

    def test_legacy_missing_pose_status_is_derived_from_saved_goals(self):
        cases = [
            (passing_goals(9) + [{"pose_agreement_pass": False}], "FAIL"),
            (passing_goals(11), "FAIL"),
            (passing_goals(4), "NOT RUN"),
            ({"malformed": "goal collection"}, "FAIL"),
        ]
        for goals, expected in cases:
            with self.subTest(goals=goals), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root / "acceptance.json").write_text(json.dumps({"status": "PASS", "goals": goals}))
                with self.assertRaises(OSError):
                    with preserve_acceptance_failure(root):
                        raise OSError("shutdown failed")
                report = saved_report(root)
                self.assertEqual(report["status"], "FAIL")
                self.assertEqual(report["pose_agreement_status"], expected)
                self.assertEqual(report["goals"], goals)

    def test_overfull_route_fails_the_main_acceptance_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(AssertionError):
                write_and_require_acceptance(root, passing_goals(11), VALID_METRICS)
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["original_scope_status"], "FAIL")
            self.assertEqual(report["pose_agreement_status"], "FAIL")

    def test_legacy_report_without_scope_keys_gets_defaults_only_if_absent(self):
        legacy = {
            "status": "PASS",
            "pose_agreement_status": "PASS",
            "navigation_actions": "PASS",
            # Reclassification would produce NOT RUN. Preserve the existing
            # historical scope verdict while recording the new overall failure.
            "goals": passing_goals(4),
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "acceptance.json").write_text(json.dumps(legacy))
            with self.assertRaises(OSError):
                with preserve_acceptance_failure(root):
                    raise OSError("shutdown.json could not be written")
            report = saved_report(root)
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(report["pose_agreement_status"], "PASS")
            self.assertEqual(report["original_scope_status"], "FAIL")
            self.assertEqual(report["navigation_actions"], "PASS")
            self.assertEqual(report["failure_type"], "OSError")


if __name__ == "__main__":
    unittest.main()
