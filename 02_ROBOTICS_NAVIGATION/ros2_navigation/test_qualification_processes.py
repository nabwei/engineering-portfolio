"""Exercise bounded shutdown with a real timed-out child, without Gazebo."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from qualification_processes import recorded_processes
from run_qualification import start_logged


class OwnedProcessEvidenceTests(unittest.TestCase):
    def test_cleanup_error_keeps_client_failure_primary_and_chains_cleanup(self):
        for client_failure in (None, TimeoutError("client deadline")):
            with (
                self.subTest(client=client_failure),
                tempfile.TemporaryDirectory() as directory,
            ):
                output = Path(directory)
                cleanup_failure = OSError("owned process cleanup failed")

                def fail_cleanup():
                    raise cleanup_failure

                primary = client_failure or cleanup_failure
                with self.assertRaises(type(primary)) as raised:
                    with recorded_processes(output) as stack:
                        stack.callback(fail_cleanup)
                        if client_failure is not None:
                            raise client_failure
                self.assertIs(raised.exception, primary)
                if client_failure is not None:
                    self.assertIs(raised.exception.__cause__, cleanup_failure)
                acceptance = json.loads((output / "acceptance.json").read_text())
                self.assertEqual(acceptance["failure"], str(primary))
                shutdown = json.loads((output / "shutdown.json").read_text())
                self.assertEqual(shutdown["owned_process_cleanup"], "FAIL")
                self.assertEqual(shutdown["cleanup_failure"], str(cleanup_failure))

    def test_non_utf8_stack_log_still_records_crash_and_original_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            raw_log = b"device output: \xff\xfe\nprocess has died\n"
            (output / "stack.log").write_bytes(raw_log)
            failure = RuntimeError("client exited 1")
            with self.assertRaises(RuntimeError) as raised:
                with recorded_processes(output):
                    raise failure
            self.assertIs(raised.exception, failure)
            shutdown = json.loads((output / "shutdown.json").read_text())
            self.assertTrue(shutdown["stack_crash_detected"])
            self.assertEqual(shutdown["status"], "FAIL")
            self.assertEqual((output / "stack.log").read_bytes(), raw_log)

    def test_client_timeout_cleans_owned_child_and_records_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.assertRaises(subprocess.TimeoutExpired):
                with recorded_processes(output) as stack:
                    child = start_logged(
                        stack,
                        [
                            sys.executable,
                            "-c",
                            "import time; print('ready', flush=True); time.sleep(30)",
                        ],
                        output / "acceptance.log",
                        dict(os.environ),
                    )
                    child.wait(timeout=0.2)
            self.assertIsNotNone(child.poll())
            with self.assertRaises(ProcessLookupError):
                os.killpg(child.pid, 0)
            shutdown = json.loads((output / "shutdown.json").read_text())
            self.assertEqual(shutdown["status"], "FAIL")
            self.assertEqual(shutdown["owned_process_cleanup"], "PASS")
            self.assertIn("timed out", shutdown["client_failure"])
            acceptance = json.loads((output / "acceptance.json").read_text())
            self.assertEqual(acceptance["status"], "FAIL")
            self.assertEqual(acceptance["failure_type"], "TimeoutExpired")
            self.assertEqual(acceptance["goals"], [])


if __name__ == "__main__":
    unittest.main()
