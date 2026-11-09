"""Record failed acceptance and owned cleanup even when a client times out."""

from contextlib import ExitStack, contextmanager
import json
import subprocess
import sys

from qualification_report import save_partial_failure


def save_shutdown(output, failure, cleanup_failure):
    stack_log = output / "stack.log"
    crashed = stack_log.exists() and "process has died" in stack_log.read_text(
        encoding="utf-8", errors="replace"
    )
    report = {
        "status": "FAIL" if failure or cleanup_failure or crashed else "PASS",
        "owned_process_cleanup": "FAIL" if cleanup_failure else "PASS",
        "stack_crash_detected": crashed,
        "client_failure": str(failure) if failure else None,
        "client_failure_type": type(failure).__name__ if failure else None,
        "cleanup_failure": str(cleanup_failure) if cleanup_failure else None,
        "cleanup_failure_type": type(cleanup_failure).__name__
        if cleanup_failure
        else None,
    }
    (output / "shutdown.json").write_text(json.dumps(report, indent=2))


@contextmanager
def recorded_processes(output):
    stack = ExitStack()
    try:
        yield stack
    finally:
        failure = sys.exc_info()[1]
        try:
            stack.close()
        except (OSError, subprocess.TimeoutExpired) as cleanup_failure:
            save_shutdown(output, failure, cleanup_failure)
            save_partial_failure(output, failure or cleanup_failure)
            if failure is not None:
                raise failure.with_traceback(failure.__traceback__) from cleanup_failure
            raise
        else:
            save_shutdown(output, failure, None)
            if failure is not None:
                save_partial_failure(output, failure)
