"""Bounded cleanup of process groups created by this runner."""
import os
import signal
import subprocess
import time


def stop_group(process, timeout=8):
    if process is None:
        return
    # The leader may have exited while its children remain in the owned group.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait(timeout=1)
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.poll()  # Reap the leader when possible.
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f"Owned process group {process.pid} did not terminate") from error


def cleanup(actions):
    errors = []
    for name, action in actions:
        try:
            action()
        except Exception as error:
            errors.append(f"{name}: {type(error).__name__}: {error}")
    return errors
