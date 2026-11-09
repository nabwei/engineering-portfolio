import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from processes import cleanup, stop_group


@pytest.mark.skipif(os.name != "posix", reason="PX4/Gazebo runner requires Linux process groups")
def test_exited_leader_still_has_its_child_terminated():
    script = """import os, signal, time
if os.fork():
    os._exit(0)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
print(os.getpid(), flush=True)
time.sleep(30)
"""
    leader = subprocess.Popen([sys.executable, "-c", script], stdout=subprocess.PIPE,
                              text=True, start_new_session=True)
    child = int(leader.stdout.readline())
    leader.wait(timeout=2)
    try:
        stop_group(leader, timeout=0.2)
        state = Path(f"/proc/{child}/stat")
        deadline = time.monotonic() + 1
        while state.exists() and state.read_text().split()[2] != "Z" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not state.exists() or state.read_text().split()[2] == "Z"
    finally:
        stop_group(leader, timeout=0.1)
        leader.stdout.close()


def test_persistence_failure_does_not_skip_remaining_cleanup():
    visited = []

    def fail_save():
        raise OSError("disk full")

    errors = cleanup([("save", fail_save), ("simulator", lambda: visited.append("simulator")),
                      ("socket", lambda: visited.append("socket"))])
    assert visited == ["simulator", "socket"]
    assert errors == ["save: OSError: disk full"]
