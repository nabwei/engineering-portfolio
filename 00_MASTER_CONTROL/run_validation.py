"""Aggregate local validation with exact commands, logs and honest exit status."""
import json
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def validation_commands(destination):
    python = sys.executable
    return [
        ("G_configure", "02_ROBOTICS_NAVIGATION/robot_core", ["cmake", "-S", ".", "-B", "build", "-DCMAKE_BUILD_TYPE=Debug"], ["cmake", "g++"]),
        ("G_build", "02_ROBOTICS_NAVIGATION/robot_core", ["cmake", "--build", "build", "-j2"], ["cmake"]),
        ("G_test", "02_ROBOTICS_NAVIGATION/robot_core", ["ctest", "--test-dir", "build", "--output-on-failure"], ["ctest"]),
        ("D", "03_COMPUTER_VISION", [python, "-m", "pytest", "-q"], []),
        ("I", "04_SENSOR_FUSION", [python, "-m", "pytest", "-q"], []),
        ("J_unit", "02_ROBOTICS_NAVIGATION/arm", [python, "-m", "pytest", "-q"], []),
        ("J_mission", "02_ROBOTICS_NAVIGATION/arm", [python, "run.py", "--output", str(destination/"arm")], []),
        ("K_unit", "02_ROBOTICS_NAVIGATION/fleet", [python, "-m", "pytest", "-q"], []),
        ("K_mqtt", "02_ROBOTICS_NAVIGATION/fleet", [python, "run.py", "--output", str(destination/"fleet")], ["mosquitto"]),
        ("L_unit", "06_UAV_DIGITAL_TWIN/inspection", [python, "-m", "pytest", "-q"], []),
        ("L_scene_reconstruction", "06_UAV_DIGITAL_TWIN/px4_integration", [python, "-m", "pytest", "-q", "test_scene.py", "test_reconstruct.py", "test_geometry.py", "test_processes.py"], []),
        ("L_reduced_mission", "06_UAV_DIGITAL_TWIN/inspection", [python, "run.py", "--output", str(destination/"inspection")], []),
        ("document_encoding", ".", [python, "00_MASTER_CONTROL/check_document_encoding.py"], []),
        ("python_static", ".", [python, "-m", "ruff", "check", "03_COMPUTER_VISION", "04_SENSOR_FUSION", "02_ROBOTICS_NAVIGATION/arm", "02_ROBOTICS_NAVIGATION/fleet", "06_UAV_DIGITAL_TWIN", "00_MASTER_CONTROL/run_validation.py", "00_MASTER_CONTROL/check_document_encoding.py"], []),
    ]


def execute(check, destination):
    name, directory, command, prerequisites = check
    missing = [tool for tool in prerequisites if shutil.which(tool) is None]
    record = {"name": name, "cwd": directory, "command": command}
    if missing:
        return record | {"status": "BLOCKED", "reason": "missing tools: " + ", ".join(missing)}
    log_path = destination / (name + ".log")
    with log_path.open("w", encoding="utf-8") as log:
        try:
            completed = subprocess.run(command, cwd=ROOT/directory, stdout=log,
                                       stderr=subprocess.STDOUT, timeout=300)
        except subprocess.TimeoutExpired:
            return record | {"status": "FAIL", "reason": "300 second timeout", "log": str(log_path.relative_to(ROOT))}
    return record | {"status": "PASS" if completed.returncode == 0 else "FAIL",
                     "exit_code": completed.returncode, "log": str(log_path.relative_to(ROOT))}


def main():
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    destination = ROOT / "08_TEST_RESULTS" / "aggregate" / timestamp
    destination.mkdir(parents=True)
    checks = []
    build_failed = False
    for check in validation_commands(destination):
        if check[0] in {"G_build", "G_test"} and build_failed:
            record = {"name": check[0], "status": "BLOCKED", "reason": "prior robot build stage failed"}
        else:
            record = execute(check, destination)
        checks.append(record)
        if check[0].startswith("G_") and record["status"] != "PASS":
            build_failed = True
        print(record["name"], record["status"], flush=True)
    report = {"timestamp_utc": timestamp, "scope": "local automated checks; hardware and external review separate", "checks": checks}
    (destination/"results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if all(check["status"] == "PASS" for check in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
