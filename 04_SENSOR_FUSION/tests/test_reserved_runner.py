import hashlib
import json
from argparse import Namespace
from pathlib import Path

import pytest
import cv2
import yaml

from run_flow_evaluation import ALGORITHM_VERSION, REQUIRED_SOURCE, run_reserved, verify_frozen_source
from test_euroc import make_sequence
from vio.stereo_refinement import GATES


def write_freeze(path, **overrides):
    source = Path(__file__).resolve().parents[1]
    freeze = {"source_sha256": {name: hashlib.sha256((source/name).read_bytes()).hexdigest() for name in REQUIRED_SOURCE},
              "observation_sigmas": [.01, .005], "fusion_model": "noisy_frame_pose", "frame_noise_beta": .5,
              "algorithm_version": ALGORITHM_VERSION, "stereo_refinement": dict(GATES)}
    freeze.update(overrides)
    path.write_text(json.dumps(freeze))
    return path


@pytest.mark.parametrize("field, value, message", [
    ("stereo_refinement", {**GATES, "held_out_median_limit_px": .5}, "self-refinement gates"),
    ("algorithm_version", "v1", "algorithm version"),
    ("source_sha256", {}, "omits required"),
])
def test_freeze_must_carry_v2_version_gates_and_new_module_hash(tmp_path, field, value, message):
    # A relaxed gate, an old version label or a freeze without the new module must be refused.
    with pytest.raises(ValueError, match=message):
        verify_frozen_source(write_freeze(tmp_path/"freeze.json", **{field: value}))


@pytest.mark.parametrize("invalid_calibration", [False, True])
def test_rejected_real_input_keeps_failure_report_and_original_exception(tmp_path, invalid_calibration):
    mav0 = tmp_path/"mav0"
    make_sequence(mav0)
    if invalid_calibration:
        calibration_path = mav0/"cam0/sensor.yaml"
        calibration = yaml.safe_load(calibration_path.read_text())
        calibration["distortion_coefficients"] = [0., 0., 0.]
        calibration_path.write_text(yaml.safe_dump(calibration))
    freeze = write_freeze(tmp_path/"freeze.json")
    manifest = tmp_path/"acquisition.json"
    manifest.write_text(json.dumps({"provenance": "SYNTHETIC_TEST_FIXTURE"}))
    output = tmp_path/"rejected"
    error_type = cv2.error if invalid_calibration else ValueError
    with pytest.raises(error_type) as original:
        run_reserved(Namespace(mav0=mav0, output=output, freeze=freeze, acquisition_manifest=manifest))
    report = json.loads((output/"report.json").read_text())
    assert report["status"] == "FAIL"
    assert report["metrics"]["status"] == "NOT RUN"
    assert report["failure_type"] == error_type.__name__
    assert report["failure"] == str(original.value)
    if not invalid_calibration:
        assert report["failure"] == "moving initialization needs at least50 poses spanning at least5 seconds"
    assert report["ground_truth_states_scored"] is False
    assert (output/"experiment_manifest.json").is_file()
    assert not (output/"trajectory.npz").exists()
