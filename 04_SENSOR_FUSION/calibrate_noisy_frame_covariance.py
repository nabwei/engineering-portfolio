"""One V1-only covariance grid for the fixed beta=.5 noisy-frame model."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from calibrate_flow_covariance import load_development
from run_noisy_frame_development import evaluate

FACTORS = (1., .5, .25)


def verify_baseline(output, reference, report):
    with np.load(output / "trajectory.npz", allow_pickle=False) as observed:
        with np.load(reference / "trajectory.npz", allow_pickle=False) as expected:
            checks = {name: bool(np.array_equal(observed[name], expected[name], equal_nan=True)) for name in expected.files}
    original = json.loads((reference / "report.json").read_text(encoding="utf-8"))
    checks.update(diagnostics=report["diagnostics"] == original["diagnostics"], metrics=report["metrics"] == original["metrics"])
    (output.parent / "baseline_exact_match.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
    if not all(checks.values()):
        raise RuntimeError("noisy-frame baseline changed; calibration aborted")


def run_candidate(arguments, cache, factor):
    output = arguments.output / f"factor_{factor:g}"
    output.mkdir()
    settings = argparse.Namespace(mav0=arguments.mav0, output=output)
    sigmas = (.01*factor, .005*factor)
    report = evaluate(settings, cache, sigmas)
    if factor == 1.:
        verify_baseline(output, arguments.baseline, report)
    rows = report["diagnostics"]
    nis = [row["nis"] for row in rows if "nis" in row]
    return {"factor": factor, "sigmas": sigmas, **report["metrics"]["development"],
            "accepted": sum(row.get("accepted", False) for row in rows),
            "rejected": sum(row.get("accepted") is False for row in rows),
            "nis_mean": float(np.mean(nis)), "nis_p95": float(np.quantile(nis, .95)), "nis_max": float(max(nis))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True, help="Original four-view V1 visual observations")
    parser.add_argument("--baseline", type=Path, required=True, help="Completed fixed-noise noisy-frame V1 run")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    cache = load_development(arguments.trial)
    arguments.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), Path(__file__).parent / "run_noisy_frame_development.py", Path(__file__).parent / "vio/noisy_frame.py"]
    manifest = {"beta": .5, "factors": FACTORS, "role": "V1 DEVELOPMENT ONLY; NO EXTENSION",
                "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}}
    (arguments.output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    candidates = [run_candidate(arguments, cache, factor) for factor in FACTORS]
    passed = [row for row in candidates if row["status"] == "PASS"]
    winner = min(passed, key=lambda row: row["vio"]["ate_se3_rmse_m"]) if passed else None
    result = {"status": "PASS" if winner else "FAIL", "selected": winner, "candidates": candidates}
    (arguments.output / "selection.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if winner else 1


if __name__ == "__main__":
    raise SystemExit(main())
