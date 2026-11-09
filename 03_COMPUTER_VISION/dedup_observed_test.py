"""Recount frozen observed-test predictions after excluding images identical to development content.

Nothing is retrained, rescored or retuned here. The script reads the predictions that a
historical run already wrote, drops every test row whose image SHA-256 also occurs on the
development side of the same run, and recounts the confusion matrices of the recorded
predictions. The result is a corrected descriptive statistic for that historical run. It is
not a fresh hold-out evaluation and must not be presented as one: the remaining test rows
were observed before and the model/threshold stay exactly as frozen.

Two evidence layouts are supported:

* ``--run DIR``: a ``qualify_inspection.py`` output with ``test_predictions.csv`` and
  ``development_manifest.json`` (selected candidate and original baseline columns).
* ``--baseline DIR``: a ``run_inspection.py`` output with ``split_manifest.csv`` and
  ``inspection_report.json`` (predictions recomputed as ``score >= recorded threshold``).

Standard library only, so it can run in isolated mode: ``python -I dedup_observed_test.py``.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def confusion_counts(rows, label_key, prediction_key):
    tn = fp = fn = tp = 0
    for row in rows:
        label, predicted = int(row[label_key]), int(row[prediction_key])
        if label not in (0, 1) or predicted not in (0, 1):
            raise ValueError("binary labels and predictions must be 0 or 1")
        tn += label == 0 and predicted == 0
        fp += label == 0 and predicted == 1
        fn += label == 1 and predicted == 0
        tp += label == 1 and predicted == 1
    count = tn + fp + fn + tp
    return {"matrix_rows_true_columns_predicted": [[tn, fp], [fn, tp]],
            "precision": tp / (tp + fp) if tp + fp else 0.0,
            "recall": tp / (tp + fn) if tp + fn else 0.0,
            "f1": 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.0,
            "accuracy": (tn + tp) / count if count else 0.0, "count": count}


def identical_groups(rows, hash_key):
    by_hash = defaultdict(list)
    for row in rows:
        by_hash[row[hash_key]].append(f'{row["item"]}/{row["image"]}')
    return [{"sha256": digest, "images": names} for digest, names in sorted(by_hash.items()) if len(names) > 1]


def partition(test_rows, development_rows, hash_key):
    development_by_hash = defaultdict(list)
    for row in development_rows:
        development_by_hash[row[hash_key]].append(f'{row["item"]}/{row["image"]}')
    excluded, retained = [], []
    for row in test_rows:
        if row[hash_key] in development_by_hash:
            excluded.append({**row, "identical_development_images": development_by_hash[row[hash_key]]})
        else:
            retained.append(row)
    return retained, excluded


def summarize(test_rows, development_rows, hash_key, predictions, source):
    if not test_rows or not development_rows:
        raise ValueError("both test and development rows are required")
    retained, excluded = partition(test_rows, development_rows, hash_key)
    if not retained:
        raise ValueError("every test row duplicates development content; nothing to recount")
    results = {}
    for name, prediction_key in predictions.items():
        results[name] = {"as_recorded": confusion_counts(test_rows, "label", prediction_key),
                         "excluding_development_duplicates": confusion_counts(retained, "label", prediction_key)}
    return {"source": source,
            "method": "Rows whose image SHA-256 occurs in the development manifest are removed; frozen "
                      "recorded predictions are recounted. No retraining, rescoring or threshold change.",
            "claim_boundary": "Corrected descriptive statistic for a previously observed test partition; "
                              "not a fresh independent hold-out and not a new qualification result.",
            "development_rows": len(development_rows), "test_rows": len(test_rows),
            "retained_rows": len(retained), "excluded_rows": len(excluded),
            "excluded": [{key: row[key] for key in ("item", "image", "label", *predictions.values(), hash_key)}
                         | {"identical_development_images": row["identical_development_images"]} for row in excluded],
            "identical_within_test": identical_groups(test_rows, hash_key),
            "identical_within_development": identical_groups(development_rows, hash_key),
            **results}


def from_qualification_run(directory):
    with (directory / "test_predictions.csv").open(encoding="utf-8", newline="") as stream:
        test_rows = list(csv.DictReader(stream))
    development_rows = json.loads((directory / "development_manifest.json").read_text(encoding="utf-8"))
    summary = summarize(test_rows, development_rows, "image_sha256",
                        {"selected": "selected_prediction", "baseline": "baseline_prediction"}, str(directory))
    report_path = directory / "qualification_report.json"
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        summary["threshold_frozen_before_test"] = report.get("threshold_frozen_before_test")
        summary["selected_candidate"] = report.get("selected_candidate")
    return summary


def from_baseline_run(directory):
    threshold = json.loads((directory / "inspection_report.json").read_text(encoding="utf-8"))["threshold_selected_on_validation"]
    with (directory / "split_manifest.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row["prediction"] = int(float(row["score"]) >= threshold)
    test_rows = [row for row in rows if row["split"] == "test"]
    development_rows = [row for row in rows if row["split"] != "test"]
    summary = summarize(test_rows, development_rows, "sha256", {"baseline": "prediction"}, str(directory))
    summary["threshold_selected_on_validation"] = threshold
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--run", type=Path, help="qualify_inspection.py output directory")
    source.add_argument("--baseline", type=Path, help="run_inspection.py output directory")
    parser.add_argument("--output", type=Path, help="write the JSON summary to this new file")
    arguments = parser.parse_args()
    summary = from_qualification_run(arguments.run) if arguments.run else from_baseline_run(arguments.baseline)
    text = json.dumps(summary, indent=2)
    if arguments.output is not None:
        with arguments.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text)


if __name__ == "__main__":
    main()
