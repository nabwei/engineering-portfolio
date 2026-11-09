"""Frozen nested grouped development selection and observed-test comparison."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.stats import beta

from audit_duplicates import POLICIES, content_audit
from inspection import confusion, select_threshold, train
from multiscale import enhanced_features
from run_inspection import audit_summary, plot_evaluation, records, split_groups
from selection import fit, grouped_folds, load_model, save_model, select_candidate


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


def features_for(samples):
    start = time.perf_counter()
    values = np.array([enhanced_features(cv2.imread(str(sample["path"]), cv2.IMREAD_GRAYSCALE)) for sample in samples])
    return {"original": values[:, :18], "enhanced": values}, time.perf_counter()-start


def binomial_intervals(metrics):
    (_, fp), (fn, tp) = metrics["matrix_rows_true_columns_predicted"]
    def interval(success, total):
        if total == 0:
            return None
        return [0. if success == 0 else float(beta.ppf(.025, success, total-success+1)),
                1. if success == total else float(beta.ppf(.975, success+1, total-success))]
    return {"recall_nominal_95": interval(tp, tp+fn), "precision_nominal_95": interval(tp, tp+fp),
            "limitation": "Image-level exact binomial intervals assume independent Bernoulli trials; correlated item views violate that assumption."}


def nested_evaluation(features, labels, groups):
    assignments = grouped_folds(groups, labels, 4)
    predictions, scores, details = np.empty(len(labels), int), np.empty(len(labels)), []
    for fold in range(4):
        training, validation = assignments != fold, assignments == fold
        subset = {name: values[training] for name, values in features.items()}
        candidate, threshold, selection, inner_assignments = select_candidate(subset, labels[training], groups[training], 3)
        model = fit(candidate, subset, labels[training])
        scores[validation] = model.scores(features[candidate[1]][validation])
        predictions[validation] = scores[validation] >= threshold
        details.append({"outer_fold": fold, "candidate": candidate[0], "threshold": threshold,
                        "training_rows": np.flatnonzero(training).tolist(), "validation_rows": np.flatnonzero(validation).tolist(),
                        "inner_folds": inner_assignments.tolist(), "candidate_selection": selection,
                        "metrics": confusion(labels[validation], predictions[validation])})
    return {"metrics": confusion(labels, predictions), "predictions": predictions.tolist(), "scores": scores.tolist(),
            "outer_folds": assignments.tolist(), "details": details}


def passes_target(metrics):
    return metrics["recall"] >= .85 and metrics["precision"] >= .80


def baseline_evaluation(features, labels, dev_splits, test_features):
    baseline = train(features["original"][dev_splits == "train"], labels[dev_splits == "train"])
    threshold = select_threshold(labels[dev_splits == "validation"], baseline.scores(features["original"][dev_splits == "validation"]))
    scores = baseline.scores(test_features["original"])
    return {"threshold": threshold, "scores": scores, "predictions": scores >= threshold}


def write_manifests(development, dev_splits, evaluation, output):
    baseline = evaluation["baseline"]
    with (output / "test_predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["item", "group", "image", "label", "selected_score", "selected_prediction", "baseline_score", "baseline_prediction", "image_sha256"])
        writer.writerows([sample["item"], sample["group"], sample["path"].name, sample["label"], score, int(predicted), old_score, int(old_predicted), sample["sha256"]]
                         for sample, score, predicted, old_score, old_predicted in zip(evaluation["samples"], evaluation["scores"], evaluation["predictions"], baseline["scores"], baseline["predictions"]))
    write_json(output / "development_manifest.json", [{"item": sample["item"], "group": sample["group"], "image": sample["path"].name,
                "label": sample["label"], "original_split": str(split), "image_sha256": sample["sha256"]}
                for sample, split in zip(development, dev_splits)])


def build_report(frozen, nested, evaluation, timings, audit):
    test_metrics = confusion(evaluation["labels"], evaluation["predictions"])
    baseline = evaluation["baseline"]
    gates = {"nested_development": passes_target(nested["metrics"]), "observed_test_descriptive": passes_target(test_metrics),
             "mean_extraction_under_half_second": timings["mean_extraction_per_image"] < .5}
    failures = [{"image": f'{sample["item"]}/{sample["path"].name}', "truth": sample["label"], "prediction": int(prediction), "score": float(score)}
                for sample, prediction, score in zip(evaluation["samples"], evaluation["predictions"], evaluation["scores"]) if prediction != sample["label"]]
    return {"protocol_sha256": frozen["protocol_sha256"], "status": "PASS" if all(gates.values()) else "FAIL", "acceptance_gates": gates,
            "acceptance_contract": "Additional predeclared engineering target; not a user-supplied production specification or the original functional scope",
            "dataset": "KolektorSDD existing authorized local archive; image classification only",
            "duplicate_policy": audit["policy"], "content_hash_audit": audit_summary(audit),
            "test_exposure": "Previously observed 2026-10-25; descriptive comparison, not unseen independent qualification",
            "selected_candidate": frozen["candidate"], "threshold_frozen_before_test": frozen["threshold"],
            "nested_development": {"metrics": nested["metrics"], "intervals": binomial_intervals(nested["metrics"])},
            "observed_test": {"metrics": test_metrics, "intervals": binomial_intervals(test_metrics)},
            "original_baseline": {"threshold": baseline["threshold"], "metrics": confusion(evaluation["labels"], baseline["predictions"])},
            "timing_seconds": timings, "failures": failures,
            "calibration": "No geometric measurement or calibration metadata; camera calibration is not applicable."}


def development_model(features, labels, groups):
    started = time.perf_counter()
    nested = nested_evaluation(features, labels, groups)
    candidate, threshold, reports, folds = select_candidate(features, labels, groups, 5)
    model = fit(candidate, features, labels)
    return {"nested": nested, "candidate": candidate, "threshold": threshold, "reports": reports, "folds": folds,
            "model": model, "row_count": len(labels), "item_count": len(set(groups)), "seconds": time.perf_counter()-started}


def frozen_metadata(selected, model_path, development, audit):
    protocol = Path(__file__).with_name("QUALIFICATION_PROTOCOL_20261026.md")
    return {"protocol_sha256": hashlib.sha256(protocol.read_bytes()).hexdigest(), "candidate": selected["candidate"],
            "threshold": selected["threshold"], "development_rows": selected["row_count"],
            "development_split_groups": selected["item_count"],
            "development_item_folders": len({sample["item"] for sample in development}),
            "duplicate_policy": audit["policy"], "content_hash_audit": audit_summary(audit),
            "grouped_selection_folds": selected["folds"].tolist(), "candidates": selected["reports"],
            "model_file": model_path.name, "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest()}


def evaluate(dataset, output, duplicate_policy="reject"):
    cv2.setNumThreads(1)
    output.mkdir(parents=True, exist_ok=False)
    samples = records(dataset, duplicate_policy)
    audit = content_audit(samples, duplicate_policy)
    write_json(output / "duplicate_audit.json", audit)
    splits = split_groups(samples)
    development = [sample for sample, split in zip(samples, splits) if split != "test"]
    dev_splits = splits[splits != "test"]
    labels = np.array([sample["label"] for sample in development])
    # Grouped folds use the content-checked split group, so merged items never straddle a fold.
    groups = np.array([sample["group"] for sample in development])
    features, extraction_seconds = features_for(development)
    selected = development_model(features, labels, groups)
    nested, candidate, threshold, model = (selected[key] for key in ["nested", "candidate", "threshold", "model"])
    saved = save_model(model, output / "selected_model")
    frozen = frozen_metadata(selected, saved, development, audit)
    # Freeze the artifact before extracting/scoring any original-test image.
    write_json(output / "frozen_selection.json", frozen)
    write_json(output / "nested_development.json", nested)
    test = [sample for sample, split in zip(samples, splits) if split == "test"]
    test_features, test_extraction_seconds = features_for(test)
    test_labels = np.array([sample["label"] for sample in test])
    inference_start = time.perf_counter()
    test_scores = load_model(saved).scores(test_features[candidate[1]])
    inference_seconds = time.perf_counter()-inference_start
    np.testing.assert_array_equal(test_scores, model.scores(test_features[candidate[1]]))
    predictions = test_scores >= threshold
    evaluation = {"samples": test, "labels": test_labels, "scores": test_scores, "predictions": predictions,
                  "baseline": baseline_evaluation(features, labels, dev_splits, test_features)}
    write_manifests(development, dev_splits, evaluation, output)
    timings = {"development_extraction": extraction_seconds, "test_extraction": test_extraction_seconds,
               "mean_extraction_per_image": (extraction_seconds+test_extraction_seconds)/len(samples),
               "nested_selection_and_refit": selected["seconds"], "test_model_load_and_predict": inference_seconds}
    report = build_report(frozen, nested, evaluation, timings, audit)
    write_json(output / "qualification_report.json", report)
    plot_evaluation(test, test_scores, predictions, output)
    write_json(output / "prediction_signature.json", {"candidate": candidate, "threshold": threshold,
               "nested_predictions": nested["predictions"], "nested_scores": nested["scores"],
               "test_scores": test_scores.tolist(), "test_predictions": predictions.astype(int).tolist(),
               "model_sha256": frozen["model_sha256"]})
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duplicate-policy", choices=POLICIES, default="reject",
                        help="reject: refuse identical content in two item folders (default); "
                             "merge: fuse such items into one split group. See DATASET.md.")
    arguments = parser.parse_args()
    qualification = evaluate(arguments.dataset, arguments.output, arguments.duplicate_policy)
    print(json.dumps(qualification, indent=2))
    raise SystemExit(0 if qualification["status"] == "PASS" else 1)
