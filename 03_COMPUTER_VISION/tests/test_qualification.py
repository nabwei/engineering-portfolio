import csv
import cv2
import hashlib
import json
import numpy as np
import pytest

from dedup_observed_test import confusion_counts, from_baseline_run, from_qualification_run
from multiscale import enhanced_features
from inspect_image import inspection_decision
from inspection import Detector, confusion
from run_inspection import records
from selection import CANDIDATES, fit, grouped_folds, load_model, save_model, select_candidate, threshold_selection


def test_native_horizontal_crack_at_image_edge_survives_feature_extraction():
    clean = np.full((900, 400), 180, np.uint8)
    crack = clean.copy()
    crack[301:303, :250] = 20
    reference, observed = enhanced_features(clean), enhanced_features(crack)
    assert np.isfinite(observed).all() and observed.shape == (158,)
    # The first vertical-kernel row concentration responds to a known thin edge crack.
    assert observed[24] > reference[24]+.3


def fixture():
    groups = np.repeat(np.array([f"item{i:02}" for i in range(24)]), 4)
    labels = np.zeros(len(groups), int)
    labels[np.arange(0, 48, 4)] = 1
    rng = np.random.default_rng(54)
    values = np.column_stack([labels*5+rng.normal(0, .1, len(labels)), rng.normal(size=len(labels)), np.ones(len(labels))])
    return {"original": values, "enhanced": values}, labels, groups


def test_group_stratification_keeps_items_intact_and_distributes_normal_items():
    _, labels, groups = fixture()
    folds = grouped_folds(groups, labels, 4)
    np.testing.assert_array_equal(folds, grouped_folds(groups, labels, 4))
    for item in set(groups):
        assert len(set(folds[groups == item])) == 1
    assert np.bincount(folds).max()-np.bincount(folds).min() <= 4
    for index in range(4):
        assert set(labels[folds == index]) == {0, 1}


def test_precision_constraint_precedes_recall_and_uses_score_boundaries():
    # Lower threshold would recover both positives but precision would be only 2/3.
    report = threshold_selection(np.array([1, 0, 1, 0]), np.array([.9, .8, .7, .1]))
    assert report["threshold"] == .9
    assert report["precision_constraint_met"]
    assert report["metrics"]["matrix_rows_true_columns_predicted"] == [[2, 0], [1, 1]]


@pytest.mark.parametrize("candidate", [CANDIDATES[0], CANDIDATES[-1]])
def test_models_learn_binary_separation_and_roundtrip_without_score_changes(candidate, tmp_path):
    features, labels, _ = fixture()
    model = fit(candidate, features, labels)
    scores = model.scores(features[candidate[1]])
    assert np.array_equal(scores >= .5, labels)
    path = save_model(model, tmp_path / "model")
    np.testing.assert_array_equal(scores, load_model(path).scores(features[candidate[1]]))


def test_development_only_selection_is_deterministic_with_grouped_oof_predictions():
    features, labels, groups = fixture()
    first = select_candidate(features, labels, groups, 3)
    second = select_candidate(features, labels, groups, 3)
    assert first[:3] == second[:3]
    np.testing.assert_array_equal(first[3], second[3])
    assert len(first[2]) == 6
    assert all(len(candidate["oof_scores"]) == len(labels) for candidate in first[2])


def test_stratification_fails_if_a_validation_fold_cannot_contain_defects():
    with pytest.raises(ValueError, match="both classes"):
        grouped_folds(np.array(["a", "b", "c", "d"]), np.array([1, 0, 0, 0]), 3)


def test_missing_annotation_cannot_silently_become_a_normal_training_label(tmp_path):
    item_directory = tmp_path / "kos00"
    item_directory.mkdir()
    assert cv2.imwrite(str(item_directory / "Part0.jpg"), np.full((40, 40), 150, np.uint8))
    with pytest.raises(ValueError, match="annotation"):
        records(tmp_path)


@pytest.mark.parametrize("invalid", [None, "modified_model", "wrong_feature_family", "unknown_family",
                                      "nan_threshold", "infinite_threshold", "below_zero_threshold",
                                      "above_one_threshold", "bool_threshold", "zero_threshold", "one_threshold",
                                      "missing_model_file", "empty_model_file", "numeric_model_file",
                                      "path_model_file", "parent_model_file"])
def test_single_image_inference_verifies_frozen_model_and_feature_contract(tmp_path, invalid):
    model = Detector(np.zeros(18), np.ones(18), np.zeros(19))
    path = save_model(model, tmp_path / "model")
    configuration = {"model_file": path.name, "model_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                     "candidate": ["fixture", "original", "logistic", .05], "threshold": .5}
    if invalid == "modified_model":
        # NPZ readers tolerate trailing bytes, so hashing must detect this change.
        with path.open("ab") as stream:
            stream.write(b"unexpected alteration")
    elif invalid == "wrong_feature_family":
        configuration["candidate"][1] = "enhanced"
    elif invalid == "unknown_family":
        configuration["candidate"][1] = "unknown"
    elif invalid == "missing_model_file":
        del configuration["model_file"]
    elif invalid == "empty_model_file":
        configuration["model_file"] = ""
    elif invalid == "numeric_model_file":
        configuration["model_file"] = 5
    elif invalid == "path_model_file":
        configuration["model_file"] = f"../{tmp_path.name}/{path.name}"
    elif invalid == "parent_model_file":
        configuration["model_file"] = ".."
    thresholds = {"nan_threshold": float("nan"), "infinite_threshold": float("inf"),
                  "below_zero_threshold": -.01, "above_one_threshold": 1.01,
                  "bool_threshold": True, "zero_threshold": 0., "one_threshold": 1.}
    if invalid in thresholds:
        configuration["threshold"] = thresholds[invalid]
    (tmp_path / "frozen_selection.json").write_text(json.dumps(configuration))
    image_path = tmp_path / "surface.png"
    assert cv2.imwrite(str(image_path), np.full((120, 80), 150, np.uint8))
    if invalid not in (None, "zero_threshold", "one_threshold"):
        messages = {"modified_model": "SHA-256", "wrong_feature_family": "feature dimension",
                    "unknown_family": "feature family"}
        message = "model file name" if invalid.endswith("model_file") else messages.get(invalid, "threshold")
        with pytest.raises(ValueError, match=message):
            inspection_decision(tmp_path, image_path)
    else:
        result = inspection_decision(tmp_path, image_path)
        assert result["score"] == .5
        assert result["prediction"] == ("normal" if invalid == "one_threshold" else "defect")


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def write_qualification_evidence(directory):
    # Six test rows: two of them (itemT1/Part0 and itemT3/Part1) carry development content.
    test_rows = [("itemT1", "Part0.jpg", 1, .99, 1, .73, 1, digest("dup-defect")),
                 ("itemT1", "Part1.jpg", 0, .05, 0, .30, 0, digest("t1-1")),
                 ("itemT2", "Part0.jpg", 1, .40, 0, .20, 0, digest("t2-0")),
                 ("itemT2", "Part1.jpg", 0, .70, 1, .10, 0, digest("t2-1")),
                 ("itemT3", "Part0.jpg", 1, .90, 1, .90, 1, digest("t3-0")),
                 ("itemT3", "Part1.jpg", 0, .01, 0, .80, 1, digest("dup-normal"))]
    with (directory / "test_predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["item", "image", "label", "selected_score", "selected_prediction",
                         "baseline_score", "baseline_prediction", "image_sha256"])
        writer.writerows(test_rows)
    development = [{"item": "itemD1", "image": "Part0.jpg", "label": 1, "original_split": "validation", "image_sha256": digest("dup-defect")},
                   {"item": "itemD1", "image": "Part1.jpg", "label": 0, "original_split": "validation", "image_sha256": digest("dup-normal")},
                   {"item": "itemD2", "image": "Part0.jpg", "label": 0, "original_split": "train", "image_sha256": digest("d2-0")},
                   {"item": "itemD2", "image": "Part1.jpg", "label": 0, "original_split": "train", "image_sha256": digest("d2-0")}]
    (directory / "development_manifest.json").write_text(json.dumps(development), encoding="utf-8")
    (directory / "qualification_report.json").write_text(json.dumps(
        {"threshold_frozen_before_test": .5867, "selected_candidate": ["fixture", "enhanced", "logistic", .05]}), encoding="utf-8")


def test_dedup_recount_excludes_only_rows_identical_to_development_and_keeps_frozen_predictions(tmp_path):
    write_qualification_evidence(tmp_path)
    summary = from_qualification_run(tmp_path)
    assert (summary["test_rows"], summary["development_rows"], summary["excluded_rows"], summary["retained_rows"]) == (6, 4, 2, 4)
    assert [(row["item"], row["image"], row["identical_development_images"]) for row in summary["excluded"]] == [
        ("itemT1", "Part0.jpg", ["itemD1/Part0.jpg"]), ("itemT3", "Part1.jpg", ["itemD1/Part1.jpg"])]
    # As recorded: selected TN=2 (T1/1, T3/1), FP=1 (T2/1), FN=1 (T2/0), TP=2 (T1/0, T3/0).
    assert summary["selected"]["as_recorded"]["matrix_rows_true_columns_predicted"] == [[2, 1], [1, 2]]
    # Without the two development-identical rows: TN=1, FP=1, FN=1, TP=1; recall drops from 2/3 to 1/2.
    assert summary["selected"]["excluding_development_duplicates"]["matrix_rows_true_columns_predicted"] == [[1, 1], [1, 1]]
    assert summary["selected"]["excluding_development_duplicates"]["recall"] == .5
    assert summary["baseline"]["as_recorded"]["matrix_rows_true_columns_predicted"] == [[2, 1], [1, 2]]
    assert summary["baseline"]["excluding_development_duplicates"]["matrix_rows_true_columns_predicted"] == [[2, 0], [1, 1]]
    assert summary["identical_within_test"] == []
    assert summary["identical_within_development"] == [{"sha256": digest("d2-0"), "images": ["itemD2/Part0.jpg", "itemD2/Part1.jpg"]}]
    assert summary["threshold_frozen_before_test"] == .5867
    assert "not a fresh independent hold-out" in summary["claim_boundary"]
    assert json.loads(json.dumps(summary)) == summary


def test_dedup_counts_agree_with_the_project_confusion_definition():
    rows = [{"label": label, "prediction": prediction} for label, prediction in zip([0, 0, 0, 1, 1, 1, 1], [0, 1, 1, 1, 0, 1, 1])]
    counted = confusion_counts(rows, "label", "prediction")
    reference = confusion([row["label"] for row in rows], [row["prediction"] for row in rows])
    assert counted == reference
    with pytest.raises(ValueError, match="binary"):
        confusion_counts([{"label": 2, "prediction": 0}], "label", "prediction")


def test_dedup_baseline_mode_recomputes_predictions_from_the_recorded_threshold(tmp_path):
    threshold = .7303149860036773
    rows = [("kosA", "Part0.jpg", "validation", 1, repr(threshold), digest("tie")),
            ("kosA", "Part1.jpg", "validation", 0, "0.2", digest("a1")),
            ("kosB", "Part0.jpg", "train", 0, "0.1", digest("b0")),
            ("kosC", "Part0.jpg", "test", 1, repr(threshold), digest("tie")),
            ("kosC", "Part1.jpg", "test", 0, "0.9", digest("c1")),
            ("kosD", "Part0.jpg", "test", 1, "0.4", digest("d0"))]
    with (tmp_path / "split_manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["item", "image", "split", "label", "score", "sha256"])
        writer.writerows(rows)
    (tmp_path / "inspection_report.json").write_text(json.dumps({"threshold_selected_on_validation": threshold}), encoding="utf-8")
    summary = from_baseline_run(tmp_path)
    # kosC/Part0 ties the validation-selected threshold exactly because it is the same image bytes.
    assert summary["baseline"]["as_recorded"]["matrix_rows_true_columns_predicted"] == [[0, 1], [1, 1]]
    assert summary["excluded_rows"] == 1 and summary["excluded"][0]["image"] == "Part0.jpg"
    assert summary["baseline"]["excluding_development_duplicates"]["matrix_rows_true_columns_predicted"] == [[0, 1], [1, 0]]
    assert summary["threshold_selected_on_validation"] == threshold
    with pytest.raises(ValueError, match="nothing to recount"):
        write_rows = [row for row in rows if row[2] != "test" or row[5] == digest("tie")]
        with (tmp_path / "split_manifest.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["item", "image", "split", "label", "score", "sha256"])
            writer.writerows(write_rows)
        from_baseline_run(tmp_path)
