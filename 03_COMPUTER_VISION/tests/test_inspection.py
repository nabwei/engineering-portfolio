import json
import shutil

import cv2
import numpy as np
import pytest

from audit_duplicates import assign_split_groups, content_audit, merged_item_map
from inspection import confusion, defect_features, preprocess, select_threshold, train
from run_inspection import case_title, records, split_groups, evaluate as baseline_evaluate
from qualify_inspection import evaluate as qualification_evaluate


def write_dataset(root, item_count, fill=lambda index: 90 + 5 * index):
    """Synthetic KolektorSDD layout: kosNN/PartM.jpg with a zero label mask, distinct bytes per image."""
    written = []
    for index in range(item_count):
        directory = root / f"kos{index:02}"
        directory.mkdir()
        for part in range(2):
            image = np.full((48, 40), fill(index) + part, np.uint8)
            assert cv2.imwrite(str(directory / f"Part{part}.jpg"), image)
            assert cv2.imwrite(str(directory / f"Part{part}_label.bmp"), np.zeros((48, 40), np.uint8))
            written.append((directory / f"Part{part}.jpg").read_bytes())
    # The fixture itself must be duplicate-free, otherwise the guard tests would be meaningless.
    assert len(set(written)) == len(written)
    return root


def test_crack_has_more_blackhat_response_than_uniform_surface():
    clean = np.full((704, 256), 180, np.uint8)
    cracked = clean.copy()
    cracked[200:500, 120:124] = 20
    assert defect_features(cracked)[8] > defect_features(clean)[8]


def test_confusion_matrix_does_not_swap_false_positives_and_negatives():
    metrics = confusion([0, 0, 0, 1, 1], [0, 1, 1, 1, 0])
    assert metrics["matrix_rows_true_columns_predicted"] == [[1, 2], [1, 1]]
    assert metrics["precision"] == pytest.approx(1/3)
    assert metrics["recall"] == .5


def test_item_views_never_cross_dataset_partitions():
    samples = [{"group": f"kos{i:02}"} for i in range(50) for _ in range(8)]
    assignments = split_groups(samples)
    for start in range(0, len(samples), 8):
        assert len(set(assignments[start:start+8])) == 1
    assert set(assignments) == {"train", "validation", "test"}
    assert np.array_equal(assignments, split_groups(samples))


def test_distinct_item_content_passes_the_hash_guard_and_keeps_folder_groups(tmp_path):
    samples = records(write_dataset(tmp_path, 12))
    assert len(samples) == 24
    assert all(sample["group"] == sample["item"] for sample in samples)
    audit = content_audit(samples, "reject")
    assert audit["identical_content_groups"] == [] and audit["cross_item_groups"] == []
    assert audit["distinct_content_count"] == 24 and audit["boundary_safe"]
    assert audit["split_group_count"] == 12


@pytest.mark.parametrize("entry", ["records", "baseline", "qualification"])
def test_identical_content_in_two_item_folders_is_rejected_before_any_split(tmp_path, entry):
    write_dataset(tmp_path, 12)
    # Byte-identical copies under two folder names: a folder-name split cannot see this.
    shutil.copyfile(tmp_path / "kos03" / "Part0.jpg", tmp_path / "kos09" / "Part0.jpg")
    with pytest.raises(ValueError, match="identical image content crosses item folders"):
        if entry == "records":
            records(tmp_path)
        else:
            runner = baseline_evaluate if entry == "baseline" else qualification_evaluate
            runner(tmp_path, tmp_path / "output")
    assert not (tmp_path / "output" / "selected_model.npz").exists()
    assert not (tmp_path / "output" / "detector.npz").exists()
    with pytest.raises(ValueError, match="kos03/Part0.jpg == kos09/Part0.jpg"):
        records(tmp_path, "reject")


def test_merge_policy_keeps_identical_content_on_one_side_of_every_split(tmp_path):
    write_dataset(tmp_path, 12)
    shutil.copyfile(tmp_path / "kos03" / "Part0.jpg", tmp_path / "kos09" / "Part0.jpg")
    samples = records(tmp_path, "merge")
    by_item = {sample["item"]: sample["group"] for sample in samples}
    assert by_item["kos03"] == by_item["kos09"] == "kos03+kos09"
    assert all(group == item for item, group in by_item.items() if item not in ("kos03", "kos09"))
    assignments = split_groups(samples)
    linked = {assignments[i] for i, sample in enumerate(samples) if sample["item"] in ("kos03", "kos09")}
    assert len(linked) == 1
    audit = content_audit(samples, "merge")
    assert audit["cross_item_groups"] == [{"sha256": samples[6]["sha256"],
                                           "images": ["kos03/Part0.jpg", "kos09/Part0.jpg"],
                                           "items": ["kos03", "kos09"]}]
    assert audit["merged_split_groups"] == ["kos03+kos09"]
    assert audit["split_group_count"] == 11 and audit["item_folder_count"] == 12 and audit["boundary_safe"]
    assert json.loads(json.dumps(audit)) == audit


def test_within_item_duplicates_are_reported_but_do_not_cross_a_boundary(tmp_path):
    write_dataset(tmp_path, 12)
    shutil.copyfile(tmp_path / "kos05" / "Part0.jpg", tmp_path / "kos05" / "Part1.jpg")
    samples = records(tmp_path)
    audit = content_audit(samples, "reject")
    assert audit["identical_content_groups"] == [{"sha256": samples[10]["sha256"],
                                                  "images": ["kos05/Part0.jpg", "kos05/Part1.jpg"],
                                                  "items": ["kos05"]}]
    assert audit["cross_item_groups"] == [] and audit["boundary_safe"]
    assert audit["distinct_content_count"] == 23


def test_merge_is_transitive_and_rejection_lists_every_crossing_group():
    entries = [{"item": "a", "image": "0.jpg", "sha256": "h1"}, {"item": "b", "image": "0.jpg", "sha256": "h1"},
               {"item": "b", "image": "1.jpg", "sha256": "h2"}, {"item": "c", "image": "0.jpg", "sha256": "h2"},
               {"item": "d", "image": "0.jpg", "sha256": "h3"}, {"item": "e", "image": "0.jpg", "sha256": "h3"},
               {"item": "f", "image": "0.jpg", "sha256": "h4"}]
    assert merged_item_map(entries) == {"a": "a+b+c", "b": "a+b+c", "c": "a+b+c", "d": "d+e", "e": "d+e", "f": "f"}
    merged = assign_split_groups([dict(entry) for entry in entries], "merge")
    assert [entry["group"] for entry in merged] == ["a+b+c", "a+b+c", "a+b+c", "a+b+c", "d+e", "d+e", "f"]
    with pytest.raises(ValueError, match=r"6 images in 3 groups"):
        assign_split_groups([dict(entry) for entry in entries], "reject")
    with pytest.raises(ValueError, match="duplicate policy"):
        assign_split_groups(entries, "ignore")


def test_montage_titles_print_integer_labels_for_boolean_predictions(tmp_path):
    sample = {"item": "kos27", "group": "kos27+kos34", "path": tmp_path / "Part0.jpg", "label": np.bool_(True)}
    title = case_title(sample, np.float64(0.99986), np.bool_(True))
    assert title == "kos27/Part0\ntrue=1 pred=1\np=1.000"
    assert "True" not in case_title(sample, 0.3, np.array([True, False])[1])


def test_model_learns_separable_features_without_nan_constant_column():
    features = np.array([[-3, 1], [-2, 1], [-1, 1], [1, 1], [2, 1], [3, 1]])
    labels = np.array([0, 0, 0, 1, 1, 1])
    scores = train(features, labels).scores(features)
    assert np.array_equal(scores >= .5, labels)
    threshold = select_threshold(labels, scores)
    assert confusion(labels, scores >= threshold)["f1"] == 1


@pytest.mark.parametrize("image", [None, np.zeros((10, 10), np.uint8), np.zeros((40, 40), float), np.zeros((40, 40, 3), np.uint8)])
def test_unreadable_or_incompatible_images_fail_explicitly(image):
    with pytest.raises(ValueError):
        preprocess(image)
