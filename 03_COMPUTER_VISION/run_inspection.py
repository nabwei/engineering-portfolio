"""KolektorSDD training, validation and held-out evaluation with a content-hash split guard."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from audit_duplicates import POLICIES, assign_split_groups, content_audit
from inspection import confusion, defect_features, preprocess, select_threshold, train


def records(dataset, duplicate_policy="reject"):
    samples = []
    for path in sorted(dataset.glob("kos*/*.jpg")):
        label_path = path.with_name(path.stem + "_label.bmp")
        mask = cv2.imread(str(label_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError(f"missing or unreadable annotation: {label_path}")
        samples.append({"path": path, "item": path.parent.name, "image": path.name,
                        "label": int(np.any(mask > 0)),
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if not samples:
        raise ValueError("no KolektorSDD kos*/*.jpg images found")
    # Folder names alone do not prove disjoint content. Identical bytes stored under two
    # item folders would otherwise sit on both sides of the split (see DATASET.md), so the
    # split group is assigned by the explicit duplicate policy, never by folder name alone.
    return assign_split_groups(samples, duplicate_policy)


def split_groups(samples):
    groups = sorted({sample["group"] for sample in samples})
    if len(groups) < 10:
        raise ValueError("at least 10 physical item groups required")
    shuffled = np.random.default_rng(2026).permutation(groups)
    first, second = int(.6*len(groups)), int(.8*len(groups))
    assignments = {group: partition for partition, subset in zip(
        ["train", "validation", "test"], [shuffled[:first], shuffled[first:second], shuffled[second:]]) for group in subset}
    return np.array([assignments[sample["group"]] for sample in samples])


def case_title(sample, score, prediction):
    # Labels and predictions are printed as 0/1 even when the caller passes numpy booleans.
    name = sample.get("item", sample["group"])
    return f'{name}/{sample["path"].stem}\ntrue={int(sample["label"])} pred={int(prediction)}\np={float(score):.3f}'


def plot_evaluation(samples, scores, predictions, output):
    mistakes = [i for i, sample in enumerate(samples) if sample["label"] != predictions[i]]
    chosen = (mistakes + [i for i in range(len(samples)) if i not in mistakes])[:6]
    figure, axes = plt.subplots(1, len(chosen), figsize=(12, 7))
    for axis, index in zip(np.atleast_1d(axes), chosen):
        sample = samples[index]
        image = preprocess(cv2.imread(str(sample["path"]), cv2.IMREAD_GRAYSCALE))
        axis.imshow(image, cmap="gray", vmin=0, vmax=255)
        axis.set_title(case_title(sample, scores[index], predictions[index]), fontsize=9)
        axis.axis("off")
    figure.suptitle("Held-out cases, failures first | KolektorSDD / ViCoS, CC BY-NC-SA 4.0")
    figure.tight_layout()
    figure.savefig(output / "heldout_cases.png", dpi=130)
    plt.close(figure)


def save_manifest(samples, splits, scores, output):
    with (output / "split_manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["item", "group", "image", "split", "label", "score", "sha256"])
        writer.writerows((s["item"], s["group"], s["path"].name, partition, s["label"], score, s["sha256"])
                         for s, partition, score in zip(samples, splits, scores))


def audit_summary(audit):
    return {key: audit[key] for key in ("policy", "image_count", "item_folder_count", "distinct_content_count",
                                        "split_group_count", "cross_item_image_count", "merged_split_groups",
                                        "boundary_safe")}


def evaluate(dataset, output, duplicate_policy="reject"):
    cv2.setNumThreads(1)
    samples = records(dataset, duplicate_policy)
    audit = content_audit(samples, duplicate_policy)
    splits = split_groups(samples)
    features = np.array([defect_features(cv2.imread(str(s["path"]), cv2.IMREAD_GRAYSCALE)) for s in samples])
    labels = np.array([s["label"] for s in samples])
    detector = train(features[splits == "train"], labels[splits == "train"])
    scores = detector.scores(features)
    threshold = select_threshold(labels[splits == "validation"], scores[splits == "validation"])
    predictions = np.uint8(scores >= threshold)
    output.mkdir(parents=True, exist_ok=True)
    (output / "duplicate_audit.json").write_text(json.dumps(audit, indent=2), encoding="utf-8")
    save_manifest(samples, splits, scores, output)
    np.savez(output / "detector.npz", mean=detector.mean, scale=detector.scale,
             coefficients=detector.coefficients, threshold=threshold)
    report = {"threshold_selected_on_validation": threshold, "seed": 2026,
              "dataset": f"KolektorSDD, {len(samples)} real industrial images; split-group-disjoint custom split "
                         f"({audit['split_group_count']} content-checked groups), not official benchmark folds",
              "duplicate_policy": duplicate_policy, "content_hash_audit": audit_summary(audit),
              "calibration": "Not applicable: no geometric measurement or calibration target/metadata; image classification only",
              "splits": {split: confusion(labels[splits == split], predictions[splits == split]) for split in ["train", "validation", "test"]}}
    test_indices = np.flatnonzero(splits == "test")
    plot_evaluation([samples[i] for i in test_indices], scores[test_indices], predictions[test_indices], output)
    report["failures"] = [{"image": f'{samples[i]["item"]}/{samples[i]["path"].name}', "truth": int(labels[i]),
                            "prediction": int(predictions[i]), "score": float(scores[i])} for i in test_indices if predictions[i] != labels[i]]
    (output / "inspection_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duplicate-policy", choices=POLICIES, default="reject",
                        help="reject: refuse identical content in two item folders (default); "
                             "merge: fuse such items into one split group. See DATASET.md.")
    arguments = parser.parse_args()
    print(json.dumps(evaluate(arguments.dataset, arguments.output, arguments.duplicate_policy), indent=2))


if __name__ == "__main__":
    main()
