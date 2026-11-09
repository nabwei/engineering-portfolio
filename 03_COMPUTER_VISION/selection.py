"""Grouped model selection consumes only the supplied development rows."""

from dataclasses import dataclass

import cv2
import numpy as np

from inspection import Detector, confusion, train


CANDIDATES = [
    ("original_logistic_005", "original", "logistic", .05),
    ("multiscale_logistic_0005", "enhanced", "logistic", .005),
    ("multiscale_logistic_005", "enhanced", "logistic", .05),
    ("multiscale_logistic_05", "enhanced", "logistic", .5),
    ("multiscale_forest_6", "enhanced", "forest", 6),
    ("multiscale_forest_12", "enhanced", "forest", 12),
]


def grouped_folds(groups, labels, folds, seed=20261026):
    groups, labels = np.asarray(groups), np.asarray(labels)
    if labels.shape != groups.shape or groups.ndim != 1 or not np.isin(labels, [0, 1]).all():
        raise ValueError("paired binary labels and physical-item group vectors required")
    unique = np.unique(groups)
    if folds < 2 or len(unique) < folds:
        raise ValueError("not enough physical items for grouped folds")
    shuffled = np.random.default_rng(seed).permutation(unique)
    ordered = sorted(shuffled, key=lambda group: (-int(labels[groups == group].sum()), -int(np.sum(groups == group))))
    positive_counts, image_counts = np.zeros(folds, int), np.zeros(folds, int)
    assignment = np.empty(len(groups), int)
    for group in ordered:
        mask = groups == group
        positives = int(labels[mask].sum())
        index = min(range(folds), key=lambda i: (positive_counts[i] if positives else 0, image_counts[i], i))
        assignment[mask] = index
        positive_counts[index] += int(labels[mask].sum())
        image_counts[index] += int(mask.sum())
    for index in range(folds):
        if set(labels[assignment == index]) != {0, 1} or set(labels[assignment != index]) != {0, 1}:
            raise ValueError("every train/validation fold must contain both classes")
    return assignment


def f2(metrics):
    (_, fp), (fn, tp) = metrics["matrix_rows_true_columns_predicted"]
    return 5*tp/(5*tp+4*fn+fp) if 5*tp+4*fn+fp else 0.


def threshold_selection(labels, scores):
    labels, scores = np.asarray(labels), np.asarray(scores)
    if labels.shape != scores.shape or set(labels) != {0, 1} or not np.isfinite(scores).all():
        raise ValueError("selection requires paired finite scores and both classes")
    choices = []
    for threshold in np.unique(np.r_[0., scores, 1.]):
        metric = confusion(labels, scores >= threshold)
        choices.append((metric["precision"] >= .8, f2(metric), metric["recall"], -float(threshold), metric))
    feasible, _, _, negative_threshold, metrics = max(choices, key=lambda choice: choice[:4])
    return {"threshold": -negative_threshold, "precision_constraint_met": bool(feasible), "f2": f2(metrics), "metrics": metrics}


@dataclass
class Forest:
    model: object

    def scores(self, features):
        votes = self.model.getVotes(np.ascontiguousarray(features, dtype=np.float32), 0)
        positive = np.flatnonzero(votes[0] == 1)
        if len(positive) != 1:
            raise RuntimeError("forest class labels are not binary")
        return votes[1:, positive[0]]/votes[1:].sum(axis=1)


def fit(candidate, features, labels):
    labels = np.asarray(labels)
    _, family, kind, parameter = candidate
    values = features[family]
    if kind == "logistic":
        return train(values, labels, parameter)
    if set(labels) != {0, 1} or not np.isfinite(values).all():
        raise ValueError("forest needs finite features and both classes")
    cv2.setRNGSeed(20261026)
    model = cv2.ml.RTrees_create()
    model.setMaxDepth(int(parameter))
    model.setMinSampleCount(3)
    model.setActiveVarCount(max(1, int(np.sqrt(values.shape[1]))))
    model.setPriors(np.array([.5/np.mean(labels == 0), .5/np.mean(labels == 1)], np.float32))
    model.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, 200, 0))
    if not model.train(np.ascontiguousarray(values, np.float32), cv2.ml.ROW_SAMPLE, np.asarray(labels, np.int32)):
        raise RuntimeError("forest training failed")
    return Forest(model)


def select_candidate(features, labels, groups, folds):
    assignments = grouped_folds(groups, labels, folds)
    reports = []
    for candidate in CANDIDATES:
        scores = np.empty(len(labels))
        for index in range(folds):
            training, validation = assignments != index, assignments == index
            model = fit(candidate, {name: values[training] for name, values in features.items()}, labels[training])
            scores[validation] = model.scores(features[candidate[1]][validation])
        report = threshold_selection(labels, scores)
        reports.append({"candidate": candidate[0], **report, "oof_scores": scores.tolist()})
    winner = max(range(len(reports)), key=lambda i: (
        reports[i]["precision_constraint_met"], reports[i]["f2"], reports[i]["metrics"]["recall"], -i))
    return CANDIDATES[winner], reports[winner]["threshold"], reports, assignments


def save_model(model, path):
    if isinstance(model, Detector):
        np.savez(path.with_suffix(".npz"), mean=model.mean, scale=model.scale, coefficients=model.coefficients)
        return path.with_suffix(".npz")
    model.model.save(str(path.with_suffix(".xml")))
    return path.with_suffix(".xml")


def load_model(path):
    if path.suffix == ".npz":
        with np.load(path) as parameters:
            return Detector(parameters["mean"], parameters["scale"], parameters["coefficients"])
    return Forest(cv2.ml.RTrees_load(str(path)))
