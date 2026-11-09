"""OpenCV feature extraction and an interpretable binary inspection model."""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import minimize
from scipy.special import expit


def preprocess(image):
    if image is None or image.ndim != 2 or image.dtype != np.uint8 or min(image.shape) < 32:
        raise ValueError("inspection requires an 8-bit grayscale image at least 32x32")
    return cv2.resize(image, (256, 704), interpolation=cv2.INTER_AREA)


def defect_features(image):
    gray = preprocess(image)
    smoothed = cv2.GaussianBlur(gray, (3, 3), 0)
    blackhat = cv2.morphologyEx(smoothed, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    # Exclude border artifacts caused by cropping different physical surfaces.
    response = blackhat[20:-20, 12:-12]
    features = list(np.quantile(response, [.5, .9, .99, .999]))
    features.extend([float(response.mean()), float(response.std())])
    for threshold in [15, 30, 45, 60]:
        mask = np.uint8(response > threshold)
        count, _, regions, _ = cv2.connectedComponentsWithStats(mask)
        sizes = regions[1:, cv2.CC_STAT_AREA] if count > 1 else np.array([0])
        features.extend([float(mask.mean()), float(sizes.max()), float(np.sum(sizes >= 10))])
    return np.asarray(features, dtype=float)


def confusion(labels, predictions):
    truth, predicted = np.asarray(labels), np.asarray(predictions)
    if truth.ndim != 1 or truth.shape != predicted.shape or truth.size == 0:
        raise ValueError("nonempty label and prediction vectors must have equal shape")
    if not np.isin(truth, [0, 1]).all() or not np.isin(predicted, [0, 1]).all():
        raise ValueError("binary labels must be 0 or 1")
    tn, fp = int(np.sum((truth == 0) & (predicted == 0))), int(np.sum((truth == 0) & (predicted == 1)))
    fn, tp = int(np.sum((truth == 1) & (predicted == 0))), int(np.sum((truth == 1) & (predicted == 1)))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    return {"matrix_rows_true_columns_predicted": [[tn, fp], [fn, tp]], "precision": precision,
            "recall": recall, "f1": 2*tp / (2*tp+fp+fn) if 2*tp+fp+fn else 0.0,
            "accuracy": (tn+tp) / truth.size, "count": int(truth.size)}


@dataclass(frozen=True)
class Detector:
    mean: np.ndarray
    scale: np.ndarray
    coefficients: np.ndarray

    def scores(self, features):
        normalized = (np.asarray(features) - self.mean) / self.scale
        design = np.column_stack((normalized, np.ones(len(normalized))))
        return expit(design @ self.coefficients)


def train(features, labels, penalty=.05):
    features, labels = np.asarray(features, dtype=float), np.asarray(labels, dtype=float)
    if features.ndim != 2 or labels.shape != (len(features),) or set(labels) != {0, 1}:
        raise ValueError("training needs a feature matrix with both binary classes")
    if not np.isfinite(features).all():
        raise ValueError("features must be finite")
    if not np.isfinite(penalty) or penalty <= 0:
        raise ValueError("regularization penalty must be finite and positive")
    mean, scale = features.mean(axis=0), features.std(axis=0)
    scale[scale < 1e-12] = 1
    design = np.column_stack(((features-mean)/scale, np.ones(len(features))))
    weights = np.where(labels == 1, .5 / labels.mean(), .5 / (1-labels.mean()))
    def loss(coefficients):
        logits = design @ coefficients
        regularization = penalty * np.sum(coefficients[:-1]**2)
        objective = np.mean(weights * (np.logaddexp(0, logits) - labels*logits)) + regularization
        gradient = design.T @ (weights * (expit(logits)-labels)) / len(labels)
        gradient[:-1] += 2*penalty * coefficients[:-1]
        return objective, gradient
    fit = minimize(loss, np.zeros(design.shape[1]), jac=True, method="L-BFGS-B")
    if not fit.success:
        raise RuntimeError(f"logistic optimization failed: {fit.message}")
    return Detector(mean, scale, fit.x)


def select_threshold(labels, scores):
    if set(labels) != {0, 1}:
        raise ValueError("validation needs both classes")
    candidates = np.unique(np.r_[0, scores, 1])
    # F1 ties favor fewer missed defects; test labels are never used here.
    return float(max(candidates, key=lambda threshold: (
        confusion(labels, scores >= threshold)["f1"], -threshold)))
