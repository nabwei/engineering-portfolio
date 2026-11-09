"""Apply a frozen inspection model to one image without requiring an annotation."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2

from inspection import Detector, defect_features
from multiscale import enhanced_features
from selection import load_model


def validated_selection(configuration):
    extractors = {"original": defect_features, "enhanced": enhanced_features}
    if not isinstance(configuration, dict):
        raise ValueError("frozen selection must be a JSON object")
    candidate = configuration.get("candidate")
    if (not isinstance(candidate, list) or len(candidate) < 2
            or not isinstance(candidate[1], str) or candidate[1] not in extractors):
        raise ValueError("frozen selection requires a known feature family")
    threshold = configuration.get("threshold")
    if (isinstance(threshold, bool) or not isinstance(threshold, (int, float))
            or not 0 <= threshold <= 1 or not math.isfinite(threshold)):
        raise ValueError("frozen threshold must be a finite probability in [0, 1]")
    validated_model_name(configuration)
    return extractors[candidate[1]], threshold


def validated_model_name(configuration):
    model_name = configuration.get("model_file")
    # A missing, empty, non-string or path-like name must fail here, not as KeyError/TypeError
    # or as an attempt to read a directory.
    if (not isinstance(model_name, str) or not model_name or model_name in (".", "..")
            or Path(model_name).name != model_name):
        raise ValueError("frozen selection requires a model file name inside the selected model directory")
    return model_name


def verified_model(model_directory, configuration):
    model_name = validated_model_name(configuration)
    path = model_directory / model_name
    if hashlib.sha256(path.read_bytes()).hexdigest() != configuration.get("model_sha256"):
        raise ValueError("model SHA-256 does not match frozen selection")
    return load_model(path)


def verify_feature_dimension(model, dimension):
    if isinstance(model, Detector):
        compatible = (model.mean.shape == (dimension,) and model.scale.shape == (dimension,)
                      and model.coefficients.shape == (dimension+1,))
    else:
        compatible = model.model.getVarCount() == dimension
    if not compatible:
        raise ValueError("model feature dimension does not match the selected extractor")


def inspection_decision(model_directory, image_path):
    configuration = json.loads((model_directory / "frozen_selection.json").read_text(encoding="utf-8"))
    extract, threshold = validated_selection(configuration)
    model = verified_model(model_directory, configuration)
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    features = extract(image)
    verify_feature_dimension(model, len(features))
    score = float(model.scores(features[None, :])[0])
    return {"image": str(image_path), "score": score, "threshold": threshold,
            "prediction": "defect" if score >= threshold else "normal",
            "model": configuration["candidate"][0], "limitation": "Research classifier score; not a calibrated safety probability."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-directory", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    arguments = parser.parse_args()
    print(json.dumps(inspection_decision(arguments.model_directory, arguments.image), indent=2))
