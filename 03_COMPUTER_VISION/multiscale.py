"""Native-resolution edge-inclusive morphology; no annotation input."""

import cv2
import numpy as np

from inspection import defect_features, preprocess


def enhanced_features(image):
    preprocess(image)  # Shared input contract; all added features use the original pixels.
    features = list(defect_features(image))
    height, width = image.shape
    for kernel in [(1, 7), (1, 21), (1, 51), (9, 9), (31, 31)]:
        response = cv2.morphologyEx(image, cv2.MORPH_BLACKHAT,
                                    cv2.getStructuringElement(cv2.MORPH_RECT, kernel),
                                    borderType=cv2.BORDER_REFLECT_101)
        features.extend(np.quantile(response, [.9, .99, .999, .9999])/255)
        features.extend([response.mean()/255, response.std()/255])
        row_response = response.mean(axis=1)/255
        features.extend([row_response.max(), np.quantile(row_response, .99), np.quantile(row_response, .95)])
        for extent in [32, 96, 256]:
            pooled = cv2.boxFilter(response.astype(np.float32)/255, -1,
                                   (min(extent, width), 1), borderType=cv2.BORDER_REFLECT_101)
            features.extend([float(pooled.max()), float(np.quantile(pooled, .999)), float(np.quantile(pooled, .99))])
        for threshold in [15, 30]:
            count, _, regions, _ = cv2.connectedComponentsWithStats(np.uint8(response > threshold))
            regions = regions[1:] if count > 1 else np.zeros((1, 5))
            area = regions[:, cv2.CC_STAT_AREA]
            features.extend([float(area.max()/(height*width)), float(np.sort(area)[-3:].sum()/(height*width)),
                             float(regions[:, cv2.CC_STAT_WIDTH].max()/width),
                             float(regions[:, cv2.CC_STAT_HEIGHT].max()/height), float(np.sum(area >= 10)/(height*width))])
    return np.asarray(features, dtype=float)
