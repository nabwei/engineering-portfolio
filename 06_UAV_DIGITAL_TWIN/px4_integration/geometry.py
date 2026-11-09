"""Minimal OBJ data import and area-weighted surface reference sampling."""
import numpy as np


def public_blade(path):
    vertices, triangles = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(value) for value in fields[1:4]])
        elif fields and fields[0] == "f":
            indices = [int(value.split("/")[0]) - 1 for value in fields[1:]]
            triangles.extend([indices[0], indices[index], indices[index + 1]] for index in range(1, len(indices) - 1))
    vertices, triangles = np.asarray(vertices, dtype=float), np.asarray(triangles, dtype=int)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError("OBJ must contain finite 3D vertices")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or triangles.min() < 0 or triangles.max() >= len(vertices):
        raise ValueError("OBJ requires valid positive face indices")
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    if high[2] <= low[2]:
        raise ValueError("Blade must have positive height")
    scale = 3.0 / (high[2] - low[2])
    origin = np.array([(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]])
    transformed = (vertices - origin) * scale + [0, 0, 0.2]
    faces = transformed[triangles]
    if (np.linalg.norm(np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0]), axis=1) <= 1e-12).any():
        raise ValueError("OBJ contains degenerate triangles")
    metadata = {"uniform_scale": float(scale), "subtract_original_origin": origin.tolist(),
                "add_scaled_offset": [0, 0, 0.2], "original_bounds": [low.tolist(), high.tolist()],
                "scaled_bounds": [transformed.min(axis=0).tolist(), transformed.max(axis=0).tolist()]}
    return transformed, triangles, metadata


def sample_surface(vertices, faces, count=12000):
    triangles = np.asarray(vertices)[np.asarray(faces)]
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1) / 2
    generator = np.random.default_rng(1701)
    chosen = triangles[generator.choice(len(triangles), count, p=areas / areas.sum())]
    root, other = np.sqrt(generator.random(count)), generator.random(count)
    weights = np.stack([1 - root, root * (1 - other), root * other], axis=1)
    return np.sum(chosen * weights[..., None], axis=1)
