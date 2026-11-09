from pathlib import Path

import numpy as np

from geometry import public_blade, sample_surface


def test_public_blade_uniform_scale_preserves_aspect_ratio():
    source = Path(__file__).resolve().parents[1] / "assets/public_blade/blade_geometry.obj"
    vertices, faces, transform = public_blade(source)
    assert vertices.shape == (600, 3)
    assert faces.shape == (1102, 3)
    np.testing.assert_allclose([vertices[:, 2].min(), vertices[:, 2].max()], [0.2, 3.2])
    original = np.diff(np.asarray(transform["original_bounds"]), axis=0)[0]
    np.testing.assert_allclose(np.ptp(vertices, axis=0) / original, 3 / 60.5)


def test_surface_sampling_is_area_weighted_and_lies_on_triangles():
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [10, 0, 0], [12, 0, 0], [10, 2, 0]])
    points = sample_surface(vertices, [[0, 1, 2], [3, 4, 5]], count=12000)
    first = points[:, 0] < 5
    assert abs(first.mean() - 0.2) < 0.015
    np.testing.assert_allclose(points[:, 2], 0)
    assert ((points[first, 0] + points[first, 1]) <= 1).all()
    assert ((points[~first, 0] - 10 + points[~first, 1]) <= 2).all()
