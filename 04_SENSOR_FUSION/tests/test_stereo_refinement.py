"""Stereo extrinsic self-refinement: synthetic recovery, gating, determinism, invalid input."""
import cv2
import numpy as np
import pytest
import yaml

from test_euroc import make_sequence
from vio.euroc import stereo_rectification
from vio.geometry import rotation_exp, rotation_log
from vio.stereo_refinement import (GATES, StereoGeometry, applied_correction, correction_between,
                                   fit_rotation_correction, normalized_coordinates, rectified_vertical,
                                   refine_from_pairs, sampson_pixels)


def euroc_like_geometry():
    """Nonparallel supplied rig with the public EuRoC intrinsics/distortion magnitudes."""
    kl = np.array([[458.654, 0, 367.215], [0, 457.296, 248.375], [0, 0, 1.]])
    kr = np.array([[457.587, 0, 379.999], [0, 456.134, 255.238], [0, 0, 1.]])
    dl = np.array([-.28340811, .07395907, .00019359, 1.76187114e-05])
    dr = np.array([-.28368365, .07451284, -.00010473, -3.555907e-05])
    rotation = rotation_exp([.004, -.003, .0025])
    translation = np.array([-.110074, .000398, -.000853])
    return StereoGeometry(kl, dl, kr, dr, rotation, translation, (752, 480))


def synthetic_correspondences(geometry, true_delta_deg, seed=5, count=900, noise_px=.1):
    """Project random points through BOTH distorted cameras with the TRUE right rotation."""
    random = np.random.default_rng(seed)
    points = random.uniform([-1.5, -1.0, 2.5], [1.5, 1.0, 7.0], (count, 3))
    true_rotation = geometry.rotated(np.radians(true_delta_deg))
    left = cv2.projectPoints(points, np.zeros(3), np.zeros(3), geometry.left_intrinsic, geometry.left_distortion)[0].reshape(-1, 2)
    right = cv2.projectPoints(points, cv2.Rodrigues(true_rotation)[0], geometry.translation,
                              geometry.right_intrinsic, geometry.right_distortion)[0].reshape(-1, 2)
    inside = ((left >= 10) & (left <= [742, 470]) & (right >= 10) & (right <= [742, 470])).all(axis=1)
    left, right = left[inside], right[inside]
    return left+random.normal(0, noise_px, left.shape), right+random.normal(0, noise_px, right.shape)


@pytest.mark.parametrize("true_delta_deg", [[.2, -.1, .15], [0., 0., 0.], [-.35, .05, -.2]])
@pytest.mark.parametrize("noise_px", [0., .1])
def test_injected_right_rotation_is_recovered_to_tight_tolerance(true_delta_deg, noise_px):
    geometry = euroc_like_geometry()
    left, right = synthetic_correspondences(geometry, true_delta_deg, noise_px=noise_px)
    nl = normalized_coordinates(left, geometry.left_intrinsic, geometry.left_distortion)
    nr = normalized_coordinates(right, geometry.right_intrinsic, geometry.right_distortion)
    delta, report = fit_rotation_correction(nl, nr, geometry)
    assert report["success"]
    recovered = np.degrees(delta)
    error = np.abs(recovered-np.asarray(true_delta_deg))
    if noise_px == 0.:
        assert error.max() < 1e-6, error  # exact and unbiased on noise-free distorted projections
    else:
        # Pitch (x) is strongly observable, roll (z) less, vergence (y) only through the x*y
        # term of the epipolar constraint; 900 points at 0.1 px give these precision limits.
        assert error[0] <= .002 and error[2] <= .01 and error[1] <= .02, error
    if not np.any(true_delta_deg):
        assert np.linalg.norm(recovered) < .02
    assert abs(np.median(rectified_vertical(left, right, geometry, delta)[0])) < .05
    np.testing.assert_allclose(correction_between(geometry.rotation, geometry.rotated(delta)), delta, atol=1e-12)


def test_supplied_geometry_shows_the_injected_vertical_residual_before_refinement():
    # A 0.2 deg pitch injection should appear as roughly 1.6 px vertical residual at f=458.
    geometry = euroc_like_geometry()
    left, right = synthetic_correspondences(geometry, [.2, 0, 0], noise_px=0.)
    before = np.median(np.abs(rectified_vertical(left, right, geometry)[0]))
    assert 1.2 < before < 2.2
    sampson = sampson_pixels(np.zeros(3), normalized_coordinates(left, geometry.left_intrinsic, geometry.left_distortion),
                             normalized_coordinates(right, geometry.right_intrinsic, geometry.right_distortion), geometry)
    assert np.median(np.abs(sampson)) > .8


def textured_pair_images(geometry_free_intrinsic, baseline, camera_rotation, camera_position, delta, seed):
    """Plane at depth 5 m rendered for a left camera and a right camera rotated by Exp(-delta) about its center."""
    random = np.random.default_rng(seed)
    texture = cv2.GaussianBlur(random.integers(0, 256, (480, 640), np.uint8), (5, 5), .9)
    images = []
    for which in ["left", "right"]:
        rotation = camera_rotation if which == "left" else camera_rotation @ rotation_exp(-delta)
        position = camera_position if which == "left" else camera_position+rotation @ [baseline, 0, 0]
        homography = rotation.T @ (np.eye(3)-np.outer(position, [0, 0, 1])/5.)
        homography = geometry_free_intrinsic @ homography @ np.linalg.inv(geometry_free_intrinsic)
        images.append(cv2.warpPerspective(texture, homography, (640, 480), flags=cv2.INTER_LINEAR))
    return images


def rendered_pairs(delta, count=12):
    intrinsic = np.array([[360., 0, 320], [0, 360., 240], [0, 0, 1]])
    geometry = StereoGeometry(intrinsic, np.zeros(4), intrinsic, np.zeros(4), np.eye(3), np.array([-.25, 0, 0]), (640, 480))
    pairs = []
    for index in range(count):
        rotation = rotation_exp([.004*index, -.003*index, .002*index])
        position = np.array([.03*index, -.02*index, .01*index])
        left, right = textured_pair_images(intrinsic, .25, rotation, position, delta, seed=100+index)
        pairs.append((1_400_000_000_000_000_000+index*250_000_000, left, right))
    return pairs, geometry


def test_rendered_right_camera_rotation_is_applied_and_held_out_residual_drops():
    cv2.setNumThreads(1)
    delta = np.radians([.25, -.1, .12])
    pairs, geometry = rendered_pairs(delta)
    report = refine_from_pairs(pairs, geometry)
    assert report["status"] == "APPLIED"
    assert report["counts"]["fit_pairs"] == 6 and report["counts"]["held_out_pairs"] == 6
    assert set(report["fit_timestamps_ns"]).isdisjoint(report["held_out_timestamps_ns"])
    np.testing.assert_allclose(report["correction"]["degrees_per_axis"], np.degrees(delta), atol=.03)
    assert report["supplied"]["held_out"]["max_abs_quadrant_signed_median_px"] > 1.0
    assert report["refined"]["held_out"]["max_abs_quadrant_signed_median_px"] <= GATES["held_out_median_limit_px"]
    assert min(report["refined"]["held_out"]["quadrant_counts"]) >= GATES["min_held_out_per_quadrant"]
    assert applied_correction(report) is not None and report["ground_truth_used"] is False


def test_zero_injection_gives_near_zero_correction_and_is_deterministic():
    cv2.setNumThreads(1)
    pairs, geometry = rendered_pairs(np.zeros(3))
    first = refine_from_pairs(pairs, geometry)
    second = refine_from_pairs(pairs, geometry)
    assert first == second
    assert first["status"] in {"APPLIED", "RETAINED_SUPPLIED"}
    assert first["correction"]["magnitude_deg"] < .05
    assert first["refined"]["held_out"]["max_abs_quadrant_signed_median_px"] <= GATES["held_out_median_limit_px"]


def test_correction_above_declared_bound_is_rejected_and_bad_supplied_geometry_fails_loudly():
    cv2.setNumThreads(1)
    pairs, geometry = rendered_pairs(np.radians([1.8, 0, 0]))
    with pytest.raises(ValueError, match="rejected and supplied calibration fails"):
        refine_from_pairs(pairs, geometry)


def test_insufficient_pairs_retain_supplied_geometry_without_fitting():
    pairs, geometry = rendered_pairs(np.zeros(3), count=3)
    report = refine_from_pairs(pairs, geometry)
    assert report["status"] == "NOT_EVALUATED"
    assert applied_correction(report) is None
    assert "optimizer" not in report


def test_empty_held_out_quadrant_retains_supplied_geometry_and_reports_the_unapplied_fit():
    cv2.setNumThreads(1)
    pairs, geometry = rendered_pairs(np.radians([.25, 0, 0]))
    # Blank the top half of every held-out (odd) pair so two quadrants lose their correspondences.
    for index in range(1, len(pairs), 2):
        for image in pairs[index][1:]:
            image[:240] = 0
    report = refine_from_pairs(pairs, geometry)
    assert report["status"] == "NOT_EVALUATED"
    assert applied_correction(report) is None
    assert abs(report["correction"]["fitted_but_not_applied_deg"][0]-.25) < .03


@pytest.mark.parametrize("left, right, message", [
    (np.zeros((2, 2)), np.zeros((2, 2)), "at least three"),
    (np.zeros((5, 2)), np.zeros((4, 2)), "equal shape"),
    (np.full((5, 2), np.nan), np.zeros((5, 2)), "finite"),
])
def test_invalid_fit_inputs_are_rejected(left, right, message):
    with pytest.raises(ValueError, match=message):
        fit_rotation_correction(left, right, euroc_like_geometry())


def test_invalid_geometry_is_rejected():
    kl = np.eye(3)*[400, 400, 1]
    with pytest.raises(ValueError, match="proper orthonormal"):
        StereoGeometry(kl, np.zeros(4), kl, np.zeros(4), np.diag([1., 1., -1.]), np.array([-.1, 0, 0]), (640, 480))
    with pytest.raises(ValueError, match="nonzero"):
        StereoGeometry(kl, np.zeros(4), kl, np.zeros(4), np.eye(3), np.zeros(3), (640, 480))


def test_rectification_with_zero_correction_is_unchanged_and_left_body_extrinsic_is_preserved(tmp_path):
    make_sequence(tmp_path)
    body_left = np.eye(4)
    body_left[:3, :3], body_left[:3, 3] = rotation_exp([.2, -.1, .3]), [.1, -.2, .05]
    for camera, baseline in [("cam0", 0.), ("cam1", .25)]:
        path = tmp_path/camera/"sensor.yaml"
        configuration = yaml.safe_load(path.read_text())
        transform = body_left.copy()
        transform[:3, 3] += body_left[:3, :3] @ [baseline, 0, 0]
        configuration["T_BS"]["data"] = transform.ravel().tolist()
        path.write_text(yaml.safe_dump(configuration))
    plain, zero = stereo_rectification(tmp_path), stereo_rectification(tmp_path, np.zeros(3))
    np.testing.assert_array_equal(plain.left_maps[0], zero.left_maps[0])
    np.testing.assert_array_equal(plain.right_maps[1], zero.right_maps[1])
    np.testing.assert_array_equal(plain.calibration.body_from_camera, zero.calibration.body_from_camera)
    corrected = stereo_rectification(tmp_path, np.radians([.3, 0, 0]))
    assert not np.array_equal(plain.right_maps[1], corrected.right_maps[1])
    # The left body extrinsic contract is kept: body_from_camera = T_BS_left @ [r1^T].
    rectifying = np.linalg.inv(body_left) @ corrected.calibration.body_from_camera
    np.testing.assert_allclose(rectifying[:3, 3], 0, atol=1e-12)
    assert np.linalg.norm(rotation_log(rectifying[:3, :3])) < np.radians(.4)
    assert corrected.calibration.baseline_m == pytest.approx(.25, abs=1e-6)
    with pytest.raises(ValueError, match="rotation 3-vector"):
        stereo_rectification(tmp_path, [np.nan, 0, 0])


def _decision_report(monkeypatch, supplied_px, refined_px, magnitude_deg=.2):
    """Drive refine_from_pairs through controlled held-out statistics to test the decision rule alone."""
    import vio.stereo_refinement as module

    def fake_fit(left_normalized, right_normalized, geometry):
        delta = np.zeros(3)
        delta[0] = np.radians(magnitude_deg)
        return delta, {"success": True, "evaluations": 1, "cost": 0., "message": "stub"}

    def fake_evaluate(subset, geometry, correction):
        value = supplied_px if correction is None else refined_px
        return {"count": 400, "median_abs_vertical_px": abs(value), "median_signed_vertical_px": value,
                "quadrant_counts": [100, 100, 100, 100], "quadrant_signed_medians_px": [value]*4,
                "max_abs_quadrant_signed_median_px": abs(value), "share_within_gate": 1.,
                "sampson_median_abs_px": abs(value), "sampson_quantiles_05_50_95_px": [value]*3}
    monkeypatch.setattr(module, "fit_rotation_correction", fake_fit)
    monkeypatch.setattr(module, "_evaluate", fake_evaluate)
    pairs, geometry = rendered_pairs(np.zeros(3))
    return refine_from_pairs(pairs, geometry)


def test_refinement_that_worsens_a_good_supplied_geometry_is_not_applied(monkeypatch):
    # Added after independent review: the `refined_not_worse_than_supplied` clause needs its own test.
    report = _decision_report(monkeypatch, supplied_px=.10, refined_px=.20)
    assert report["checks"]["refined_held_out_within_limit"] is True
    assert report["checks"]["refined_not_worse_than_supplied"] is False
    assert report["status"] == "RETAINED_SUPPLIED" and applied_correction(report) is None


@pytest.mark.parametrize("refined_px, expected", [(.24, "APPLIED"), (.26, "ValueError")])
def test_held_out_limit_is_a_behavioural_boundary(monkeypatch, refined_px, expected):
    # Added after independent review: a refined offset just inside/outside the 0.25 px limit must flip the decision.
    assert GATES["held_out_median_limit_px"] == .25
    if expected == "ValueError":
        with pytest.raises(ValueError, match="limit 0.25 px"):
            _decision_report(monkeypatch, supplied_px=1.8, refined_px=refined_px)
    else:
        report = _decision_report(monkeypatch, supplied_px=1.8, refined_px=refined_px)
        assert report["status"] == "APPLIED"
        np.testing.assert_allclose(applied_correction(report), [np.radians(.2), 0, 0])
