from copy import deepcopy

import numpy as np
import pytest

from vio.eskf import Noise, State
from vio.geometry import rotation_exp, rotation_log
from vio.initialization import quiet_initialization
from vio.relative import RelativePoseFilter, inject_state, relative_observation


def test_relative_jacobian_matches_independent_perturbed_pose():
    state = State(position=[2., -.3, 1.2], rotation=rotation_exp([.3, -.2, .1]))
    clone = State(position=[.4, .2, -.1], rotation=rotation_exp([-.1, .2, .3]))
    _, rotation, analytical = relative_observation(state, clone)
    numerical = np.zeros((6, 21))
    for column in range(21):
        predictions = []
        for sign in [-1, 1]:
            perturbed, old = deepcopy(state), deepcopy(clone)
            delta = np.zeros(21)
            delta[column] = sign*1e-6
            inject_state(perturbed, delta[:15])
            old.position += delta[15:18]
            old.rotation = old.rotation @ rotation_exp(delta[18:21])
            predicted_t = old.rotation.T @ (perturbed.position-old.position)
            predicted_r = old.rotation.T @ perturbed.rotation
            predictions.append(np.r_[predicted_t, rotation_log(rotation.T @ predicted_r)])
        numerical[:, column] = (predictions[1]-predictions[0])/2e-6
    np.testing.assert_allclose(analytical, numerical, atol=1e-8, rtol=1e-6)


def test_relative_measurement_preserves_unobservable_global_position_uncertainty():
    estimator = RelativePoseFilter(noise=Noise(0, 0, 0, 0))
    prior = estimator.covariance[:3, :3].copy()
    for _ in range(20):
        for _ in range(4):
            estimator.predict(np.array([0, 0, 9.80665]), np.zeros(3), .01)
        _, _, observation = relative_observation(estimator.state, estimator.clone)
        before_relative = np.trace(observation[:3] @ estimator.covariance @ observation[:3].T)
        assert estimator.update_relative(np.zeros(3), np.eye(3), np.eye(6)*1e-8)
        after_relative = np.trace(observation[:3] @ estimator.covariance @ observation[:3].T)
        assert after_relative < before_relative
        assert np.linalg.eigvalsh(estimator.covariance[:3, :3]-prior).min() >= -1e-10
        estimator.clone_current_pose()


def test_stationary_relative_updates_estimate_acceleration_bias_without_resets():
    estimator = RelativePoseFilter()
    measurement_covariance = np.diag([1e-6]*3 + [1e-6]*3)
    for frame in range(160):
        estimator.clone_current_pose()
        for _ in range(10):
            estimator.predict(np.array([.12, -.06, 9.80665]), np.zeros(3), .005)
        assert estimator.update_relative(np.zeros(3), np.eye(3), measurement_covariance)
        assert np.linalg.eigvalsh(estimator.covariance).min() > -1e-10
    # Horizontal bias and gravity tilt are ambiguous at rest; verify observable
    # motion is suppressed, not a falsely identifiable individual bias value.
    assert np.linalg.norm(estimator.state.velocity) < .005
    assert np.linalg.norm(estimator.state.position) < .03


def test_rejected_outlier_does_not_modify_state_or_covariance():
    estimator = RelativePoseFilter()
    estimator.predict(np.array([0, 0, 9.80665]), np.zeros(3), .05)
    before = deepcopy(estimator)
    assert not estimator.update_relative([100, 0, 0], np.eye(3), np.eye(6)*.001)
    np.testing.assert_array_equal(estimator.state.position, before.state.position)
    np.testing.assert_array_equal(estimator.covariance, before.covariance)


@pytest.mark.parametrize("moving", [False, True])
def test_quiet_initialization_uses_imu_and_rejects_motion(moving):
    timestamps = np.arange(401, dtype=np.int64)*5_000_000 + 10**18
    rotation = rotation_exp([.3, -.8, .2])
    samples = np.tile(np.r_[[.01, -.02, .03], rotation.T @ [0, 0, 9.80665]], (401, 1))
    if moving:
        samples[:, 3] += np.sin(np.arange(401))
        with pytest.raises(ValueError, match="no one-second quiet"):
            quiet_initialization((timestamps, samples))
        return
    state, report = quiet_initialization((timestamps, samples))
    np.testing.assert_allclose(state.rotation @ samples[0, 3:], [0, 0, 9.80665], atol=1e-12)
    np.testing.assert_allclose(state.gyro_bias, [.01, -.02, .03], atol=1e-12)
    assert report["end_ns"]-report["start_ns"] == 1_000_000_000
    assert report["ground_truth_used"] is False
