import numpy as np
import pytest

from vio.eskf import ErrorStateFilter, State, discretize, error_dynamics
from vio.geometry import rotation_exp, trajectory_metrics


def test_stationary_gravity_cancels_and_covariance_remains_psd():
    estimator = ErrorStateFilter()
    for _ in range(100):
        estimator.predict(np.array([0., 0., 9.80665]), np.zeros(3), 0.01)
    np.testing.assert_allclose(estimator.state.position, 0., atol=1e-12)
    np.testing.assert_allclose(estimator.covariance, estimator.covariance.T, atol=1e-12)
    assert np.linalg.eigvalsh(estimator.covariance).min() > 0


def test_constant_acceleration_has_analytic_solution():
    estimator = ErrorStateFilter()
    for _ in range(20):
        estimator.predict([2., 0., 9.80665], [0., 0., 0.], 0.05)
    np.testing.assert_allclose(estimator.state.position, [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(estimator.state.velocity, [2, 0, 0], atol=1e-12)


def test_rotation_integrates_body_rate_with_right_multiplication():
    estimator = ErrorStateFilter(State(rotation=rotation_exp([0.3, 0., 0.])))
    initial = estimator.state.rotation.copy()
    for _ in range(20):
        estimator.predict([0., 0., 0.], [0., 0., 0.5], 0.05)
    np.testing.assert_allclose(estimator.state.rotation, initial @ rotation_exp([0, 0, 0.5]), atol=1e-12)


def test_attitude_acceleration_jacobian_matches_finite_difference():
    state = State(rotation=rotation_exp([0.2, -0.3, 0.1]), accel_bias=np.array([0.1, 0.2, -0.1]))
    acceleration = np.array([1., 2., 9.])
    analytic = error_dynamics(state, acceleration, np.zeros(3))[3:6, 6:9]
    numerical = np.zeros((3, 3))
    for axis in range(3):
        perturbation = np.eye(3)[axis]*1e-6
        plus = state.rotation @ rotation_exp(perturbation) @ (acceleration-state.accel_bias)
        minus = state.rotation @ rotation_exp(-perturbation) @ (acceleration-state.accel_bias)
        numerical[:, axis] = (plus-minus)/2e-6
    np.testing.assert_allclose(analytic, numerical, atol=1e-8)


def test_van_loan_includes_position_velocity_cross_covariance():
    dynamics = np.zeros((15, 15))
    dynamics[0, 3] = 1
    noise = np.zeros((15, 15))
    noise[3, 3] = 2
    transition, covariance = discretize(dynamics, noise, 0.5)
    assert transition[0, 3] == pytest.approx(0.5)
    assert covariance[0, 0] == pytest.approx(2*0.5**3/3)
    assert covariance[0, 3] == pytest.approx(2*0.5**2/2)
    assert covariance[3, 3] == pytest.approx(1.)


def test_pose_correction_reduces_error_and_rejects_outlier_without_mutation():
    estimator = ErrorStateFilter(State(position=np.array([0.1, 0., 0.])))
    assert estimator.update_pose(np.zeros(3), np.eye(3), np.eye(6)*0.001)
    assert abs(estimator.state.position[0]) < 0.02
    before = estimator.state.position.copy(), estimator.covariance.copy()
    assert not estimator.update_pose(np.array([100., 0., 0.]), np.eye(3), np.eye(6)*0.001)
    np.testing.assert_array_equal(estimator.state.position, before[0])
    np.testing.assert_array_equal(estimator.covariance, before[1])


@pytest.mark.parametrize("dt", [0, -0.01, 0.101, float("nan")])
def test_invalid_timestep_is_rejected(dt):
    with pytest.raises(ValueError, match="time step"):
        ErrorStateFilter().predict([0., 0., 0.], [0., 0., 0.], dt)


@pytest.mark.parametrize("covariance", [np.eye(5), -np.eye(6), np.full((6, 6), np.nan)])
def test_invalid_measurement_covariance_is_rejected(covariance):
    with pytest.raises(ValueError, match="covariance"):
        ErrorStateFilter().update_pose(np.zeros(3), np.eye(3), covariance)


def test_trajectory_alignment_removes_rigid_offset_but_preserves_scale_error():
    truth = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 1.]])
    estimate = truth @ rotation_exp([0.1, 0.2, 0.3]).T + [2, 3, 4]
    assert trajectory_metrics(estimate, truth)["ate_se3_rmse_m"] < 1e-12
    assert trajectory_metrics(estimate*2, truth)["ate_se3_rmse_m"] > 0.5
