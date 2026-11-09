import numpy as np
import pytest

from vio.eskf import ErrorStateFilter, State
from vio.recovery import RecoveryPolicy, RecoverySupervisor


COVARIANCE = np.diag([.05**2]*3+[.02**2]*3)


def displaced_filter():
    return ErrorStateFilter(State(position=np.array([5., 0, 0]), accel_bias=np.array([.12, .03, -.02])))


def test_sustained_rejection_recovers_observed_pose_and_fitted_velocity_without_truth():
    estimator, supervisor = displaced_filter(), RecoverySupervisor()
    bias = estimator.state.accel_bias.copy()
    velocity = np.array([.3, -.1, .04])
    for index in range(22):
        time = index*.05
        decision = supervisor.update(estimator, index*50_000_000, velocity*time, np.eye(3), COVARIANCE)
        if decision == "REINITIALIZED_DEGRADED":
            np.testing.assert_allclose(estimator.state.position, velocity*time, atol=1e-12)
            np.testing.assert_allclose(estimator.state.velocity, velocity, atol=1e-12)
    assert len(supervisor.events) == 1
    assert supervisor.events[0]["timestamp_ns"] == 1_000_000_000
    assert supervisor.events[0]["position_jump_m"] > 4
    np.testing.assert_array_equal(estimator.state.accel_bias, bias)
    assert np.linalg.eigvalsh(estimator.covariance).min() > 0


def test_isolated_gross_outlier_does_not_mutate_filter_or_trigger_recovery():
    estimator, supervisor = ErrorStateFilter(), RecoverySupervisor()
    supervisor.update(estimator, 0, np.zeros(3), np.eye(3), COVARIANCE)
    before = estimator.state.position.copy(), estimator.covariance.copy()
    assert supervisor.update(estimator, 50_000_000, np.array([100., 0, 0]), np.eye(3), COVARIANCE) == "INVALID_VISUAL_MOTION"
    np.testing.assert_array_equal(estimator.state.position, before[0])
    np.testing.assert_array_equal(estimator.covariance, before[1])
    assert supervisor.update(estimator, 100_000_000, np.zeros(3), np.eye(3), COVARIANCE) == "ACCEPTED"
    assert supervisor.events == []


def test_repeated_incoherent_outliers_do_not_regain_trust():
    estimator, supervisor = displaced_filter(), RecoverySupervisor()
    supervisor.update(estimator, 0, np.zeros(3), np.eye(3), COVARIANCE)
    for index in range(1, 41):
        decision = supervisor.update(estimator, index*50_000_000, np.array([100*(-1.)**index, 0, 0]), np.eye(3), COVARIANCE)
        assert decision == "INVALID_VISUAL_MOTION"
    assert supervisor.events == []
    np.testing.assert_allclose(estimator.state.position, [5, 0, 0])


def test_visual_dropout_requires_new_continuous_rejection_history():
    estimator, supervisor = displaced_filter(), RecoverySupervisor()
    for index in range(18):
        supervisor.update(estimator, index*50_000_000, np.zeros(3), np.eye(3), COVARIANCE)
    supervisor.missing()
    for index in range(25, 43):
        supervisor.update(estimator, index*50_000_000, np.zeros(3), np.eye(3), COVARIANCE)
    assert supervisor.events == []


def test_recovery_budget_cannot_silently_reset_forever():
    estimator = displaced_filter()
    supervisor = RecoverySupervisor(RecoveryPolicy(maximum_recoveries=1))
    for index in range(21):
        supervisor.update(estimator, index*50_000_000, np.zeros(3), np.eye(3), COVARIANCE)
    assert len(supervisor.events) == 1
    estimator.state.position = np.array([100., 0, 0])
    estimator.covariance = np.eye(15)*.001
    for index in range(21, 50):
        decision = supervisor.update(estimator, index*50_000_000, np.zeros(3), np.eye(3), COVARIANCE)
    assert decision == "RECOVERY_BUDGET_EXHAUSTED"
    assert len(supervisor.events) == 1
    np.testing.assert_allclose(estimator.state.position, [100, 0, 0])


@pytest.mark.parametrize("policy", [{"maximum_speed_m_s": 0}, {"history_seconds": float("nan")}, {"minimum_samples": 2.5}])
def test_invalid_recovery_policy_rejected(policy):
    with pytest.raises(ValueError):
        RecoveryPolicy(**policy)


def test_nonmonotonic_recovery_clock_rejected():
    supervisor, estimator = RecoverySupervisor(), ErrorStateFilter()
    supervisor.update(estimator, 100, np.zeros(3), np.eye(3), COVARIANCE)
    with pytest.raises(ValueError, match="timestamps"):
        supervisor.update(estimator, 100, np.zeros(3), np.eye(3), COVARIANCE)


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), 1.5])
def test_invalid_clock_does_not_poison_supervisor(invalid):
    supervisor, estimator = RecoverySupervisor(), ErrorStateFilter()
    supervisor.update(estimator, 0, np.zeros(3), np.eye(3), COVARIANCE)
    with pytest.raises(ValueError, match="integer nanoseconds"):
        supervisor.update(estimator, invalid, np.zeros(3), np.eye(3), COVARIANCE)
    assert supervisor.update(estimator, 50_000_000, np.zeros(3), np.eye(3), COVARIANCE) == "ACCEPTED"
