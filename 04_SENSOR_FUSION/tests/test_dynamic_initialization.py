"""Independent analytic rigid-body motion, without estimator/filter-generated truth."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vio.dynamic_initialization import GRAVITY, MovingWindow, initialization_covariance, initialize_moving
from vio.geometry import skew

EPOCH = 1_400_000_000_000_000_000
BIAS_ACCEL = np.array([.12, -.08, .05])
BIAS_GYRO = np.array([.012, -.018, .009])


def analytic_motion(time, excited):
    time = np.asarray(time)
    base_velocity = np.array([.9, -.35, .2])
    if not excited:
        rotations = np.repeat(np.eye(3)[None], len(time), axis=0)
        return rotations, np.zeros((len(time), 3)), time[:, None]*base_velocity, np.zeros((len(time), 3)), np.repeat(base_velocity[None], len(time), axis=0)
    x, y, z = .7*np.sin(.9*time)+.1*time, .55*np.sin(1.1*time+.4), .5*np.sin(.7*time-.3)+.2*time
    dx, dy, dz = .63*np.cos(.9*time)+.1, .605*np.cos(1.1*time+.4), .35*np.cos(.7*time-.3)+.2
    rotations = Rotation.from_euler("xyz", np.c_[x, y, z]).as_matrix()
    angular_rate = np.c_[dx-dz*np.sin(y), dy*np.cos(x)+dz*np.sin(x)*np.cos(y), -dy*np.sin(x)+dz*np.cos(x)*np.cos(y)]
    positions = time[:, None]*base_velocity + np.c_[.3*np.sin(.8*time), .25*(np.cos(1.1*time)-1), .2*np.sin(1.3*time)]
    accelerations = np.c_[-.192*np.sin(.8*time), -.3025*np.cos(1.1*time), -.338*np.sin(1.3*time)]
    velocities = base_velocity + np.c_[.24*np.cos(.8*time), -.275*np.sin(1.1*time), .26*np.cos(1.3*time)]
    return rotations, angular_rate, positions, accelerations, velocities


def synthetic_window(excited=True, world_rotation=None, off_grid=False):
    imu_time = np.arange(1001)*.005
    visual_time = np.arange(101)*.05
    if off_grid:
        visual_time[1:-1] += .0017  # Force actual IMU intervals to split at visual endpoints.
    visual_rotation = Rotation.from_rotvec([.6, -.3, .8]).as_matrix() if world_rotation is None else world_rotation
    rotations, angular_rate, _, acceleration, _ = analytic_motion(imu_time, excited)
    force = np.einsum("nji,nj->ni", rotations, acceleration-[0, 0, -GRAVITY]) + BIAS_ACCEL
    imu = np.c_[angular_rate+BIAS_GYRO, force]
    camera_rotations, _, positions, _, velocities = analytic_motion(visual_time, excited)
    poses = np.repeat(np.eye(4)[None], len(visual_time), axis=0)
    poses[:, :3, :3] = visual_rotation @ camera_rotations
    poses[:, :3, 3] = positions @ visual_rotation.T + [3, -2, 7]
    times = EPOCH + np.rint(visual_time*1e9).astype(np.int64)
    imu_times = EPOCH + np.rint(imu_time*1e9).astype(np.int64)
    return MovingWindow(times, poses, (imu_times, imu)), camera_rotations[-1], velocities[-1]


@pytest.mark.parametrize("off_grid", [False, True])
def test_excited_moving_start_recovers_bias_and_yaw_invariant_endpoint_state(off_grid):
    window, final_rotation, final_velocity = synthetic_window(off_grid=off_grid)
    original = window.body_poses.copy()
    state, report = initialize_moving(window)
    np.testing.assert_allclose(state.gyro_bias, BIAS_GYRO, atol=.002)
    np.testing.assert_allclose(state.accel_bias, BIAS_ACCEL, atol=.03)
    # Body-frame velocity/gravity do not depend on the arbitrary global yaw choice.
    np.testing.assert_allclose(state.rotation.T @ state.velocity, final_rotation.T @ final_velocity, atol=.025)
    np.testing.assert_allclose(state.rotation.T @ [0, 0, -GRAVITY], final_rotation.T @ [0, 0, -GRAVITY], atol=.035)
    np.testing.assert_array_equal(state.position, np.zeros(3))
    np.testing.assert_array_equal(window.body_poses, original)
    assert report["observability"]["bias_identified_from_data"]
    assert report["accel_bias_policy"] == "estimate_with_weak_prior"
    assert report["observability"]["practical_rank_at_relative_1e_3"] == 9
    assert report["ground_truth_used"] is False
    assert report["end_ns"] == EPOCH+5_000_000_000


def test_changing_arbitrary_visual_frame_preserves_body_velocity_and_bias():
    first, _, _ = synthetic_window(world_rotation=np.eye(3))
    second, _, _ = synthetic_window(world_rotation=Rotation.from_rotvec([1.2, -.7, .4]).as_matrix())
    a, _ = initialize_moving(first)
    b, _ = initialize_moving(second)
    np.testing.assert_allclose(a.rotation.T @ a.velocity, b.rotation.T @ b.velocity, atol=2e-6)
    np.testing.assert_allclose(a.accel_bias, b.accel_bias, atol=2e-6)
    np.testing.assert_allclose(a.gyro_bias, b.gyro_bias, atol=2e-6)


@pytest.mark.parametrize("radial_bias", [0., .05])
def test_constant_velocity_uses_prior_mean_or_rejects_incompatible_gravity(radial_bias):
    window, _, _ = synthetic_window(excited=False)
    window.imu_records[1][:, 5] += radial_bias-BIAS_ACCEL[2]
    # With no rotation, a radial bias can make the zero-prior model inconsistent
    # with gravity. Reject it rather than relax a gate or invent calibration.
    if radial_bias:
        with pytest.raises(ValueError, match="translation fit rejected"):
            initialize_moving(window)
        return
    state, report = initialize_moving(window)
    assert np.isfinite(state.velocity).all()
    assert report["observability"]["numerical_rank"] == 6
    assert report["observability"]["mode"] == "prior_conditioned"
    assert not report["observability"]["bias_identified_from_data"]
    assert report["accel_bias_policy"] == "fixed_zero_prior_mean_due_to_rank"
    np.testing.assert_array_equal(state.accel_bias, np.zeros(3))
    assert report["uncertainty_contract"]["minimum_accel_bias_variance_m2_s4"] == 1.
    assert not report["uncertainty_contract"]["accel_bias_is_calibrated"]
    assert report["assumptions"]["zero_mean_accel_bias_prior_sigma_m_s2"] == 1.
    # Do not falsely expect identifiable lateral bias from unrotated constant motion.
    assert np.linalg.norm(state.accel_bias-BIAS_ACCEL) > .05


@pytest.mark.parametrize("case", ["floating_time", "duplicate_time", "short_window", "missing_imu", "imu_gap", "bad_rotation", "nonfinite_force"])
def test_invalid_window_is_rejected_before_state_acceptance(case):
    window, _, _ = synthetic_window()
    times, poses = window.times_ns.copy(), window.body_poses.copy()
    imu_times, samples = (value.copy() for value in window.imu_records)
    if case == "floating_time":
        times = times.astype(float)
    elif case == "duplicate_time":
        times[20] = times[19]
    elif case == "short_window":
        times, poses = times[:50], poses[:50]
    elif case == "missing_imu":
        imu_times, samples = imu_times[1:], samples[1:]
    elif case == "imu_gap":
        imu_times, samples = np.delete(imu_times, np.arange(10, 20)), np.delete(samples, np.arange(10, 20), axis=0)
    elif case == "bad_rotation":
        poses[10, 0, 0] += .5
    elif case == "nonfinite_force":
        samples[20, 3] = np.nan
    with pytest.raises(ValueError):
        initialize_moving(MovingWindow(times, poses, (imu_times, samples)))


@pytest.mark.parametrize("case", ["visual_position_jump", "gyro_rotation_inconsistency"])
def test_inconsistent_visual_imu_fit_is_rejected(case):
    window, _, _ = synthetic_window()
    if case == "visual_position_jump":
        window.body_poses[50, :3, 3] += [1, -1, .5]
        message = "translation fit rejected"
    else:
        time = (window.imu_records[0]-EPOCH)/1e9
        window.imu_records[1][:, 0] += 3*np.sin(5*time)
        message = "rotation fit rejected"
    with pytest.raises(ValueError, match=message):
        initialize_moving(window)


@pytest.mark.parametrize("visual_tilt_degrees", [0., 110.])
@pytest.mark.parametrize("body_rotvec", [(0., 0., 0.), (.45, -.35, .2)])
def test_covariance_cross_terms_match_analytic_bias_gravity_coupling_in_arbitrary_frame(visual_tilt_degrees, body_rotvec):
    # Fable review regression: a large arbitrary visual-frame tilt must not
    # introduce world-yaw/bias coupling into the fixed baseline gauge.
    visual_rotation = Rotation.from_euler("x", visual_tilt_degrees, degrees=True).as_matrix()
    window, _, _ = synthetic_window(excited=False, world_rotation=visual_rotation)
    body_rotation = Rotation.from_rotvec(body_rotvec).as_matrix()
    window.body_poses[:, :3, :3] = visual_rotation @ body_rotation
    # Constant body attitude and velocity: the measured specific force is -R^T*g.
    window.imu_records[1][:, 3:6] = body_rotation.T @ [0., 0., GRAVITY]
    state, _ = initialize_moving(window)
    covariance, report = initialization_covariance(window, state)
    gravity_world = np.array([0., 0., -GRAVITY])
    gravity_body = body_rotation.T @ gravity_world
    expected_velocity = np.array([.9, -.35, .2])
    np.testing.assert_allclose(state.rotation, body_rotation, atol=1e-8)
    np.testing.assert_allclose(state.velocity, expected_velocity, atol=1e-8)
    tilt_bias = -skew(gravity_body)/GRAVITY**2  # Right-local/body attitude error.
    world_tilt_bias = -skew(gravity_world) @ body_rotation / GRAVITY**2
    velocity_bias = -skew(expected_velocity) @ world_tilt_bias
    # The radial response is a two-variable scalar fit: velocity and gravity.
    time = (window.times_ns[1:]-window.times_ns[0])/1e9
    design = np.c_[time, .5*time**2]
    normal = design.T @ design/.01**2 + np.diag([0., 1/.01**2])
    response = np.linalg.solve(normal, design.T @ (.5*time**2)/.01**2)
    radial_response = response[0]+5*response[1]-5
    velocity_bias += radial_response*np.outer(gravity_world, gravity_body)/GRAVITY**2
    np.testing.assert_allclose(covariance[6:9, 9:12], tilt_bias, atol=2e-6)
    np.testing.assert_allclose(covariance[3:6, 9:12], velocity_bias, atol=2e-5)
    np.testing.assert_allclose(covariance[9:12, 9:12], np.eye(3), atol=1e-12)
    assert np.linalg.eigvalsh(covariance).min() > 0
    assert report["ground_truth_used"] is False


def test_full_rank_covariance_preserves_state_and_rejects_a_mismatched_endpoint():
    window, _, _ = synthetic_window()
    state, _ = initialize_moving(window)
    original = {name: value.copy() for name, value in vars(state).items()}
    covariance, report = initialization_covariance(window, state)
    assert covariance.shape == (15, 15)
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
    assert np.linalg.eigvalsh(covariance).min() > 0
    assert np.diag(covariance)[9:12].min() >= 1.
    assert np.diag(covariance)[3:6].min() >= .5**2
    assert np.diag(covariance)[6:9].min() >= .15**2
    assert np.linalg.norm(np.array(report["conditional_bias_jacobian"])[6:9]) > .01
    for name, value in vars(state).items():
        np.testing.assert_array_equal(value, original[name])
    state.velocity[0] += 1.
    with pytest.raises(ValueError, match="does not match"):
        initialization_covariance(window, state)
