"""Metric stereo/IMU moving-start alignment; no ground-truth or filter-reset input.

See DYNAMIC_INITIALIZATION_PROTOCOL.md for units, priors, gates and observability.
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from .eskf import State

GRAVITY = 9.80665
POSITION_SIGMA = .01
ACCEL_BIAS_SIGMA = 1.
GRAVITY_NORM_SIGMA = .01
GYRO_ROBUST_SCALE = .01


@dataclass(frozen=True)
class MovingWindow:
    times_ns: np.ndarray
    body_poses: np.ndarray
    imu_records: tuple


def _timestamps(values, name):
    values = np.asarray(values)
    if values.ndim != 1 or values.dtype.kind not in "iu" or len(values) < 2:
        raise ValueError(f"{name} requires integer nanosecond timestamps")
    if np.any(values < 0) or np.any(values > np.iinfo(np.int64).max):
        raise ValueError(f"{name} timestamps outside nonnegative int64 range")
    values = values.astype(np.int64, copy=True)
    if np.any(np.diff(values) <= 0):
        raise ValueError(f"{name} timestamps must strictly increase")
    return values


def _validate(window):
    if not isinstance(window.imu_records, (tuple, list)) or len(window.imu_records) != 2:
        raise ValueError("IMU records must contain timestamps and samples")
    times = _timestamps(window.times_ns, "visual")
    imu_times = _timestamps(window.imu_records[0], "IMU")
    poses = np.array(window.body_poses, dtype=float, copy=True)
    samples = np.array(window.imu_records[1], dtype=float, copy=True)
    if len(times) < 50 or int(times[-1])-int(times[0]) < 5_000_000_000:
        raise ValueError("moving initialization needs at least50 poses spanning at least5 seconds")
    if poses.shape != (len(times), 4, 4) or not np.isfinite(poses).all():
        raise ValueError("visual body poses must be finite Nx4x4 transforms")
    rotations = poses[:, :3, :3]
    if (not np.allclose(poses[:, 3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-6, rtol=0)
            or not np.allclose(np.linalg.det(rotations), 1, atol=1e-6, rtol=0)):
        raise ValueError("visual poses require proper rigid transforms")
    if samples.shape != (len(imu_times), 6) or not np.isfinite(samples).all():
        raise ValueError("IMU records must be finite Nx6 gyro-then-specific-force samples")
    if imu_times[0] > times[0] or imu_times[-1] < times[-1]:
        raise ValueError("IMU must bracket the complete visual window; no extrapolation")
    first = max(0, np.searchsorted(imu_times, times[0], side="right")-1)
    last = np.searchsorted(imu_times, times[-1], side="left")+1
    if np.max(np.diff(times)) > 250_000_000 or np.max(np.diff(imu_times[first:last])) > 20_000_000:
        raise ValueError("visual or IMU gap exceeds interpolation contract")
    return MovingWindow(times, poses, (imu_times[first:last], samples[first:last]))


def _segments(window):
    origin = window.times_ns[0]
    visual_time = (window.times_ns-origin)/1e9
    imu_ns, samples = window.imu_records
    imu_time = (imu_ns-origin)/1e9
    knots = np.unique(np.r_[visual_time, imu_time[(imu_time > 0) & (imu_time < visual_time[-1])]])
    dt = np.diff(knots)
    midpoint = (knots[:-1]+knots[1:])/2
    measurements = np.column_stack([np.interp(midpoint, imu_time, samples[:, axis]) for axis in range(6)])
    rotations = Slerp(visual_time, Rotation.from_matrix(window.body_poses[:, :3, :3]))(midpoint).as_matrix()
    return visual_time, knots, dt, measurements, rotations


def _gyro_alignment(window, timeline):
    times, knots, dt, samples, _ = timeline
    intervals = np.searchsorted(times, (knots[:-1]+knots[1:])/2, side="right")-1
    visual = window.body_poses[:, :3, :3]
    observed = visual[:-1].transpose(0, 2, 1) @ visual[1:]

    def residual(bias):
        increments = Rotation.from_rotvec((samples[:, :3]-bias)*dt[:, None]).as_matrix()
        integrated = np.repeat(np.eye(3)[None], len(observed), axis=0)
        for interval, increment in zip(intervals, increments):
            integrated[interval] = integrated[interval] @ increment
        return Rotation.from_matrix(integrated.transpose(0, 2, 1) @ observed).as_rotvec().ravel()

    solution = least_squares(residual, np.zeros(3), loss="soft_l1", f_scale=GYRO_ROBUST_SCALE,
                             xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=100)
    errors = np.linalg.norm(residual(solution.x).reshape(-1, 3), axis=1)
    rms = float(np.sqrt(np.mean(errors**2)))
    if not solution.success or not np.isfinite(solution.x).all() or rms > .03 or errors.max() > .15:
        raise ValueError("gyro/visual rotation fit rejected: residual or convergence gate")
    singular = np.linalg.svd(solution.jac, compute_uv=False)
    if np.linalg.matrix_rank(solution.jac) != 3:
        raise ValueError("gyro bias fit lacks three independent directions")
    return solution.x, {"rotation_rms_rad": rms, "rotation_max_rad": float(errors.max()),
                        "jacobian_singular_values": singular.tolist(), "rank": 3,
                        "bias_rad_s": solution.x.tolist(), "evaluations": solution.nfev}


def _force_integrals(timeline):
    times, knots, dt, samples, rotations = timeline
    force = np.einsum("nij,nj->ni", rotations, samples[:, 3:6])
    velocity_force = np.zeros(3)
    velocity_rotation = np.zeros((3, 3))
    integrated_force = [np.zeros(3)]
    integrated_rotation = [np.zeros((3, 3))]
    for step, rotation, acceleration in zip(dt, rotations, force):
        integrated_force.append(integrated_force[-1] + velocity_force*step + .5*acceleration*step**2)
        integrated_rotation.append(integrated_rotation[-1] + velocity_rotation*step + .5*rotation*step**2)
        velocity_force += acceleration*step
        velocity_rotation += rotation*step
    selected = np.searchsorted(knots, times)
    return (np.array(integrated_force)[selected], np.array(integrated_rotation)[selected],
            velocity_force, velocity_rotation)


def _translation_system(window, times, integrals):
    force, rotation = integrals[:2]
    design = np.concatenate([times[:, None, None]*np.eye(3),
                             .5*times[:, None, None]**2*np.eye(3), -rotation], axis=2)[1:].reshape(-1, 9)
    target = (window.body_poses[:, :3, 3]-window.body_poses[0, :3, 3]-force)[1:].ravel()
    column_norms = np.linalg.norm(design, axis=0)
    if np.any(column_norms < 1e-12):
        raise ValueError("insufficient time/pose geometry for alignment")
    normalized = design / column_norms
    singular = np.linalg.svd(normalized, compute_uv=False)
    numerical_rank = int(np.linalg.matrix_rank(normalized))
    practical_rank = int(np.sum(singular > singular[0]*1e-3))
    if numerical_rank < 6:
        raise ValueError("insufficient time/pose geometry for velocity and gravity alignment")
    observability = {"column_normalized_singular_values": singular.tolist(),
                     "numerical_rank": numerical_rank, "practical_rank_at_relative_1e_3": practical_rank,
                     "bias_identified_from_data": practical_rank == 9,
                     "mode": "data_constrained" if practical_rank == 9 else "prior_conditioned",
                     "warning": "Rank is a local design diagnostic, not covariance or proof of unbiased visual poses."}
    return design, target, observability


def _translation_alignment(design, target, estimate_bias):
    active = design if estimate_bias else design[:, :6]
    bias_prior = np.c_[np.zeros((3, 6)), np.eye(3)/ACCEL_BIAS_SIGMA] if estimate_bias else np.empty((0, 6))
    initial = np.linalg.lstsq(np.vstack([active/POSITION_SIGMA, bias_prior]),
                              np.r_[target/POSITION_SIGMA, np.zeros(len(bias_prior))], rcond=None)[0]

    def residual(unknown):
        return np.r_[(active @ unknown-target)/POSITION_SIGMA,
                     bias_prior @ unknown,
                     (np.linalg.norm(unknown[3:6])-GRAVITY)/GRAVITY_NORM_SIGMA]

    solution = least_squares(residual, initial, max_nfev=200, xtol=1e-10, ftol=1e-10, gtol=1e-10)
    unknown = solution.x if estimate_bias else np.r_[solution.x, np.zeros(3)]
    errors = np.linalg.norm((design @ unknown-target).reshape(-1, 3), axis=1)
    rms = float(np.sqrt(np.mean(errors**2)))
    gravity_error = float(abs(np.linalg.norm(solution.x[3:6])-GRAVITY))
    if (not solution.success or not np.isfinite(solution.x).all() or rms > .05
            or errors.max() > .2 or gravity_error > .05):
        raise ValueError("visual/IMU translation fit rejected: residual, gravity or convergence gate")
    return unknown, {"position_rms_m": rms, "position_max_m": float(errors.max()),
                        "gravity_norm_error_m_s2": gravity_error, "evaluations": solution.nfev}


def _endpoint_velocity(window, integrals, unknown):
    velocity0, gravity, accel_bias = unknown[:3], unknown[3:6], unknown[6:]
    duration = (int(window.times_ns[-1])-int(window.times_ns[0]))/1e9
    return velocity0 + gravity*duration + integrals[2] - integrals[3] @ accel_bias


def _gravity_alignment(gravity):
    return Rotation.align_vectors([[0., 0., -1.]], [gravity/np.linalg.norm(gravity)])[0].as_matrix()


def _endpoint_state(window, integrals, unknown, gyro_bias):
    velocity_end = _endpoint_velocity(window, integrals, unknown)
    alignment = _gravity_alignment(unknown[3:6])
    state = State(velocity=alignment @ velocity_end, rotation=alignment @ window.body_poses[-1, :3, :3],
                  accel_bias=unknown[6:], gyro_bias=gyro_bias)
    return state, alignment, velocity_end


def initialize_moving(window):
    """Return an endpoint State (new origin) and auditable fit/observability report."""
    window = _validate(window)
    timeline = _segments(window)
    gyro_bias, gyro_report = _gyro_alignment(window, timeline)
    integrals = _force_integrals(timeline)
    design, target, observability = _translation_system(window, timeline[0], integrals)
    estimate_bias = observability["bias_identified_from_data"]
    unknown, translation_report = _translation_alignment(design, target, estimate_bias)
    velocity0, gravity, accel_bias = unknown[:3], unknown[3:6], unknown[6:]
    duration = timeline[0][-1]
    state, alignment, velocity_end = _endpoint_state(window, integrals, unknown, gyro_bias)
    report = {"method": "independent metric stereo/IMU moving-window alignment",
              "start_ns": int(window.times_ns[0]), "end_ns": int(window.times_ns[-1]),
              "duration_s": float(duration), "visual_poses": len(window.times_ns),
              "imu_samples": len(window.imu_records[0]), "ground_truth_used": False,
              "gyro_fit": gyro_report, "translation_fit": translation_report, "observability": observability,
              "gravity_visual_m_s2": gravity.tolist(), "visual_to_gravity_rotation": alignment.tolist(),
              "initial_velocity_visual_m_s": velocity0.tolist(), "endpoint_velocity_visual_m_s": velocity_end.tolist(),
              "accel_bias_m_s2": accel_bias.tolist(),
              "accel_bias_policy": "estimate_with_weak_prior" if estimate_bias else "fixed_zero_prior_mean_due_to_rank",
              "uncertainty_contract": {"accel_bias_is_calibrated": False,
                                       "minimum_accel_bias_variance_m2_s4": ACCEL_BIAS_SIGMA**2,
                                       "unresolved_bias_tilt_scale_rad": ACCEL_BIAS_SIGMA/GRAVITY,
                                       "complete_state_covariance_available": False,
                                       "warning": "Do not initialize bias variance below the stated prior; gravity/bias coupling and visual uncertainty remain."},
              "assumptions": {"metric_stereo_scale_known": True, "camera_IMU_extrinsics_known": True,
                              "time_offset_known": True, "constant_body_frame_biases": True,
                              "position_sigma_m": POSITION_SIGMA, "zero_mean_accel_bias_prior_sigma_m_s2": ACCEL_BIAS_SIGMA,
                              "gravity_norm_m_s2": GRAVITY, "gravity_norm_sigma_m_s2": GRAVITY_NORM_SIGMA,
                              "yaw_and_global_position_unobservable": True, "endpoint_position_is_new_origin": True,
                              "full_initializer_covariance_recovered": False}}
    return state, report


def initialization_covariance(window, state):
    """Build a PSD engineering prior from conditional bias sensitivity, not a posterior."""
    window = _validate(window)
    timeline = _segments(window)
    integrals = _force_integrals(timeline)
    design, target, _ = _translation_system(window, timeline[0], integrals)

    def conditional(bias):
        # H=[F,-B]: F*x=target+B*bias when the bias is held fixed.
        unknown, _ = _translation_alignment(design, target-design[:, 6:] @ bias, False)
        unknown[6:] = bias
        return unknown

    baseline, baseline_alignment, _ = _endpoint_state(
        window, integrals, conditional(state.accel_bias), state.gyro_bias,
    )
    if (not np.allclose(baseline.velocity, state.velocity, atol=1e-5, rtol=0)
            or not np.allclose(baseline.rotation, state.rotation, atol=1e-5, rtol=0)):
        raise ValueError("initial state does not match the window's conditional endpoint fit")

    def perturbed_endpoint(bias):
        unknown = conditional(bias)
        # Keep the baseline yaw gauge: apply only the small gravity correction
        # in its already-aligned frame, not a fresh visual-frame minimum rotation.
        alignment = _gravity_alignment(baseline_alignment @ unknown[3:6]) @ baseline_alignment
        velocity = alignment @ _endpoint_velocity(window, integrals, unknown)
        rotation = alignment @ window.body_poses[-1, :3, :3]
        return velocity, rotation

    step = 1e-4  # m/s^2; central differences of the same constrained six-variable fit.
    sensitivity = np.zeros((15, 3))
    for axis in range(3):
        perturbation = np.eye(3)[axis]*step
        plus_velocity, plus_rotation = perturbed_endpoint(state.accel_bias+perturbation)
        minus_velocity, minus_rotation = perturbed_endpoint(state.accel_bias-perturbation)
        sensitivity[3:6, axis] = (plus_velocity-minus_velocity)/(2*step)
        plus_angle = Rotation.from_matrix(baseline.rotation.T @ plus_rotation).as_rotvec()
        minus_angle = Rotation.from_matrix(baseline.rotation.T @ minus_rotation).as_rotvec()
        sensitivity[6:9, axis] = (plus_angle-minus_angle)/(2*step)
    sensitivity[9:12] = np.eye(3)
    independent = np.repeat(np.square([.1, .5, .15, 0., .03]), 3)
    covariance = np.diag(independent) + ACCEL_BIAS_SIGMA**2*sensitivity @ sensitivity.T
    covariance = (covariance+covariance.T)/2
    report = {"policy": "PSD engineering prior from conditional bias sensitivity; not proven consistent",
              "block_order": ["position_world", "velocity_world", "right_attitude_body", "accel_bias_body", "gyro_bias_body"],
              "independent_diagonal_variances": independent.tolist(),
              "accel_bias_prior_variance_m2_s4": ACCEL_BIAS_SIGMA**2,
              "alignment_gauge": "minimal gravity perturbation in the baseline-aligned frame; fixed baseline yaw",
              "central_difference_step_m_s2": step, "conditional_bias_jacobian": sensitivity.tolist(),
              "diagonal_variances": np.diag(covariance).tolist(), "ground_truth_used": False,
              "limitations": "Visual pose and gyro-fit uncertainty enter broad independent allowances; their full joint posterior is not recovered."}
    return covariance, report
