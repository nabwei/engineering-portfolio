"""Loose-coupled ESKF with a correlated previous-pose clone (21 errors)."""

import numpy as np

from .eskf import Noise, State, discretize, error_dynamics, process_covariance
from .geometry import rotation_exp, rotation_log, skew


def relative_observation(state, clone):
    translation = clone.rotation.T @ (state.position - clone.position)
    rotation = clone.rotation.T @ state.rotation
    jacobian = np.zeros((6, 21))
    jacobian[:3, :3] = clone.rotation.T
    jacobian[:3, 15:18] = -clone.rotation.T
    jacobian[:3, 18:21] = skew(translation)
    jacobian[3:, 6:9] = np.eye(3)
    jacobian[3:, 18:21] = -rotation.T
    return translation, rotation, jacobian


def inject_state(state, error):
    state.position += error[:3]
    state.velocity += error[3:6]
    state.rotation = state.rotation @ rotation_exp(error[6:9])
    state.accel_bias += error[9:12]
    state.gyro_bias += error[12:15]


class RelativePoseFilter:
    overlaps_frames = False
    fusion_model = "disjoint_relative_pose"

    def __init__(self, state=None, noise=None, initial_covariance=None):
        self.state = state if state is not None else State()
        self.noise = noise if noise is not None else Noise()
        self.covariance = np.zeros((21, 21))
        prior = (np.diag(np.repeat([0.01, 0.01, 0.01, 0.04, 0.001], 3))
                 if initial_covariance is None else np.asarray(initial_covariance))
        if (prior.shape != (15, 15) or not np.isfinite(prior).all()
                or not np.allclose(prior, prior.T) or np.linalg.eigvalsh(prior).min() < -1e-12):
            raise ValueError("initial covariance must be finite symmetric positive semidefinite 15x15")
        self.covariance[:15, :15] = prior
        self.last_nis = 0.0
        self.clone_current_pose()

    def clone_current_pose(self):
        self.clone = State(position=self.state.position, rotation=self.state.rotation)
        augmentation = np.zeros((21, 15))
        augmentation[:15, :] = np.eye(15)
        augmentation[15:18, :3] = np.eye(3)
        augmentation[18:21, 6:9] = np.eye(3)
        self.covariance = augmentation @ self.covariance[:15, :15] @ augmentation.T

    def predict(self, acceleration, angular_rate, dt):
        acceleration, angular_rate = np.asarray(acceleration), np.asarray(angular_rate)
        if not np.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("IMU time step must be in (0, 0.1] seconds")
        if acceleration.shape != (3,) or angular_rate.shape != (3,) or not np.isfinite([acceleration, angular_rate]).all():
            raise ValueError("IMU samples must be finite 3-vectors")
        midpoint = State(rotation=self.state.rotation @ rotation_exp((angular_rate-self.state.gyro_bias)*dt/2),
                         accel_bias=self.state.accel_bias, gyro_bias=self.state.gyro_bias)
        transition, noise = discretize(error_dynamics(midpoint, acceleration, angular_rate),
                                       process_covariance(midpoint, self.noise), dt)
        augmented = np.eye(21)
        augmented[:15, :15] = transition
        self.covariance = augmented @ self.covariance @ augmented.T
        self.covariance[:15, :15] += noise
        self.covariance = (self.covariance+self.covariance.T)/2
        world_accel = midpoint.rotation @ (acceleration-self.state.accel_bias) + [0, 0, -9.80665]
        self.state.position += self.state.velocity*dt + world_accel*dt**2/2
        self.state.velocity += world_accel*dt
        self.state.rotation = self.state.rotation @ rotation_exp((angular_rate-self.state.gyro_bias)*dt)

    def update_relative(self, translation, rotation, covariance):
        # State validates the externally supplied pose without changing this filter.
        measurement = State(position=translation, rotation=rotation)
        covariance = np.asarray(covariance)
        if covariance.shape != (6, 6) or not np.isfinite(covariance).all():
            raise ValueError("relative covariance must be finite 6x6")
        if not np.allclose(covariance, covariance.T) or np.linalg.eigvalsh(covariance).min() <= 0:
            raise ValueError("relative covariance must be symmetric positive definite")
        predicted_t, predicted_r, observation = relative_observation(self.state, self.clone)
        residual = np.r_[measurement.position-predicted_t, rotation_log(predicted_r.T @ measurement.rotation)]
        innovation = observation @ self.covariance @ observation.T + covariance
        self.last_nis = float(residual @ np.linalg.solve(innovation, residual))
        if self.last_nis > 22.458:
            return False
        self._correct(observation, residual, covariance)
        return True

    def _correct(self, observation, residual, covariance):
        innovation = observation @ self.covariance @ observation.T + covariance
        gain = np.linalg.solve(innovation, observation @ self.covariance).T
        error = gain @ residual
        remaining = np.eye(21) - gain @ observation
        posterior = remaining @ self.covariance @ remaining.T + gain @ covariance @ gain.T
        inject_state(self.state, error[:15])
        self.clone.position += error[15:18]
        self.clone.rotation = self.clone.rotation @ rotation_exp(error[18:21])
        reset = np.eye(21)
        reset[6:9, 6:9] -= skew(error[6:9])/2
        reset[18:21, 18:21] -= skew(error[18:21])/2
        self.covariance = reset @ posterior @ reset.T
        self.covariance = (self.covariance+self.covariance.T)/2
