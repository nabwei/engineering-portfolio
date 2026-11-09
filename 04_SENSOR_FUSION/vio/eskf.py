from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import expm

from .geometry import rotation_exp, rotation_log, skew


@dataclass
class Noise:
    accel_density: float = 0.03
    gyro_density: float = 0.002
    accel_bias_walk: float = 0.0003
    gyro_bias_walk: float = 0.00002

    def __post_init__(self):
        densities = np.array(list(vars(self).values()))
        if not np.isfinite(densities).all() or (densities < 0).any():
            raise ValueError("noise densities must be finite and nonnegative")


@dataclass
class State:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        for name in ["position", "velocity", "accel_bias", "gyro_bias"]:
            vector = np.array(getattr(self, name), dtype=float, copy=True)
            if vector.shape != (3,) or not np.isfinite(vector).all():
                raise ValueError(f"state {name} must be a finite 3-vector")
            setattr(self, name, vector)
        self.rotation = np.array(self.rotation, dtype=float, copy=True)
        if self.rotation.shape != (3, 3) or not np.isfinite(self.rotation).all():
            raise ValueError("state rotation must be finite 3x3")
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-7, rtol=0) or not np.isclose(np.linalg.det(self.rotation), 1):
            raise ValueError("state rotation must be proper orthonormal")


def error_dynamics(state, acceleration, angular_rate):
    dynamics = np.zeros((15, 15))
    dynamics[:3, 3:6] = np.eye(3)
    dynamics[3:6, 6:9] = -state.rotation @ skew(acceleration - state.accel_bias)
    dynamics[3:6, 9:12] = -state.rotation
    dynamics[6:9, 6:9] = -skew(angular_rate - state.gyro_bias)
    dynamics[6:9, 12:15] = -np.eye(3)
    return dynamics


def process_covariance(state, noise):
    injection = np.zeros((15, 12))
    injection[3:6, :3] = -state.rotation
    injection[6:9, 3:6] = -np.eye(3)
    injection[9:12, 6:9] = np.eye(3)
    injection[12:15, 9:12] = np.eye(3)
    density = np.repeat([noise.accel_density, noise.gyro_density,
                         noise.accel_bias_walk, noise.gyro_bias_walk], 3)
    return (injection * density**2) @ injection.T


def discretize(dynamics, spectral_covariance, dt):
    """Van Loan exact discretization of the frozen continuous linear model."""
    block = np.zeros((30, 30))
    block[:15, :15] = dynamics
    block[:15, 15:] = spectral_covariance
    block[15:, 15:] = -dynamics.T
    exponential = expm(block * dt)
    transition = exponential[:15, :15]
    covariance = exponential[:15, 15:] @ transition.T
    return transition, (covariance + covariance.T) / 2


class ErrorStateFilter:
    def __init__(self, state=None, noise=None):
        self.state = state if state is not None else State()
        self.noise = noise if noise is not None else Noise()
        self.covariance = np.diag(np.repeat([0.01, 0.1, 0.01, 0.01, 0.001], 3))

    def predict(self, acceleration, angular_rate, dt):
        if not np.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("IMU time step must be in (0, 0.1] seconds")
        acceleration, angular_rate = np.asarray(acceleration), np.asarray(angular_rate)
        if acceleration.shape != (3,) or angular_rate.shape != (3,):
            raise ValueError("IMU samples must be 3-vectors")
        if not np.isfinite([acceleration, angular_rate]).all():
            raise ValueError("IMU samples must be finite")
        dynamics = error_dynamics(self.state, acceleration, angular_rate)
        transition, discrete_noise = discretize(
            dynamics, process_covariance(self.state, self.noise), dt)
        self.covariance = transition @ self.covariance @ transition.T + discrete_noise
        self.covariance = (self.covariance + self.covariance.T) / 2
        self._integrate(np.asarray(acceleration), np.asarray(angular_rate), dt)

    def _integrate(self, acceleration, angular_rate, dt):
        state = self.state
        world_accel = state.rotation @ (acceleration-state.accel_bias) + [0, 0, -9.80665]
        state.position = state.position + state.velocity*dt + 0.5*world_accel*dt**2
        state.velocity = state.velocity + world_accel*dt
        state.rotation = state.rotation @ rotation_exp((angular_rate-state.gyro_bias)*dt)

    def update_pose(self, position, rotation, measurement_covariance):
        """Loose-coupled pose update; correlated VO drift is not modeled."""
        position, rotation = np.asarray(position), np.asarray(rotation)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("pose position must be a finite 3-vector")
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError("pose rotation must be finite 3x3")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(rotation), 1):
            raise ValueError("pose rotation must be proper orthonormal")
        observation = np.zeros((6, 15))
        observation[:3, :3] = np.eye(3)
        observation[3:, 6:9] = np.eye(3)
        residual = np.r_[position-self.state.position,
                         rotation_log(self.state.rotation.T @ rotation)]
        covariance = np.asarray(measurement_covariance)
        if covariance.shape != (6, 6) or not np.isfinite(covariance).all():
            raise ValueError("pose covariance must be finite 6x6")
        if not np.allclose(covariance, covariance.T) or np.linalg.eigvalsh(covariance).min() <= 0:
            raise ValueError("pose covariance must be symmetric positive definite")
        innovation = observation @ self.covariance @ observation.T + covariance
        normalized_error = float(residual @ np.linalg.solve(innovation, residual))
        if normalized_error > 22.458:
            return False
        self._correct(observation, residual, covariance, innovation)
        return True

    def _correct(self, observation, residual, covariance, innovation):
        gain = np.linalg.solve(innovation, observation @ self.covariance).T
        error = gain @ residual
        remaining = np.eye(15) - gain @ observation
        posterior = remaining @ self.covariance @ remaining.T + gain @ covariance @ gain.T
        self._inject(error)
        reset = np.eye(15)
        reset[6:9, 6:9] -= 0.5 * skew(error[6:9])
        self.covariance = reset @ posterior @ reset.T
        self.covariance = (self.covariance + self.covariance.T)/2

    def _inject(self, error):
        self.state.position += error[:3]
        self.state.velocity += error[3:6]
        self.state.rotation = self.state.rotation @ rotation_exp(error[6:9])
        self.state.accel_bias += error[9:12]
        self.state.gyro_bias += error[12:15]
