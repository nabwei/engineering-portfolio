"""Bounded operational fallback; this does not make correlated VO fusion consistent."""

from collections import deque
from dataclasses import asdict, dataclass
from numbers import Integral

import numpy as np

from .eskf import State
from .geometry import rotation_log


@dataclass(frozen=True)
class RecoveryPolicy:
    rejection_seconds: float = 1.0
    history_seconds: float = .5
    maximum_gap_seconds: float = .25
    maximum_speed_m_s: float = 10.
    maximum_rate_rad_s: float = 10.
    maximum_fit_residual_m: float = .25
    minimum_samples: int = 5
    maximum_recoveries: int = 3

    def __post_init__(self):
        values = np.asarray(list(asdict(self).values()))
        if not np.isfinite(values).all() or (values <= 0).any():
            raise ValueError("recovery policy bounds must be finite and positive")
        if self.minimum_samples < 2 or int(self.minimum_samples) != self.minimum_samples or int(self.maximum_recoveries) != self.maximum_recoveries:
            raise ValueError("recovery sample and event limits must be integers")


class RecoverySupervisor:
    def __init__(self, policy=None):
        self.policy = policy or RecoveryPolicy()
        self.history = deque()
        self.last_visual = None
        self.last_timestamp = None
        self.rejected_since = None
        self.events = []

    def missing(self):
        self.history.clear()
        self.rejected_since = None

    def update(self, estimator, timestamp_ns, position, rotation, covariance):
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, Integral):
            raise ValueError("recovery timestamp must be integer nanoseconds")
        # Validate the pose before it can enter the history or mutate the filter.
        checked = State(position=position, rotation=rotation)
        if self.last_timestamp is not None and timestamp_ns <= self.last_timestamp:
            raise ValueError("recovery timestamps must increase")
        self.last_timestamp = timestamp_ns
        if not self._coherent(timestamp_ns, checked):
            self.missing()
            return "INVALID_VISUAL_MOTION"
        self.last_visual = (timestamp_ns, checked)
        self.history.append((timestamp_ns, checked.position))
        cutoff = timestamp_ns-int(self.policy.history_seconds*1e9)
        # Retain one sample at/before the boundary, keeping the fit span sufficient.
        while len(self.history) > 2 and self.history[1][0] <= cutoff:
            self.history.popleft()
        if estimator.update_pose(checked.position, checked.rotation, covariance):
            self.rejected_since = None
            return "ACCEPTED"
        if self.rejected_since is None:
            self.rejected_since = timestamp_ns
        if (timestamp_ns-self.rejected_since)*1e-9 < self.policy.rejection_seconds:
            return "REJECTED"
        if len(self.events) >= self.policy.maximum_recoveries:
            return "RECOVERY_BUDGET_EXHAUSTED"
        velocity = self._fit_velocity()
        if velocity is None:
            return "INSUFFICIENT_RECOVERY_HISTORY"
        self._reset(estimator, timestamp_ns, checked, velocity)
        self.rejected_since = None
        return "REINITIALIZED_DEGRADED"

    def _coherent(self, timestamp, pose):
        if self.last_visual is None:
            return True
        last_time, last_pose = self.last_visual
        dt = (timestamp-last_time)*1e-9
        if dt > self.policy.maximum_gap_seconds:
            self.missing()
        speed = np.linalg.norm(pose.position-last_pose.position)/dt
        rate = np.linalg.norm(rotation_log(last_pose.rotation.T @ pose.rotation))/dt
        return speed <= self.policy.maximum_speed_m_s and rate <= self.policy.maximum_rate_rad_s

    def _fit_velocity(self):
        if len(self.history) < self.policy.minimum_samples:
            return None
        times = np.array([(time-self.history[0][0])*1e-9 for time, _ in self.history])
        if times[-1] < self.policy.history_seconds-1e-9:
            return None
        centered = times-times.mean()
        positions = np.array([position for _, position in self.history])
        velocity = centered @ positions / (centered @ centered)
        residual = positions-(positions.mean(axis=0)+centered[:, None]*velocity)
        if np.linalg.norm(velocity) > self.policy.maximum_speed_m_s or np.linalg.norm(residual, axis=1).max() > self.policy.maximum_fit_residual_m:
            return None
        return velocity

    def _reset(self, estimator, timestamp, pose, velocity):
        previous = estimator.state
        event = {"timestamp_ns": int(timestamp), "mode": "REINITIALIZED_DEGRADED",
                 "position_jump_m": float(np.linalg.norm(previous.position-pose.position)),
                 "attitude_jump_rad": float(np.linalg.norm(rotation_log(previous.rotation.T @ pose.rotation))),
                 "velocity_jump_m_s": float(np.linalg.norm(previous.velocity-velocity)),
                 "retained_biases": True}
        estimator.state = State(pose.position, velocity, pose.rotation, previous.accel_bias, previous.gyro_bias)
        # Marginal variance floors for an uncertain reset; no statistical consistency claim.
        floors = np.repeat([.5, 1., .2, .2, .05], 3)**2
        estimator.covariance = np.diag(np.maximum(np.diag(estimator.covariance), floors))
        self.events.append(event)
