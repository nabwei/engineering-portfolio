"""Adjacent relative updates with an explicit independent noisy-image-pose model.

Persistent errors: physical15 + noisy-image-pose clone6. A fresh image error6 is
temporarily appended for correction, then marginalized into the next noisy clone.
This is a pose-level approximation to PnP, not a feature-level consistency claim.
"""

import numpy as np

from .eskf import State
from .geometry import rotation_exp, rotation_log, skew
from .relative import RelativePoseFilter, inject_state, relative_observation

BETA = .5


def right_jacobian(vector):
    angle = np.linalg.norm(vector)
    cross = skew(vector)
    if angle < 1e-5:
        return np.eye(3)-cross/2+cross @ cross/6
    return np.eye(3)-(1-np.cos(angle))/angle**2*cross+(angle-np.sin(angle))/angle**3*(cross @ cross)


def noisy_image_pose(state, image_error):
    return State(position=state.position+state.rotation @ image_error[:3],
                 rotation=state.rotation @ rotation_exp(image_error[3:]))


def image_clone_jacobian(state, image_error):
    # Input errors are physical15 plus additive body-frame image error6.
    augmentation = np.zeros((21, 21))
    augmentation[:15, :15] = np.eye(15)
    augmentation[15:18, :3] = np.eye(3)
    augmentation[15:18, 6:9] = -state.rotation @ skew(image_error[:3])
    augmentation[15:18, 15:18] = state.rotation
    augmentation[18:21, 6:9] = rotation_exp(image_error[3:]).T
    augmentation[18:21, 18:21] = right_jacobian(image_error[3:])
    return augmentation


def covariance_matrix(value):
    matrix = np.array(value, dtype=float, copy=True)
    if matrix.shape != (6, 6) or not np.isfinite(matrix).all():
        raise ValueError("pair covariance must be finite 6x6")
    if not np.allclose(matrix, matrix.T) or np.linalg.eigvalsh(matrix).min() <= 0:
        raise ValueError("pair covariance must be symmetric positive definite")
    return matrix


def image_observation(state, clone):
    translation, rotation, physical = relative_observation(state, clone)
    observation = np.zeros((6, 27))
    observation[:, :21] = physical
    observation[:3, 21:24], observation[3:, 24:27] = rotation, np.eye(3)
    return translation, rotation, observation


class NoisyFramePoseFilter(RelativePoseFilter):
    overlaps_frames = True
    fusion_model = "noisy_frame_pose"
    frame_noise_beta = BETA

    def __init__(self, state=None, noise=None, initial_covariance=None, pair_covariance=None):
        if pair_covariance is None:
            pair_covariance = np.diag(np.repeat(np.square([.01, .005]), 3))
        self.pair_covariance = covariance_matrix(pair_covariance)
        self.frame_covariance = BETA*self.pair_covariance/2
        self.residual_covariance = (1-BETA)*self.pair_covariance
        super().__init__(state, noise, initial_covariance)

    def _retain_image(self, joint, image_error):
        self.clone = noisy_image_pose(self.state, image_error)
        augmentation = image_clone_jacobian(self.state, image_error)
        self.covariance = augmentation @ joint @ augmentation.T
        self.covariance = (self.covariance+self.covariance.T)/2

    def clone_current_pose(self):
        joint = np.zeros((21, 21))
        joint[:15, :15] = self.covariance[:15, :15]
        joint[15:, 15:] = self.frame_covariance
        self._retain_image(joint, np.zeros(6))

    def update_relative(self, translation, rotation, covariance):
        supplied = covariance_matrix(covariance)
        if not np.array_equal(supplied, self.pair_covariance):
            raise ValueError("observation covariance differs from frozen pair noise budget")
        measurement = State(position=translation, rotation=rotation)
        predicted_t, predicted_r, observation = image_observation(self.state, self.clone)
        prior = np.zeros((27, 27))
        prior[:21, :21], prior[21:, 21:] = self.covariance, self.frame_covariance
        residual = np.r_[measurement.position-predicted_t, rotation_log(predicted_r.T @ measurement.rotation)]
        innovation = observation @ prior @ observation.T + self.residual_covariance
        self.last_nis = float(residual @ np.linalg.solve(innovation, residual))
        if self.last_nis > 22.458:
            self.clone_current_pose()
            return False
        self._correct_noisy_frame(prior, observation, residual)
        return True

    def _correct_noisy_frame(self, prior, observation, residual):
        innovation = observation @ prior @ observation.T + self.residual_covariance
        gain = np.linalg.solve(innovation, observation @ prior).T
        error = gain @ residual
        remaining = np.eye(27)-gain @ observation
        posterior = remaining @ prior @ remaining.T + gain @ self.residual_covariance @ gain.T
        inject_state(self.state, error[:15])
        reset = np.eye(27)
        reset[6:9, 6:9] -= skew(error[6:9])/2
        reset[18:21, 18:21] -= skew(error[18:21])/2
        posterior = reset @ posterior @ reset.T
        keep = np.r_[np.arange(15), np.arange(21, 27)]
        self._retain_image(posterior[np.ix_(keep, keep)], error[21:27])
