from copy import deepcopy
from functools import partial

import numpy as np
import pytest

from vio.eskf import Noise, State
from vio.geometry import rotation_exp, rotation_log
from vio.noisy_frame import BETA, NoisyFramePoseFilter, image_clone_jacobian, image_observation, noisy_image_pose
from vio.relative import inject_state


def test_noisy_pose_clone_jacobian_matches_central_perturbations_at_nonzero_noise_mean():
    state = State(position=[.3, -.2, .1], rotation=rotation_exp([.3, -.4, .2]))
    error = np.array([.03, -.02, .01, .09, -.06, .04])
    reference = noisy_image_pose(state, error)
    analytical = image_clone_jacobian(state, error)[15:]
    numerical = np.zeros_like(analytical)
    for column in range(21):
        outputs = []
        for sign in [-1, 1]:
            changed = deepcopy(state)
            perturbation = np.zeros(21)
            perturbation[column] = sign*1e-6
            inject_state(changed, perturbation[:15])
            pose = noisy_image_pose(changed, error+perturbation[15:])
            outputs.append(np.r_[pose.position-reference.position, rotation_log(reference.rotation.T @ pose.rotation)])
        numerical[:, column] = (outputs[1]-outputs[0])/2e-6
    np.testing.assert_allclose(analytical, numerical, atol=1e-9, rtol=1e-6)


def test_adjacent_image_noise_filter_matches_independent_linear_gaussian_batch_posterior():
    # Unknowns in the batch solution: initial position, constant velocity, and
    # independent scalar error of each image. Each delta shares one image error.
    count, dt, pair_variance = 6, .05, .04
    covariance = np.diag([pair_variance]*3+[.01]*3)
    estimator = NoisyFramePoseFilter(noise=Noise(0, 0, 0, 0), pair_covariance=covariance)
    estimator.covariance[:] = 0
    estimator.covariance[0, 0], estimator.covariance[3, 3] = 1.2, .5
    estimator.clone_current_pose()
    measurement = np.array([.02, .008, .014, .003, .019, .007])
    prior = np.diag([1.2, .5]+[BETA*pair_variance/2]*(count+1))
    observation = np.zeros((count, count+3))
    for index in range(count):
        observation[index, 1] = dt
        observation[index, index+2:index+4] = [-1, 1]
        estimator.predict(np.array([0, 0, 9.80665]), np.zeros(3), dt)
        assert estimator.update_relative([measurement[index], 0, 0], np.eye(3), covariance)
    innovation = observation @ prior @ observation.T + np.eye(count)*(1-BETA)*pair_variance
    gain = np.linalg.solve(innovation, observation @ prior).T
    mean, posterior = gain @ measurement, prior-gain @ observation @ prior
    output = np.zeros((3, count+3))
    output[0, :2], output[1, 1] = [1, count*dt], 1
    output[2] = output[0]
    output[2, -1] = 1
    actual = [estimator.state.position[0], estimator.state.velocity[0], estimator.clone.position[0]]
    np.testing.assert_allclose(actual, output @ mean, atol=1e-11, rtol=1e-10)
    np.testing.assert_allclose(estimator.covariance[np.ix_([0, 3, 15], [0, 3, 15])], output @ posterior @ output.T, atol=1e-11)
    assert np.linalg.eigvalsh(estimator.covariance).min() > -1e-11


def test_augmented_observation_jacobian_matches_independent_noisy_pose_geometry():
    state = State(position=[2., -.5, .7], rotation=rotation_exp([.4, -.3, .2]))
    clone = State(position=[.3, .1, -.2], rotation=rotation_exp([-.2, .1, .4]))
    _, reference_rotation, analytical = image_observation(state, clone)
    numerical = np.zeros((6, 27))
    for column in range(27):
        values = []
        for sign in [-1, 1]:
            current, previous = deepcopy(state), deepcopy(clone)
            perturbation = np.zeros(27)
            perturbation[column] = sign*1e-6
            inject_state(current, perturbation[:15])
            previous.position += perturbation[15:18]
            previous.rotation = previous.rotation @ rotation_exp(perturbation[18:21])
            noisy = noisy_image_pose(current, perturbation[21:])
            translation = previous.rotation.T @ (noisy.position-previous.position)
            rotation = previous.rotation.T @ noisy.rotation
            values.append(np.r_[translation, rotation_log(reference_rotation.T @ rotation)])
        numerical[:, column] = (values[1]-values[0])/2e-6
    np.testing.assert_allclose(analytical, numerical, atol=1e-9, rtol=1e-6)


def test_stationary_image_noise_budget_matches_original_pair_variance():
    covariance = np.diag([.01]*3+[.004]*3)
    estimator = NoisyFramePoseFilter(pair_covariance=covariance)
    estimator.covariance[:] = 0
    estimator.clone_current_pose()
    residual = np.array([.01, -.02, .015, .003, -.002, .001])
    assert estimator.update_relative(residual[:3], rotation_exp(residual[3:]), covariance)
    np.testing.assert_allclose(estimator.last_nis, residual @ np.linalg.solve(covariance, residual), rtol=1e-12)


def test_rejected_relative_pose_preserves_physical_state_and_advances_uncertain_image_reference():
    estimator = NoisyFramePoseFilter()
    estimator.predict(np.array([.1, 0, 9.80665]), np.zeros(3), .05)
    before = deepcopy(estimator)
    assert not estimator.update_relative([100, 0, 0], np.eye(3), estimator.pair_covariance)
    np.testing.assert_array_equal(estimator.state.position, before.state.position)
    np.testing.assert_array_equal(estimator.state.velocity, before.state.velocity)
    np.testing.assert_array_equal(estimator.covariance[:15, :15], before.covariance[:15, :15])
    np.testing.assert_array_equal(estimator.clone.position, estimator.state.position)
    np.testing.assert_allclose(estimator.covariance[15:18, 15:18]-estimator.covariance[:3, :3], estimator.frame_covariance[:3, :3])
    estimator.predict(np.array([0, 0, 9.80665]), np.zeros(3), .05)
    assert estimator.update_relative([.00025, 0, 0], np.eye(3), estimator.pair_covariance)


def test_changed_pair_budget_is_rejected_without_mutation():
    estimator = NoisyFramePoseFilter()
    before = estimator.covariance.copy()
    with pytest.raises(ValueError, match="frozen pair noise budget"):
        estimator.update_relative([0, 0, 0], np.eye(3), estimator.pair_covariance*2)
    np.testing.assert_array_equal(estimator.covariance, before)


def test_coupled_initializer_covariance_is_preserved_with_independent_image_noise():
    random = np.random.default_rng(417)
    factor = random.normal(size=(15, 15))*.01
    prior = factor @ factor.T
    state = State(rotation=rotation_exp([.2, -.4, .1]))
    estimator = NoisyFramePoseFilter(state, None, prior)
    pose_indices = [0, 1, 2, 6, 7, 8]
    np.testing.assert_array_equal(estimator.covariance[:15, :15], prior)
    np.testing.assert_array_equal(estimator.covariance[:15, 15:], prior[:, pose_indices])
    mapping = np.eye(6)
    mapping[:3, :3] = state.rotation
    expected = prior[np.ix_(pose_indices, pose_indices)] + mapping @ estimator.frame_covariance @ mapping.T
    np.testing.assert_allclose(estimator.covariance[15:, 15:], expected, atol=1e-15)


def central_jacobian(function, center):
    columns = []
    for direction in np.eye(len(center))*1e-6:
        columns.append((function(center+direction)-function(center-direction))/2e-6)
    return np.column_stack(columns)


def perturbed_image_geometry(state, clone, error):
    rotation = state.rotation @ rotation_exp(error[6:9])
    image_position = state.position+error[:3]+rotation @ error[21:24]
    image_rotation = rotation @ rotation_exp(error[24:27])
    previous_rotation = clone.rotation @ rotation_exp(error[18:21])
    translation = previous_rotation.T @ (image_position-clone.position-error[15:18])
    relative_rotation = previous_rotation.T @ image_rotation
    nominal_rotation = clone.rotation.T @ state.rotation
    return np.r_[translation, rotation_log(nominal_rotation.T @ relative_rotation)]


def retained_coordinates(state, error, center):
    rotation = state.rotation @ rotation_exp(error[6:9])
    centered_rotation = state.rotation @ rotation_exp(center[6:9])
    physical = error[:15]-center[:15]
    physical[6:9] = rotation_log(centered_rotation.T @ rotation)
    image_position = error[:3]+rotation @ error[21:24]
    centered_position = center[:3]+centered_rotation @ center[21:24]
    image_rotation = rotation @ rotation_exp(error[24:27])
    centered_image_rotation = centered_rotation @ rotation_exp(center[24:27])
    return np.r_[physical, image_position-centered_position,
                 rotation_log(centered_image_rotation.T @ image_rotation)]


def test_turning_multiupdate_matches_numerical_geometry_information_posterior():
    # Independent geometry and information conditioning exercise finite motion,
    # shared-frame correlations and nonzero fresh-image means across updates.
    state = State(position=[.3, -.2, .4], velocity=[.8, -.4, .2],
                  rotation=rotation_exp([.4, -.3, .2]))
    estimator = NoisyFramePoseFilter(state)
    residual = np.array([.03, -.02, .01, .02, -.01, .015])*1e-3
    for step in range(4):
        estimator.predict([.4, -.3, 9.7], [.3, -.2, .4], .08)
        before = deepcopy(estimator.state)
        prior = np.zeros((27, 27))
        prior[:21, :21], prior[21:, 21:] = estimator.covariance, estimator.frame_covariance
        geometry = partial(perturbed_image_geometry, before, estimator.clone)
        jacobian = central_jacobian(geometry, np.zeros(27))
        information = np.linalg.inv(prior) + jacobian.T @ np.linalg.solve(estimator.residual_covariance, jacobian)
        posterior = np.linalg.inv(information)
        increment = residual*(step+1)
        mean = posterior @ jacobian.T @ np.linalg.solve(estimator.residual_covariance, increment)
        mapping = central_jacobian(lambda error: retained_coordinates(before, error, mean), mean)
        expected_covariance = mapping @ posterior @ mapping.T
        translation = geometry(np.zeros(27))[:3]+increment[:3]
        rotation = estimator.clone.rotation.T @ before.rotation @ rotation_exp(increment[3:])
        assert estimator.update_relative(translation, rotation, estimator.pair_covariance)
        np.testing.assert_allclose(estimator.state.position, before.position+mean[:3], atol=1e-11)
        expected_rotation = before.rotation @ rotation_exp(mean[6:9])
        np.testing.assert_allclose(estimator.state.rotation, expected_rotation, atol=1e-11)
        np.testing.assert_allclose(estimator.clone.position, before.position+mean[:3]+expected_rotation @ mean[21:24], atol=1e-11)
        np.testing.assert_allclose(estimator.clone.rotation, expected_rotation @ rotation_exp(mean[24:27]), atol=1e-11)
        # Production uses first-order reset; numerical retraction includes O(dx²).
        np.testing.assert_allclose(estimator.covariance, expected_covariance, atol=1e-10, rtol=1e-7)
