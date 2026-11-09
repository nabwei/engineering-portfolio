"""Bound-constrained finite-horizon MPC for the joint velocity integrator model."""

import numpy as np
from scipy.optimize import minimize


def trajectory(path, seconds_per_segment=1.5):
    samples = []
    for first, second in zip(path, path[1:]):
        duration = max(seconds_per_segment, 2 * np.linalg.norm(second - first))
        for fraction in np.linspace(
            0, 1, int(np.ceil(duration / 0.05)), endpoint=False
        ):
            blend = fraction * fraction * (3 - 2 * fraction)
            samples.append(first + (second - first) * blend)
    samples.append(path[-1])
    return np.asarray(samples)


def mpc_velocity(configuration, reference):
    horizon = 8
    integration = 0.05 * np.tril(np.ones((horizon, horizon)))
    offset = configuration - reference

    def objective(flat_controls):
        controls = flat_controls.reshape(horizon, 2)
        errors = offset + integration @ controls
        cost = np.sum(errors * errors) + 0.03 * np.sum(controls * controls)
        gradient = 2 * (integration.T @ errors + 0.03 * controls)
        return cost, gradient.ravel()

    solution = minimize(
        objective,
        np.zeros(horizon * 2),
        jac=True,
        bounds=[(-1.2, 1.2)] * (horizon * 2),
        method="L-BFGS-B",
    )
    if not solution.success:
        raise RuntimeError(f"MPC optimizer failed: {solution.message}")
    return solution.x.reshape(horizon, 2)[0]
