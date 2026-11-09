import numpy as np
from scipy.spatial.transform import Rotation


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation_exp(vector):
    return Rotation.from_rotvec(vector).as_matrix()


def rotation_log(matrix):
    return Rotation.from_matrix(matrix).as_rotvec()


def rigid_alignment(estimate, truth):
    """SE(3) alignment only: metric stereo scale is never fitted away."""
    estimate, truth = np.asarray(estimate), np.asarray(truth)
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 3:
        raise ValueError("trajectories must have matching Nx3 shapes")
    if len(estimate) < 3 or not np.isfinite([estimate, truth]).all():
        raise ValueError("at least three finite positions are required")
    center_e, center_t = estimate.mean(axis=0), truth.mean(axis=0)
    u, _, vt = np.linalg.svd((estimate - center_e).T @ (truth - center_t))
    correction = np.diag([1., 1., np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    return (estimate - center_e) @ rotation.T + center_t


def trajectory_metrics(estimate, truth):
    estimate, truth = np.asarray(estimate), np.asarray(truth)
    aligned = rigid_alignment(estimate, truth)
    errors = np.linalg.norm(aligned - truth, axis=1)
    # Fixed one-sample displacement error; not rotation-aware SE(3) RPE.
    relative = np.linalg.norm(np.diff(aligned, axis=0) - np.diff(truth, axis=0), axis=1)
    return {"ate_se3_rmse_m": float(np.sqrt(np.mean(errors**2))),
            "raw_rmse_m": float(np.sqrt(np.mean(np.sum((estimate-truth)**2, axis=1)))),
            "relative_displacement_rmse_m": float(np.sqrt(np.mean(relative**2)))}
