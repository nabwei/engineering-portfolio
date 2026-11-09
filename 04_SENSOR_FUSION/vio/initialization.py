"""Quiet-start initialization; cannot distinguish rest from constant velocity."""

import numpy as np
from scipy.spatial.transform import Rotation

from .eskf import State


class QuietStartUnavailable(ValueError):
    """The observed IMU stream has no accepted stationary-start window."""


def quiet_initialization(records):
    timestamps, samples = records
    for start in range(0, len(timestamps), 20):
        if int(timestamps[start])-int(timestamps[0]) > 10_000_000_000:
            break
        end = np.searchsorted(timestamps, int(timestamps[start])+1_000_000_000)
        if end >= len(timestamps):
            break
        window = samples[start:end+1]
        if len(window) < 100:
            continue
        gyro, accel = window[:, :3], window[:, 3:6]
        if np.linalg.norm(gyro.std(axis=0)) > 0.02 or np.linalg.norm(accel.std(axis=0)) > 0.25:
            continue
        mean_accel, mean_gyro = accel.mean(axis=0), gyro.mean(axis=0)
        magnitude = np.linalg.norm(mean_accel)
        if abs(magnitude-9.80665) > 0.4 or np.linalg.norm(mean_gyro) > 0.15:
            continue
        rotation, _ = Rotation.align_vectors([[0, 0, 1]], [mean_accel/magnitude])
        radial_bias = mean_accel*(1-9.80665/magnitude)
        state = State(rotation=rotation.as_matrix(), accel_bias=radial_bias, gyro_bias=mean_gyro)
        report = {"start_ns": int(timestamps[start]), "end_ns": int(timestamps[end]),
                  "samples": len(window), "gyro_std_norm": float(np.linalg.norm(gyro.std(axis=0))),
                  "accel_std_norm": float(np.linalg.norm(accel.std(axis=0))),
                  "assumption": "robot initially at rest; constant velocity cannot be detected by IMU alone",
                  "ground_truth_used": False}
        return state, report
    raise QuietStartUnavailable("no one-second quiet IMU interval; stationary initialization unavailable")
