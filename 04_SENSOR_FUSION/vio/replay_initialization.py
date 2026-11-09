"""Truth-free initialization adapters for a recorded stereo/IMU stream."""

from dataclasses import dataclass

import numpy as np

from .dynamic_initialization import MovingWindow, initialization_covariance, initialize_moving
from .initialization import QuietStartUnavailable, quiet_initialization


@dataclass
class InitializationInputs:
    imu: tuple
    cameras: tuple
    rectification: object
    tracker_factory: object


@dataclass
class InitialEstimate:
    state: object
    report: dict
    covariance: object = None
    first_replay_ns: int = 0


def quiet_start(inputs):
    state, report = quiet_initialization(inputs.imu)
    return InitialEstimate(state, report, first_replay_ns=report["end_ns"])


def automatic_start(inputs):
    try:
        return quiet_start(inputs)
    except QuietStartUnavailable as failure:
        initialized = moving_start(inputs)
        initialized.report["quiet_start_unavailable"] = str(failure)
        return initialized


def window_timestamps(cameras, imu_times):
    """First acquired synchronized pair through the first stamp at or after +5 s (shared rule)."""
    left, right = cameras
    timestamps = sorted(stamp for stamp in set(left) & set(right)
                        if imu_times[0] <= stamp <= imu_times[-1])
    available = [stamp for stamp in timestamps if left[stamp].is_file() and right[stamp].is_file()]
    if not available:
        raise ValueError("no acquired stereo images for moving initialization")
    first = available[0]
    eligible = np.asarray([stamp for stamp in timestamps if stamp >= first], dtype=np.int64)
    last_index = int(np.searchsorted(eligible, first+5_000_000_000))
    return eligible[:last_index+1]


def visual_initialization_window(inputs):
    left, right = inputs.cameras
    window_times = window_timestamps(inputs.cameras, inputs.imu[0])
    tracker = inputs.tracker_factory(inputs.rectification.calibration)
    poses = [tracker.track(*inputs.rectification.images(left[stamp], right[stamp])) for stamp in window_times]
    return MovingWindow(np.asarray(window_times, dtype=np.int64), np.asarray(poses), inputs.imu)


def moving_start(inputs):
    window = visual_initialization_window(inputs)
    state, report = initialize_moving(window)
    covariance, covariance_report = initialization_covariance(window, state)
    report["filter_initial_covariance"] = covariance_report
    # Do not use an initializer image again as a replay measurement.
    return InitialEstimate(state, report, covariance, report["end_ns"]+1)
