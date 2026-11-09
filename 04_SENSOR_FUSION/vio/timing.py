import numpy as np


def aligned_camera_times(camera_ns, offset_ns, imu_ns):
    """Offset is added to camera clock to express it in the IMU clock domain."""
    camera = np.asarray(camera_ns, dtype=np.int64)
    imu = np.asarray(imu_ns, dtype=np.int64)
    if len(camera) < 1 or len(imu) < 2:
        raise ValueError("need camera timestamps and at least two IMU samples")
    if np.any(np.diff(camera) <= 0) or np.any(np.diff(imu) <= 0):
        raise ValueError("timestamps must be strictly increasing")
    shifted = [int(timestamp) + int(offset_ns) for timestamp in camera]
    if min(shifted) < int(imu[0]) or max(shifted) > int(imu[-1]):
        raise ValueError("camera timestamps outside IMU coverage")
    return np.asarray(shifted, dtype=np.int64)


def sample_interpolated_imu(target_ns, timestamps_ns, samples):
    timestamps = np.asarray(timestamps_ns, dtype=np.int64)
    samples = np.asarray(samples, dtype=float)
    if len(timestamps) < 2 or len(timestamps) != len(samples) or np.any(np.diff(timestamps) <= 0):
        raise ValueError("IMU samples require matching ordered timestamps")
    if target_ns < timestamps[0] or target_ns > timestamps[-1]:
        raise ValueError("IMU interpolation would extrapolate")
    upper = min(max(int(np.searchsorted(timestamps, target_ns)), 1), len(timestamps)-1)
    weight = (int(target_ns)-int(timestamps[upper-1]))/(int(timestamps[upper])-int(timestamps[upper-1]))
    return samples[upper-1]*(1-weight) + samples[upper]*weight
