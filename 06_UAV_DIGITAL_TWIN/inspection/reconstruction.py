"""Calibrated simulator depth backprojection; this is not image-only SfM."""

import numpy as np
import pybullet as bullet


def backproject(depth, view, projection):
    height, width = depth.shape
    rows, columns = np.indices(depth.shape)
    clip = np.stack(
        [
            2 * (columns + 0.5) / width - 1,
            1 - 2 * (rows + 0.5) / height,
            2 * depth - 1,
            np.ones_like(depth),
        ],
        axis=-1,
    )
    inverse = np.linalg.inv(
        np.asarray(projection).reshape(4, 4, order="F")
        @ np.asarray(view).reshape(4, 4, order="F")
    )
    homogeneous = clip @ inverse.T
    return homogeneous[..., :3] / homogeneous[..., 3, None]


def capture(scene, position):
    client, blade, _ = scene
    target = [0, 0, float(position[2])]
    view = bullet.computeViewMatrix(position.tolist(), target, [0, 0, 1])
    projection = bullet.computeProjectionMatrixFOV(62, 1, 0.1, 6)
    _, _, rgba, depth, mask = bullet.getCameraImage(
        160,
        160,
        view,
        projection,
        renderer=bullet.ER_TINY_RENDERER,
        physicsClientId=client,
    )
    depth, mask = (
        np.asarray(depth).reshape(160, 160),
        np.asarray(mask).reshape(160, 160),
    )
    selected = (mask & ((1 << 24) - 1)) == blade
    cloud = backproject(depth, view, projection)[selected]
    return np.asarray(rgba).reshape(160, 160, 4), cloud


def voxel_fuse(cloud, spacing=0.008):
    _, selected = np.unique(
        np.floor(cloud / spacing).astype(np.int64), axis=0, return_index=True
    )
    return cloud[np.sort(selected)]


def write_ply(path, cloud):
    header = f"ply\nformat ascii 1.0\nelement vertex {len(cloud)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    with path.open("w") as stream:
        stream.write(header)
        np.savetxt(stream, cloud, fmt="%.6f")
