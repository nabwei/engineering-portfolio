"""Planar pose agreement under one fixed map-to-world SE(2) alignment."""

import math


def aligned_pose_error(estimated, truth, alignment):
    """Return position norm in metres and wrapped absolute yaw error in radians."""
    cosine, sine, tx, ty = alignment
    x, y, heading = estimated
    aligned_x = cosine * x - sine * y + tx
    aligned_y = sine * x + cosine * y + ty
    angle = heading + math.atan2(sine, cosine) - truth[2]
    return (
        math.hypot(aligned_x - truth[0], aligned_y - truth[1]),
        abs(math.atan2(math.sin(angle), math.cos(angle))),
    )
