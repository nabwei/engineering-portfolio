"""Re-render a saved camera mission without rerunning flight dynamics."""

import argparse
import json
from pathlib import Path
import numpy as np
import pybullet as bullet
from reconstruction import capture, voxel_fuse, write_ply
from scene import create_scene


def replay(source, output):
    frames = json.loads(source.read_text())
    if not frames or any(
        frame["sequence"] != index for index, frame in enumerate(frames)
    ):
        raise ValueError("nonempty contiguous sequence required")
    ticks = [frame["tick"] for frame in frames]
    if any(next_tick <= tick for tick, next_tick in zip(ticks, ticks[1:])):
        raise ValueError("nonmonotonic replay timestamps")
    scene = create_scene()
    try:
        clouds = []
        for frame in frames:
            position = np.asarray(frame["position_m"])
            bullet.resetBasePositionAndOrientation(
                scene[2], position.tolist(), [0, 0, 0, 1], physicsClientId=scene[0]
            )
            _, cloud = capture(scene, position)
            if len(cloud) != frame["points"]:
                raise RuntimeError("sensor replay point count differs")
            clouds.append(cloud)
        write_ply(output, voxel_fuse(np.concatenate(clouds)))
        print(f"PASS camera replay: {len(frames)} frames")
    finally:
        bullet.disconnect(scene[0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    replay(arguments.source, arguments.output)
