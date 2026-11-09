import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from control import mpc_velocity, trajectory
from planning import edge_free, plan
from simulation import ArmWorld, endpoint


def execute(output):
    output.mkdir(parents=True, exist_ok=True)
    world = ArmWorld()
    try:
        path = plan([-1.2, 0.3], [1.8, -0.6], world.collision_free)
        world.reset(path[0])
        replay = []
        for target in list(trajectory(path)) + [path[-1]] * 100:
            current = world.configuration()
            command = mpc_velocity(current, target)
            if not edge_free(current, current + 0.05 * command, world.collision_free):
                raise RuntimeError("MPC predicted edge violates collision margin")
            world.advance(command)
            if not world.collision_free(world.configuration()):
                raise RuntimeError("actual arm collision")
            replay.append(world.configuration().tolist())
        error = float(np.linalg.norm(world.configuration() - path[-1]))
        if error > 0.02:
            raise RuntimeError(f"goal error {error} exceeds tolerance")
        Image.fromarray(world.image().astype("uint8")).save(output / "arm.png")
        report = {
            "status": "PASS",
            "simulator": "PyBullet DIRECT",
            "planning": "RRT*",
            "waypoints": len(path),
            "samples": len(replay),
            "goal_joint_error_rad": error,
            "final_endpoint_m": endpoint(world.configuration()).tolist(),
        }
        (output / "arm_report.json").write_text(json.dumps(report, indent=2))
        (output / "arm_replay.json").write_text(
            json.dumps(
                {"path": [point.tolist() for point in path], "trajectory": replay}
            )
        )
        print(json.dumps(report))
    finally:
        world.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    execute(parser.parse_args().output)
