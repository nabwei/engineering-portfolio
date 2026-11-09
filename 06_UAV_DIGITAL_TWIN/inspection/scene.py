"""Original tapered inspection target and reduced translational UAV plant."""

import numpy as np
import pybullet as bullet

VERTICES = [
    [-0.45, -0.06, 0.2],
    [0.45, -0.06, 0.2],
    [0.45, 0.06, 0.2],
    [-0.45, 0.06, 0.2],
    [-0.1, -0.035, 3.2],
    [0.1, -0.035, 3.2],
    [0.1, 0.035, 3.2],
    [-0.1, 0.035, 3.2],
]
FACES = [
    [0, 2, 1],
    [0, 3, 2],
    [4, 5, 6],
    [4, 6, 7],
    [0, 1, 5],
    [0, 5, 4],
    [1, 2, 6],
    [1, 6, 5],
    [2, 3, 7],
    [2, 7, 6],
    [3, 0, 4],
    [3, 4, 7],
]


def create_scene():
    client = bullet.connect(bullet.DIRECT)
    bullet.setGravity(0, 0, -9.81, physicsClientId=client)
    bullet.setTimeStep(1 / 240, physicsClientId=client)
    indices = np.asarray(FACES).ravel().tolist()
    collision = bullet.createCollisionShape(
        bullet.GEOM_MESH,
        vertices=VERTICES,
        indices=indices,
        flags=bullet.GEOM_FORCE_CONCAVE_TRIMESH,
        physicsClientId=client,
    )
    visual = bullet.createVisualShape(
        bullet.GEOM_MESH,
        vertices=VERTICES,
        indices=indices,
        rgbaColor=[0.8, 0.85, 0.9, 1],
        physicsClientId=client,
    )
    blade = bullet.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        physicsClientId=client,
    )
    sphere = bullet.createCollisionShape(
        bullet.GEOM_SPHERE, radius=0.06, physicsClientId=client
    )
    drone = bullet.createMultiBody(
        baseMass=0.6,
        baseCollisionShapeIndex=sphere,
        basePosition=[1.4, 0, 0.6],
        physicsClientId=client,
    )
    return client, blade, drone


def surface_samples():
    vertices = np.asarray(VERTICES)
    triangles = vertices[np.asarray(FACES)]
    areas = (
        np.linalg.norm(
            np.cross(
                triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
            ),
            axis=1,
        )
        / 2
    )
    random = np.random.default_rng(81)
    selected = triangles[random.choice(len(triangles), 12000, p=areas / areas.sum())]
    root = np.sqrt(random.random((12000, 1)))
    fraction = random.random((12000, 1))
    return (
        (1 - root) * selected[:, 0]
        + root * (1 - fraction) * selected[:, 1]
        + root * fraction * selected[:, 2]
    )


def coverage_path():
    waypoints = []
    for level, height in enumerate([0.6, 1.5, 2.5, 3.0]):
        angles = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        if level % 2:
            angles = angles[::-1]
        waypoints.extend(
            [
                np.array([1.4 * np.cos(angle), 1.4 * np.sin(angle), height])
                for angle in angles
            ]
        )
    return waypoints


def advance_to(scene, target, tick):
    client, blade, drone = scene
    for step in range(700):
        position = np.asarray(
            bullet.getBasePositionAndOrientation(drone, physicsClientId=client)[0]
        )
        velocity = np.asarray(bullet.getBaseVelocity(drone, physicsClientId=client)[0])
        wind = np.array([0.025 * np.sin((tick + step) / 240), 0.01, 0])
        force = (
            0.6 * (12 * (target - position) - 7 * velocity + np.array([0, 0, 9.81]))
            + wind
        )
        bullet.applyExternalForce(
            drone,
            -1,
            force.tolist(),
            position.tolist(),
            bullet.WORLD_FRAME,
            physicsClientId=client,
        )
        bullet.stepSimulation(physicsClientId=client)
        if bullet.getContactPoints(drone, blade, physicsClientId=client):
            raise RuntimeError("inspection vehicle collided with target")
        if (
            np.linalg.norm(target - position) < 0.015
            and np.linalg.norm(velocity) < 0.02
        ):
            return position, tick + step + 1
    raise RuntimeError("inspection waypoint not reached within simulation budget")
