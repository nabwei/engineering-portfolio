"""PyBullet collision oracle and velocity-controlled two-link arm."""

from pathlib import Path
import numpy as np
import pybullet as bullet


class ArmWorld:
    def __init__(self):
        self.client = bullet.connect(bullet.DIRECT)
        bullet.setTimeStep(1 / 240, physicsClientId=self.client)
        bullet.setGravity(0, 0, 0, physicsClientId=self.client)
        self.arm = bullet.loadURDF(
            str(Path(__file__).with_name("arm.urdf")),
            useFixedBase=True,
            physicsClientId=self.client,
        )
        shape = bullet.createCollisionShape(
            bullet.GEOM_BOX, halfExtents=[0.13, 0.13, 0.2], physicsClientId=self.client
        )
        visual = bullet.createVisualShape(
            bullet.GEOM_BOX,
            halfExtents=[0.13, 0.13, 0.2],
            rgbaColor=[0.9, 0.3, 0.2, 1],
            physicsClientId=self.client,
        )
        self.obstacle = bullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=shape,
            baseVisualShapeIndex=visual,
            basePosition=[1.1, 0.8, 0],
            physicsClientId=self.client,
        )

    def close(self):
        bullet.disconnect(self.client)

    def reset(self, configuration):
        for joint, angle in enumerate(configuration):
            bullet.resetJointState(
                self.arm, joint, float(angle), physicsClientId=self.client
            )

    def configuration(self):
        return np.array(
            [
                bullet.getJointState(self.arm, joint, physicsClientId=self.client)[0]
                for joint in range(2)
            ]
        )

    def collision_free(self, configuration):
        if not np.isfinite(configuration).all() or np.max(np.abs(configuration)) > 2.8:
            return False
        saved = bullet.getJointStates(self.arm, [0, 1], physicsClientId=self.client)
        self.reset(configuration)
        contacts = bullet.getClosestPoints(
            self.arm, self.obstacle, 0.015, physicsClientId=self.client
        )
        for joint, state in enumerate(saved):
            bullet.resetJointState(
                self.arm,
                joint,
                state[0],
                targetVelocity=state[1],
                physicsClientId=self.client,
            )
        return not contacts

    def advance(self, velocity):
        bullet.setJointMotorControlArray(
            self.arm,
            [0, 1],
            bullet.VELOCITY_CONTROL,
            targetVelocities=velocity,
            forces=[20, 10],
            physicsClientId=self.client,
        )
        for _ in range(12):
            bullet.stepSimulation(physicsClientId=self.client)

    def image(self):
        view = bullet.computeViewMatrix([0, 0, 4], [0, 0, 0], [0, 1, 0])
        projection = bullet.computeProjectionMatrixFOV(55, 1, 0.1, 10)
        return np.asarray(
            bullet.getCameraImage(
                600,
                600,
                view,
                projection,
                renderer=bullet.ER_TINY_RENDERER,
                physicsClientId=self.client,
            )[2]
        ).reshape(600, 600, 4)


def endpoint(configuration):
    shoulder, elbow = configuration
    return np.array(
        [
            np.cos(shoulder) + 0.8 * np.cos(shoulder + elbow),
            np.sin(shoulder) + 0.8 * np.sin(shoulder + elbow),
        ]
    )
