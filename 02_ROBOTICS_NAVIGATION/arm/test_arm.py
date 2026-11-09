import unittest
import numpy as np
import pybullet as bullet
from simulation import ArmWorld, endpoint


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.world = ArmWorld()

    def tearDown(self):
        self.world.close()

    def test_forward_kinematics_matches_pybullet_link_frame(self):
        angles = np.array([0.4, -0.9])
        self.world.reset(angles)
        frame = bullet.getLinkState(
            self.world.arm,
            1,
            computeForwardKinematics=True,
            physicsClientId=self.world.client,
        )
        simulated = bullet.multiplyTransforms(
            frame[4], frame[5], [0.8, 0, 0], [0, 0, 0, 1]
        )[0]
        np.testing.assert_allclose(endpoint(angles), simulated[:2], atol=2e-7)
        self.assertEqual(self.world.image().shape, (600, 600, 4))

    def test_collision_query_preserves_state_and_velocity(self):
        self.world.reset([-1, 0.2])
        self.world.advance([0.2, 0.1])
        before = bullet.getJointStates(
            self.world.arm, [0, 1], physicsClientId=self.world.client
        )
        self.assertFalse(self.world.collision_free(np.array([np.arctan2(0.8, 1.1), 0])))
        after = bullet.getJointStates(
            self.world.arm, [0, 1], physicsClientId=self.world.client
        )
        np.testing.assert_allclose(
            np.array(before, dtype=object)[:, :2].astype(float),
            np.array(after, dtype=object)[:, :2].astype(float),
        )


if __name__ == "__main__":
    unittest.main()
