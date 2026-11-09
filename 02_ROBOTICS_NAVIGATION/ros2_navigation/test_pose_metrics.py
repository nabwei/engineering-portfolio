"""Independent geometric cases that scalar endpoint-error comparison misses."""

import math
import unittest

from pose_metrics import aligned_pose_error


class PoseAgreementTests(unittest.TestCase):
    def test_equal_goal_distances_can_have_large_pose_disagreement(self):
        # Both poses are 0.25 m from the origin, on opposite sides.
        position, heading = aligned_pose_error(
            (0.25, 0.0, 0.0), (-0.25, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
        )
        self.assertAlmostEqual(position, 0.5)
        self.assertEqual(heading, 0.0)

    def test_fixed_rotation_translation_and_wrapped_yaw(self):
        position, heading = aligned_pose_error(
            (2.0, 3.0, math.pi), (-2.0, 6.0, -math.pi / 2), (0.0, 1.0, 1.0, 4.0)
        )
        self.assertAlmostEqual(position, 0.0)
        self.assertAlmostEqual(heading, 0.0)


if __name__ == "__main__":
    unittest.main()
