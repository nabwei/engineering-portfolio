import unittest
import numpy as np
from control import mpc_velocity
from planning import edge_free, plan


class PlanningTests(unittest.TestCase):
    def test_disconnected_configuration_space_reports_budget_exhaustion(self):
        def free(point):
            return abs(point[0]) > 0.2

        with self.assertRaisesRegex(RuntimeError, "exhausted iteration budget"):
            plan([-1, 0], [1, 0], free, 30)

    def test_rrt_routes_around_forbidden_region_and_rejects_blocked_goal(self):
        def free(point):
            return np.linalg.norm(point) > 0.35

        path = plan([-1, 0], [1, 0], free, 200)
        self.assertTrue(
            all(edge_free(first, second, free) for first, second in zip(path, path[1:]))
        )
        self.assertGreater(
            sum(
                np.linalg.norm(second - first) for first, second in zip(path, path[1:])
            ),
            2,
        )
        with self.assertRaises(ValueError):
            plan([-1, 0], [0, 0], free, 10)

    def test_mpc_obeys_velocity_bounds_and_reduces_error(self):
        current, target = np.array([-1.0, 0.0]), np.array([1.0, 0.5])
        command = mpc_velocity(current, target)
        self.assertLessEqual(np.max(np.abs(command)), 1.2)
        self.assertLess(
            np.linalg.norm(current + 0.05 * command - target),
            np.linalg.norm(current - target),
        )


if __name__ == "__main__":
    unittest.main()
