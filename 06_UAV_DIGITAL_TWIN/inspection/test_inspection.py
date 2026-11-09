import unittest
import json
import tempfile
from pathlib import Path
import numpy as np
import pybullet as bullet
from reconstruction import backproject, voxel_fuse, capture
from scene import coverage_path, create_scene
from replay import replay


class InspectionTests(unittest.TestCase):
    def test_invalid_replay_order_is_rejected_before_rendering(self):
        invalid_sequences = [
            [],
            [{"sequence": 1, "tick": 1}],
            [{"sequence": 0, "tick": 2}, {"sequence": 1, "tick": 1}],
        ]
        with tempfile.TemporaryDirectory() as directory:
            source, destination = (
                Path(directory) / "replay.json",
                Path(directory) / "cloud.ply",
            )
            for frames in invalid_sequences:
                with self.subTest(frames=frames):
                    source.write_text(json.dumps(frames))
                    with self.assertRaises(ValueError):
                        replay(source, destination)
                    self.assertFalse(destination.exists())

    def test_rendered_camera_shape_and_target_cloud(self):
        scene = create_scene()
        try:
            rgba, cloud = capture(scene, np.array([1.4, 0, 1.5]))
            self.assertEqual(rgba.shape, (160, 160, 4))
            self.assertEqual(cloud.shape[1], 3)
            self.assertGreater(len(cloud), 100)
            self.assertTrue(np.isfinite(cloud).all())
        finally:
            bullet.disconnect(scene[0])

    def test_identity_projection_has_known_pixel_centers(self):
        cloud = backproject(
            np.array([[0.5, 1.0]]), np.eye(4).ravel(), np.eye(4).ravel()
        )
        np.testing.assert_allclose(cloud, [[[-0.5, 0, 0], [0.5, 0, 1]]])

    def test_voxel_fusion_preserves_distinct_cells(self):
        fused = voxel_fuse(np.array([[0, 0, 0], [0.001, 0, 0], [0.03, 0, 0]]), 0.01)
        np.testing.assert_allclose(fused, [[0, 0, 0], [0.03, 0, 0]])

    def test_mission_has_standoff_and_vertical_coverage(self):
        path = np.asarray(coverage_path())
        np.testing.assert_allclose(np.linalg.norm(path[:, :2], axis=1), 1.4)
        self.assertEqual(len(path), 64)
        self.assertGreater(np.ptp(path[:, 2]), 2)


if __name__ == "__main__":
    unittest.main()
