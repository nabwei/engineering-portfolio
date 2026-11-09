import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from prepare_scene import local_ned, mesh_text, prepare


class SceneTests(unittest.TestCase):
    def test_collision_mesh_contains_a_normal_for_every_face(self):
        lines = mesh_text().splitlines()
        normals = [line for line in lines if line.startswith("vn ")]
        faces = [line for line in lines if line.startswith("f ")]
        self.assertEqual(len(normals), len(faces))
        for index, face in enumerate(faces, start=1):
            self.assertTrue(all(token.endswith(f"//{index}") for token in face.split()[1:]))

    def test_axis_conversion_preserves_up_and_applies_target_offset(self):
        self.assertEqual(local_ned([1, 2, 3]), [2, 6, -3])

    def test_real_coverage_route_and_mesh_are_exported_into_separate_world(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            world = root / "Tools/simulation/gz/worlds/default.sdf"
            world.parent.mkdir(parents=True)
            original = '<sdf version="1.9"><world name="default"><gravity>0 0 -9.8</gravity></world></sdf>'
            world.write_text(original, encoding="ascii")
            report = prepare(root, root / "generated")
            generated = ET.parse(root / "generated/portfolio_inspection.sdf").getroot().find("world")
            self.assertEqual(len(report["waypoints"]), 64)
            self.assertAlmostEqual(report["waypoints"][0][1], 6.4)
            self.assertEqual(generated.attrib["name"], "portfolio_inspection")
            self.assertEqual(generated.find("model/pose").text, "5 0 0 0 0 0")
            self.assertEqual(world.read_text(encoding="ascii"), original)


if __name__ == "__main__":
    unittest.main(verbosity=2)
