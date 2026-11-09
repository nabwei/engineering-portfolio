"""Versioned SLAM Toolbox configuration files; no ROS server or simulator is needed."""

import ast
import hashlib
from pathlib import Path
import unittest

import yaml

from run_qualification import launch_command

PACKAGE = Path(__file__).resolve().parent
CONFIG = PACKAGE / "config"
LAUNCH = PACKAGE / "launch" / "gazebo_navigation.launch.py"
INSTALLED = PACKAGE / "install" / "portfolio_navigation" / "share" / "portfolio_navigation"

# The v1 values are the overrides that were inlined in gazebo_navigation.launch.py
# (SHA-256 4242ae3f6e2e1b2968df66468bf97ac975806ae311a779f6de409f7674172a09) for every
# recorded 2026-10-26 follow-up mission, verbatim; the default launch behaviour must
# keep them. Types matter: rclcpp parameters are typed at declaration.
FROZEN_V1 = {
    "use_sim_time": True,
    "odom_frame": "odom",
    "map_frame": "map",
    "base_frame": "base_link",
    "scan_topic": "/scan",
    "mode": "mapping",
    "resolution": 0.05,
    "max_laser_range": 10.0,
    "minimum_travel_distance": 0.1,
    "minimum_travel_heading": 0.1,
    "check_min_dist_and_heading_precisely": True,
    "minimum_time_interval": 0.1,
    "use_lifecycle_manager": True,
}

# Version 2 adds exactly the candidate selected by the predeclared development rule
# (08_TEST_RESULTS/completion_20261028/h_slam_v2/SELECTION.json: c1_buffer100).
SELECTED_V2_KEYS = {"scan_buffer_size": 100}


def load_parameters(path):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["slam_toolbox"]["ros__parameters"]


def typed_items(parameters):
    return {key: (type(value).__name__, value) for key, value in parameters.items()}


class SlamConfigurationTests(unittest.TestCase):
    def test_both_files_parse_under_the_slam_toolbox_node_name(self):
        for name in ("slam_params_v1.yaml", "slam_params_v2.yaml"):
            with self.subTest(file=name):
                document = yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))
                self.assertEqual(list(document), ["slam_toolbox"])
                self.assertEqual(list(document["slam_toolbox"]), ["ros__parameters"])
                self.assertIsInstance(document["slam_toolbox"]["ros__parameters"], dict)

    def test_v1_file_equals_the_frozen_launch_overrides_with_types(self):
        self.assertEqual(
            typed_items(load_parameters(CONFIG / "slam_params_v1.yaml")),
            typed_items(FROZEN_V1),
        )

    def test_v2_differs_from_v1_only_in_the_selected_keys(self):
        v1 = load_parameters(CONFIG / "slam_params_v1.yaml")
        v2 = load_parameters(CONFIG / "slam_params_v2.yaml")
        self.assertEqual(set(v2) - set(v1), set(SELECTED_V2_KEYS))
        self.assertEqual(set(v1) - set(v2), set())
        self.assertEqual(
            {key: typed_items(v2)[key] for key in v1}, typed_items(v1)
        )
        self.assertEqual(
            {key: v2[key] for key in SELECTED_V2_KEYS}, SELECTED_V2_KEYS
        )
        self.assertIsInstance(v2["scan_buffer_size"], int)
        self.assertNotIsInstance(v2["scan_buffer_size"], bool)

    def test_launch_declares_slam_params_file_defaulting_to_v1_without_inline_overrides(
        self,
    ):
        tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
        declared = {}
        slam_node_parameters = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            callee = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
            if callee == "DeclareLaunchArgument":
                name = node.args[0].value
                default = next(
                    keyword.value
                    for keyword in node.keywords
                    if keyword.arg == "default_value"
                )
                declared[name] = ast.unparse(default)
            if callee == "Node" and any(
                keyword.arg == "package"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value == "slam_toolbox"
                for keyword in node.keywords
            ):
                slam_node_parameters = next(
                    keyword.value
                    for keyword in node.keywords
                    if keyword.arg == "parameters"
                )
        self.assertIn("slam_params_file", declared)
        self.assertIn("slam_params_v1.yaml", declared["slam_params_file"])
        self.assertIn("'config'", declared["slam_params_file"])
        self.assertIsNotNone(slam_node_parameters)
        self.assertIsInstance(slam_node_parameters, ast.List)
        self.assertFalse(
            any(isinstance(element, ast.Dict) for element in slam_node_parameters.elts),
            "inline SLAM override dict must not coexist with the versioned file",
        )
        self.assertIn(
            "LaunchConfiguration('slam_params_file')",
            [ast.unparse(element) for element in slam_node_parameters.elts],
        )

    def test_runner_passes_the_file_through_only_when_requested(self):
        configuration = PACKAGE / "nav2_params.yaml"
        default = launch_command(configuration)
        self.assertFalse(any(arg.startswith("slam_params_file:=") for arg in default))
        v2 = CONFIG / "slam_params_v2.yaml"
        explicit = launch_command(configuration, v2)
        self.assertEqual(explicit[:-1], default)
        self.assertEqual(explicit[-1], "slam_params_file:=" + str(v2.resolve()))

    def test_installed_copies_equal_source_copies(self):
        if not INSTALLED.exists():
            self.skipTest("package not built in-tree; no installed copy to compare")
        for relative in (
            Path("config") / "slam_params_v1.yaml",
            Path("config") / "slam_params_v2.yaml",
            Path("launch") / "gazebo_navigation.launch.py",
            Path("worlds") / "inspection_room.sdf",
        ):
            with self.subTest(file=str(relative)):
                source = hashlib.sha256((PACKAGE / relative).read_bytes()).hexdigest()
                installed = hashlib.sha256(
                    (INSTALLED / relative).read_bytes()
                ).hexdigest()
                self.assertEqual(installed, source, f"rebuild: stale installed {relative}")


if __name__ == "__main__":
    unittest.main()
