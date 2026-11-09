"""Reuse Project L's original mesh and coverage route in a Gazebo world."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from geometry import public_blade

INSPECTION = Path(__file__).resolve().parents[1] / "inspection"
sys.path.insert(0, str(INSPECTION))
from scene import FACES, VERTICES, coverage_path  # noqa: E402 -- reuse sibling inspection implementation


def local_ned(enu):
    east, north, up = enu
    return [float(north), float(east + 5.0), float(-up)]


def mesh_text(mesh_vertices=VERTICES, mesh_faces=FACES):
    vertices = ["v " + " ".join(map(str, vertex)) for vertex in mesh_vertices]
    normals = []
    faces = []
    for normal_index, face in enumerate(mesh_faces, start=1):
        a, b, c = np.asarray(mesh_vertices)[face]
        normal = np.cross(b - a, c - a)
        normal /= np.linalg.norm(normal)
        normals.append("vn " + " ".join(map(str, normal)))
        faces.append("f " + " ".join(f"{index + 1}//{normal_index}" for index in face))
    return "\n".join(vertices + normals + faces) + "\n"


def add_target(world, mesh):
    model = ET.SubElement(world, "model", name="portfolio_blade")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "5 0 0 0 0 0"
    link = ET.SubElement(model, "link", name="blade")
    for kind in ("collision", "visual"):
        surface = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(surface, "geometry")
        mesh_node = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh_node, "uri").text = mesh.resolve().as_uri()


def prepare(px4_source, output, public_mesh=None):
    output.mkdir(parents=True, exist_ok=True)
    vertices, faces = np.asarray(VERTICES), np.asarray(FACES)
    provenance = {"kind": "original procedural blade"}
    if public_mesh is not None:
        vertices, faces, transform = public_blade(public_mesh)
        provenance = {"kind": "public MIT blade", "source_file": public_mesh.name,
                      "sha256": hashlib.sha256(public_mesh.read_bytes()).hexdigest(), "transform": transform}
    mesh = output / "blade.obj"
    mesh.write_text(mesh_text(vertices, faces), encoding="ascii")
    np.savez_compressed(output / "blade_mesh.npz", vertices=vertices, faces=faces)
    default_world = px4_source / "Tools/simulation/gz/worlds/default.sdf"
    tree = ET.parse(default_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("PX4 default world has no world element")
    world.set("name", "portfolio_inspection")
    add_target(world, mesh)
    tree.write(output / "portfolio_inspection.sdf", encoding="utf-8", xml_declaration=True)
    server_config = px4_source / "src/modules/simulation/gz_bridge/server.config"
    if server_config.is_file():
        server = ET.parse(server_config)
        plugins = server.getroot().find("plugins")
        # Native depth transport is used; the optional RTP/GStreamer plugin is unnecessary.
        for plugin in list(plugins):
            if plugin.get("filename") == "libGstCameraSystem.so":
                plugins.remove(plugin)
        server.write(output / "server.config", encoding="utf-8", xml_declaration=True)
    camera_model = output / "models/portfolio_x500"
    camera_model.mkdir(parents=True, exist_ok=True)
    (camera_model / "model.sdf").write_text('''<?xml version="1.0"?>
<sdf version="1.9"><model name="portfolio_x500">
  <include merge="true"><uri>model://x500</uri></include>
  <link name="portfolio_camera"><pose relative_to="base_link">0.15 0 0.25 0 0 0</pose>
    <inertial><mass>0.02</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
    <sensor name="depth" type="depth_camera">
      <always_on>true</always_on><update_rate>5</update_rate>
      <topic>/portfolio/depth</topic>
      <camera><horizontal_fov>1.274</horizontal_fov>
        <image><width>160</width><height>120</height><format>R_FLOAT32</format></image>
        <clip><near>0.15</near><far>8</far></clip>
      </camera>
    </sensor>
  </link>
  <joint name="portfolio_camera_mount" type="fixed"><parent>base_link</parent><child>portfolio_camera</child></joint>
</model></sdf>
''', encoding="utf-8")
    route = [local_ned(point) for point in coverage_path()]
    report = {"frame": "LOCAL_NED", "target_center_enu_m": [5, 0, 0], "waypoints": route,
              "transit_altitude_m": float(vertices[:, 2].max() + 1.0), "blade": provenance,
              "route_source": "../inspection/scene.py coverage_path",
              "scene_source_sha256": hashlib.sha256((INSPECTION / "scene.py").read_bytes()).hexdigest()}
    (output / "mission.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--public-mesh", type=Path)
    arguments = parser.parse_args()
    print(json.dumps(prepare(arguments.px4_source, arguments.output, arguments.public_mesh), indent=2))
