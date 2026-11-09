# L — PyBullet inspection core

Executable software-simulation components: area-oriented coverage waypoints around an original tapered blade mesh; force-controlled point-mass vehicle under deterministic wind in PyBullet; rendered RGB/depth acquisition; calibrated depth backprojection and voxel fusion; surface coverage evaluation; saved mission and camera replay.

This directory implements the reduced plant: it has no rotor/aerodynamic/attitude model, the target is an original synthetic mesh, and reconstruction uses simulator depth with known camera poses. The separate [PX4/Gazebo extension](../px4_integration/README.md) runs the actual autopilot and simulator, adds a licensed public target and reuses the coverage/reconstruction components. Consult that extension's evidence for its own acceptance status. Neither workflow establishes real-flight/inspection accuracy or RGB-only photogrammetry performance.

From this folder:

```sh
../../.venv/bin/python -m unittest -v test_inspection
../../.venv/bin/python run.py --output ../../08_TEST_RESULTS/robotics/inspection
../../.venv/bin/python replay.py ../../08_TEST_RESULTS/robotics/inspection/mission_replay.json ../../08_TEST_RESULTS/robotics/inspection/replayed.ply
```

The mission visits 64 points on four rings, each at 1.4 m horizontal standoff, with a 240 Hz physics clock. Position feedback applies a damped force and gravity compensation; the recorded force disturbance is deterministic. Every simulated step checks for target collision, and waypoint timeout fails explicitly.

The camera uses 160×160 images, 62° field of view, near/far planes 0.1/6 m and a known look-at matrix. `backproject` inverts the actual projection×view matrix and performs homogeneous division. Only pixels segmented as the target contribute. Fusion retains one point per 8 mm voxel.

Coverage is estimated using 12,000 seeded samples uniformly distributed over triangle area. A sample counts as covered if its nearest reconstructed point lies within 4 cm. Acceptance threshold is 85%. This is a simulator geometry coverage metric; it is not a defect-detection accuracy metric. Mesh boundaries and rendered depth discretization affect the result. The reference-to-cloud median is a coverage diagnostic, not independent metrological accuracy evidence.

Saved outputs include RGB frames, ASCII PLY cloud, mission timestamps/poses, and coverage JSON. Camera replay recreates acquisition at saved positions; it checks frame point counts and can be compared byte-for-byte after PLY export. It does not reproduce rotor control.

For actual PX4 build, Gazebo camera, MAVLink mission and recorded depth replay, follow [the integration instructions](../px4_integration/README.md). Actual PX4 communication-path fault campaigns, broader wind/turbulence studies and real hardware are possible extensions beyond the original local simulation scope. Reduced-plant fault evidence must not be represented as those experiments. Independent review and release readiness are tracked separately.
