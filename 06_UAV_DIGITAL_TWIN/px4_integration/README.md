# PX4 SITL and Gazebo inspection

This extension runs the real PX4 v1.17.0 autopilot in software, Gazebo Harmonic rigid-body simulation, MAVLink offboard control, and a native Gazebo depth camera. It reuses Project L's 64-point coverage route and point-cloud fusion/output functions. It is software simulation, not physical Hardware-in-the-Loop or a flight qualification.

The public target is Daniel Joo's MIT-licensed blade mesh; see [provenance and license](../assets/public_blade/PROVENANCE.md). Only OBJ data is imported. A uniform scale of `3 / 60.5` reduces the mesh height from 60.5 to 3 m; the original XY bounding-box center is subtracted and the scaled root is placed at Z=0.2 m. The world target is then translated by [5,0,0] m. `mission.json` records the exact transform and original file hash. This is a small-scale simulator scene, not a validated full-size wind turbine or aerodynamic model. The original procedural target remains selectable by omitting `--public-mesh`.

## Executable evidence

Saved results are in `08_TEST_RESULTS/px4/`:

- `smoke/`: initial FAIL, where missing OBJ normals caused a DART collision-loader crash. The exporter now writes normals and has a regression test.
- `smoke_mesh_fix/`: PASS, takeoff, one waypoint, native depth capture, return, landing and disarming.
- `full_mission/`: PASS, all 64 original-target waypoints, 64 depth frames, landing/disarming. Corrected reconstruction: 54,347 fused points and 93.667% of 12,000 area-weighted surface samples within 4 cm.
- `public_mission/`: PASS, all 64 public-model waypoints, 64 frames, landing/disarming; 18,732 fused points and 96.30% surface coverage within 4 cm. P95 surface distance is 1.398 cm. Consult its `result.json` and `replay/reconstruction.json` for measured evidence.
- `final_smoke/` and `calibrated_smoke/`: short public-model runs checking the final cleanup and calibration paths; their result files record executed status.

`result.json` only becomes PASS after all requested waypoints and automatic disarming. It distinguishes a partial route from a complete one. The model's estimated local position must be fresh, within 0.18 m of each target, and slower than 0.2 m/s for 0.4 s. There is an additional one-second settling period before a camera frame. Transit rises to 4.2 m before crossing over the target.

The reconstruction uses metric simulated depth, known camera intrinsics, simulator truth poses, and a known target bounding box. It does **not** estimate poses from images, perform RGB-only photogrammetry, or demonstrate real-world inspection accuracy. Surface coverage uses seeded area-weighted samples on the actual selected mesh, not mesh vertex counts. Camera/pose timestamps are checked within 30 ms; stopped streams and stale flight measurements fail explicitly.

Visual review found an initial 0.24 m vertical reconstruction error from assuming the camera mount was expressed relative to the root model. The nested upstream `x500_base` adds 0.24 m. `resolve_camera.py` now uses the actual sensor semantic pose in libsdformat, yielding [0.15,0,0.49] m relative to the recorded root pose, and archives the resolved SDF and calibration. An independent check of 163,640 ground-plane pixels across 64 frames reduced median ground Z residual from −0.240001 m to −0.00000219 m (P95 absolute residual 0.000515 m). This validates simulator geometry, not real measurement accuracy. Old reports/clouds remain in `replay_superseded_extrinsic/` and are not final evidence. Corrected public-model coverage loss is confined to the lowest 0.2–0.7 m band; its measured coverage is 81.76%.

## Source and environment

Validated environment: Ubuntu 24.04 under WSL, GCC 13.3, CMake, Ninja, Python 3.12, ROS 2 Jazzy vendor Gazebo 8.15.0, gz-transport 13.6.0. The repository-wide Python environment supplies NumPy/SciPy/PyBullet for scene export and reconstruction. The separate PX4 environment is pinned in `requirements-px4.lock.txt` and supplies PX4 build dependencies, pymavlink, protobuf and pytest.

Public source pins:

| Dependency | Revision |
|---|---|
| PX4-Autopilot v1.17.0 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| PX4 Gazebo models submodule | `b6127f4ec20de867e215fb5f78ae88b80f371909` |
| PX4-OpticalFlow | `8758bd5567aadf2f8ab5deee97701d2c495d0756` |
| Micro-CDR v2.0.1 | `3d1b17703c7cf4f22def2910bc845bdb5152d7b5` |

Upstream v1.17.0 otherwise selects OpticalFlow's moving `master` branch. `pin_opticalflow.py` checks the PX4 revision and makes this one documented build-source change before configuring. Other selected submodule revisions are fixed by the PX4 commit. Third-party source/build caches are excluded from releases; retain their original licenses in those caches. The public blade asset's MIT license is included with the asset.

Official references: [PX4 release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0), [Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/), [upstream Ubuntu setup](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/Tools/setup/ubuntu.sh). The observed release has no x86-64 prebuilt SITL Debian package, so this workspace built from source.

## Reproduce the build

Run these commands from the portfolio root on Linux and use a cache outside the release tree. They assume the official ROS Jazzy/Gazebo package repositories are already configured as described in the master environment documentation. Needed development packages include `build-essential cmake ninja-build git libeigen3-dev libxml2-dev libxml2-utils libssl-dev protobuf-compiler cppzmq-dev pkg-config libopencv-dev python3-venv` and `ros-jazzy-ros-gz`.

```bash
export PORTFOLIO="$(pwd)"
export CACHE="$HOME/Engineering_Portfolio_cache"
export INTEGRATION="$PORTFOLIO/06_UAV_DIGITAL_TWIN/px4_integration"
mkdir -p "$CACHE"
git clone --depth 1 --branch v1.17.0 --no-recurse-submodules \
  https://github.com/PX4/PX4-Autopilot.git "$CACHE/PX4-Autopilot"
git -C "$CACHE/PX4-Autopilot" submodule update --init --recursive --depth 1 --jobs 2 \
  src/modules/mavlink/mavlink src/drivers/gps/devices src/lib/events/libevents \
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client src/lib/heatshrink/heatshrink \
  Tools/simulation/gz src/lib/cdrstream/cyclonedds src/lib/cdrstream/rosidl
git clone --depth 1 --branch v2.0.1 https://github.com/eProsima/Micro-CDR.git "$CACHE/Micro-CDR"
python3 -m venv "$CACHE/px4-venv"
"$CACHE/px4-venv/bin/python" -m pip install -r "$INTEGRATION/requirements-px4.lock.txt"
bash "$INTEGRATION/build_px4.sh" "$CACHE/PX4-Autopilot" "$CACHE/px4-venv" "$CACHE/Micro-CDR"
```

The build is limited to two jobs. The local Micro-CDR URL rewrite is scoped to this build process; it avoids this host's intermittent WSL GitHub DNS failure. Windows Git was used to obtain some source caches on this machine, followed by a Linux checkout to normalize CRLF. TLS verification was not disabled. The full build and subsequent pin verification logs are saved under `08_TEST_RESULTS/px4*build*.log`.

## Run and replay

```bash
"$PORTFOLIO/.venv/bin/python" "$INTEGRATION/prepare_scene.py" \
  --px4-source "$CACHE/PX4-Autopilot" --output "$CACHE/px4-scene-public" \
  --public-mesh "$PORTFOLIO/06_UAV_DIGITAL_TWIN/assets/public_blade/blade_geometry.obj"
source /opt/ros/jazzy/setup.bash
"$CACHE/px4-venv/bin/python" "$INTEGRATION/run_sitl.py" \
  --px4-source "$CACHE/PX4-Autopilot" --scene "$CACHE/px4-scene-public" \
  --output "$PORTFOLIO/08_TEST_RESULTS/px4/public_mission_new" --waypoints 64 --camera
"$PORTFOLIO/.venv/bin/python" "$INTEGRATION/reconstruct.py" \
  --camera-directory "$PORTFOLIO/08_TEST_RESULTS/px4/public_mission_new/camera" \
  --output "$PORTFOLIO/08_TEST_RESULTS/px4/public_mission_new/replay"
```

Use a fresh output directory for each flight. The runner owns and stops only the process groups it creates. It uses a unique Gazebo partition, PX4 instance 7, localhost UDP port 14547 and system ID 8. It rejects an occupied receive port and unexpected system ID. It accepts no remote-vehicle endpoint. No physical autopilot, serial device, flight controller or aircraft is accessed.

For a quick smoke test use `--waypoints 1`. Camera frame archives include metric depth, simulator pose, and timestamps. The mission folder also stores the selected transformed mesh and resolved camera calibration, so replay needs neither Gazebo nor the upstream source cache. Run reconstruction twice into different directories and compare `ply_sha256`; exact recorded-data replay is distinct from bit-identical repetition of a dynamic flight.

## Tests and limitations

```bash
"$PORTFOLIO/.venv/bin/python" -m pytest "$INTEGRATION/test_scene.py" \
  "$INTEGRATION/test_geometry.py" "$INTEGRATION/test_reconstruct.py" "$INTEGRATION/test_processes.py" -q
"$CACHE/px4-venv/bin/python" -m pytest "$INTEGRATION/test_offboard.py" -q
```

The ten pure tests cover route conversion/export, collision normals, public-model normalization, area-weighted sampling, camera backprojection, invalid depth, a known ground plane, orphan-process cleanup and persistence failure. Five MAVLink/output unit tests cover active setpoint fields, stale pose rejection, bounded command timeout, repeated measurement timestamps and rejection of reused output folders. Actual mission logs separately exercise PX4, Gazebo physics, sensors, offboard commands and landing. Unit tests use temporary minimal world fixtures and the bundled public asset; they do not require the source cache.

The original local simulation capabilities are implemented: PX4 SITL, Gazebo vehicle,
public blade model, coverage path, simulated camera acquisition, 3D reconstruction,
coverage evaluation and recorded mission replay. The 2026-10-26 offline recheck reran
all 15 integration unit tests and recreated the 64-frame public mission cloud with
the same SHA-256 and 96.30% coverage; evidence is in
`08_TEST_RESULTS/px4/qualification_20261026/`.

Real hardware/outdoor flight, RGB-only reconstruction, calibration from real images,
broader wind/turbulence campaigns and actual PX4 communication-path fault campaigns
are additional studies, not missing features required by the original simulation
request. Timestamp/ACK failure injection is currently unit-level; process cleanup
and invalid geometric inputs also have executed failure tests. None of those unit
tests is described as a live communication-path fault experiment. Independent review
and release packaging remain separately tracked portfolio activities.


## Historical capture provenance

The saved 26 October public mission uses the older `source` metadata label;
current `prepare_scene.py` emits `route_source`. This naming change does not
mean the saved capture was generated by today's metadata schema. The 28 October
provenance audit compares actual bytes and arrays: the bundled public asset hash,
transformed mesh, 64-point route and `scene.py` hash match the saved mission.
The original corrected and qualification-replay PLY files have the same SHA-256
`abc5807c59aa81ffea3af8c3f562cdb687d3ef712faa225d38f43295cc948eb3`.
Evidence: `08_TEST_RESULTS/committee_20261028/uav_provenance.json`.

Coverage of 96.30% within 4 cm is a measured outcome of this simulated depth/truth-pose
reconstruction. The PX4 mission did not declare a numeric coverage pass threshold;
none is imposed after seeing the result. The reduced-plant demonstration has its
own checks and is not the PX4 mission's acceptance contract. This provenance audit
is not a new flight or evidence of real-world accuracy.
