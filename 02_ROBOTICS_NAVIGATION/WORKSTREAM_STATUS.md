# Robotics workstream evidence

This file is a handoff to master tracking, not publication approval. Project percentages must be calculated from the master milestone definitions.

## Existing-code audit

Inspected `nabwei/pid_regler_c_main` at `e27cc8fac68e863b2615947cc295cedea257c80b`, including `pid_min_project/pid_min.c` and `tests/check_controller.c`. Existing PID includes derivative on measurement, trapezoidal integration, output clamps and conditional antiwindup. Classification: EXISTING, reuse as the PID study reference; no duplicate PID implementation added. G's geometric controller/robot estimation are NEW. No private research source was copied into these projects.

## G — verified software simulation

PASS: C++17 build with warning checks; analytical kinematics; LiDAR wall range; footprint and swept collision checks; A* detour/unreachable behavior; EKF covariance and sensor dropout; Pure Pursuit boundary behavior; complete collision-free mission; ASan/UBSan; same-runtime deterministic replay; SVG demonstration; detailed Arabic study material.

Mission reaches its goal in 56.86 simulated seconds. Replay SHA256: `476cc07897db2d5df788196c858e199f94b803e96ad3c61e5c8c0e9d41d1d9ff`.

Evidence: `../08_TEST_RESULTS/robotics/g_build_tests.log`, `g_build_sanitizer.log`, `trajectory.csv`, `robot_replay.svg`. Source: `robot_core/`. Independent Claude review/release acceptance is controlled by root and is not claimed here.

## J — verified PyBullet arm demonstration

PASS: original URDF; analytical/PyBullet kinematics comparison; state-preserving collision oracle; seeded RRT* and rewiring; cubic trajectory; bounded finite-horizon MPC; actual PyBullet dynamics with collision/goal checks; five unit/integration scenarios including disconnected-space failure; exact repeated replay; rendered final state; Arabic study guide.

Result: 10 path waypoints, 400 control samples, final joint error `1.3697575697126277e-05` rad. Replay SHA256: `ee9310a77286b68d5dcaf4b179a65d327eb1cbce93da14a276e882d5f16aba86`.

Evidence: `../08_TEST_RESULTS/robotics/j_reproducibility.log`, `j_full_validation.log`, `arm/`, `arm_repeat/`. Source: `arm/`. Finite collision discretization and two-link geometry are explicit scope limits.

## K — verified local MQTT fleet system

PASS: actual Mosquitto broker, Paho2 producers/subscriber, validated payloads, indexed SQLite time-series persistence, duplicate suppression, sequence gap/delay monitoring, saved dashboard, continuous collector with live localhost HTTP dashboard, real MQTT→SQLite→HTTP integration test, producer-side delay/loss injection, logs, Arabic guide.

Bounded run receives 56 unique messages: alpha20, beta16, gamma20; beta gaps `[0,5,10,15]` detected. A separate task-owned network namespace run applied actual kernel netem packet faults only to its isolated loopback: 5 ms delay, 5% loss, 32 recorded packet drops, intended MQTT delivery recovered by TCP. Timing is software timing and varies between runs.

Evidence: `../08_TEST_RESULTS/robotics/k_mqtt_integration.log`, `k_service_tests.log`, `fleet/`. Source: `fleet/`. Live service sequences must continue across restarts or use a new identifier. No distributed database or production deployment claim.

## L — verified inspection core, flagship integration separate

PASS: original tapered synthetic mesh; reduced translational PyBullet plant with deterministic force disturbance; 64 coverage waypoints; RGB/depth capture; calibrated depth reconstruction; voxel fusion; area-uniform sampled coverage; recorded camera replay; exact repeated physics mission; five unit/integration scenarios including invalid replay failure; rendered evidence figure; Arabic guide.

Result: 105,399 fused points; 99.1833% of 12,000 area-uniform samples within 4 cm; 120.8458 simulated seconds. This is synthetic geometry coverage, not inspection/flight accuracy. Mission SHA256: `3883e62da443009f62f7cebca9ffe19529ac8f709fcd6d6ea0d791c070970904`.

Evidence: `../08_TEST_RESULTS/robotics/l_full_validation.log`, `l_reproducibility.log`, `inspection/`. Source: `../06_UAV_DIGITAL_TWIN/inspection/`. PX4/Gazebo/public-asset integration is owned separately in `px4_integration`; the reduced plant does not satisfy those milestones.

## H — verified ROS 2 and actual Gazebo integration

PASS: colcon C++ build; original Gazebo SDF validation; G-core ROS adapter; real Gazebo differential-drive physics and rendered LiDAR; clock/scan/odom/TF bridges; SLAM map; active Nav2 stack; goal completion; outside-map planner rejection; command clamp, near obstacle, missing/stale/future scan watchdog; managed shutdown; scientific evidence PNG; Arabic guide. Core and Gazebo backends are tested separately.

Final Gazebo run: 126 scans, 629 odometry messages, 14,161 SLAM map cells, successful NavigateToPose, independently measured goal error 0.241218 m ≤ 0.3 m, sampled clearance 0.653137 m ≥ 0.22 m, GOAL_OUTSIDE_MAP=204 rejection, clean shutdown. Evidence: `../08_TEST_RESULTS/robotics/h_final_integration.log` and `ros_gazebo_final/`. Earlier integration defects and their logs remain retained. The environment is ROS 2 Jazzy/Nav2 1.3.13 and Gazebo Harmonic on Ubuntu 24.04 WSL.

Long-duration navigation, closed-region unreachable goals, stale-TF injection and ROS bag replay remain NOT RUN scope extensions. Source/release review remains root-owned. Upstream Nav2 default-derived configurations retain version/license attribution under `ros2_navigation/third_party/`.

## Review corrections

See `../08_TEST_RESULTS/robotics/DEFECTS_AND_LIMITATIONS.md` for defects, preserved failing logs, changes and scope boundaries. Static Python checks pass. Master review status and release packaging remain root responsibilities.
