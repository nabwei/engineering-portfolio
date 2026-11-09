**Public status: SLAM v2 passed three prospective simulated missions; select v2 explicitly. The default remains v1.**

See the repository [validation summary](https://github.com/nabwei/engineering-portfolio/blob/main/docs/VALIDATION.md) for current limits. Historical evidence paths below refer to local generated outputs.

# H — ROS 2 navigation integration

Local build, real ROS watchdog tests, the C++ backend, and the actual Gazebo/SLAM/Nav2 backend PASS. The original short-mission evidence is in `../../08_TEST_RESULTS/robotics/ros_gazebo_final/`. Updated qualification evidence is in `../../08_TEST_RESULTS/robotics/qualification_20261026/`; earlier failed runs are retained separately. The initial missing-environment probe is historical evidence.

Selected integration target: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic, the supported pairing documented by [Gazebo](https://gazebosim.org/docs/latest/ros_installation/). Required packages: `ros-jazzy-ros-base`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`, `ros-jazzy-ros-gz`, `ros-jazzy-robot-state-publisher`, and `python3-colcon-common-extensions` from the official ROS/Ubuntu repositories.

Run `python3 check_environment.py` after sourcing the ROS installation. Exit 2 means blocked; exit 0 only verifies tool/package presence, not integration correctness.

Two explicit backends share the ROS interface:

- `navigation.launch.py`: custom C++ node wrapping the independently tested G kinematic core; publishes scan, odometry and TF. This backend is software simulation without Gazebo dynamics.
- `gazebo_navigation.launch.py`: original differential-drive SDF, room and obstacles in actual Gazebo; software-rendered LiDAR, ros_gz bridges, simulated clock and rigid-body wheel dynamics. Custom C++ `velocity_guard` checks command/scan freshness, clamps speed and suppresses forward motion toward a close front obstacle. It is a demonstration guard, not a certified safety controller.

Both launch SLAM Toolbox and Nav2. The acceptance runner derives a configuration from installed Nav2 defaults, selects its RegulatedPurePursuit controller, aligns frame names/laser height and saves the exact YAML beside logs. Each run uses a unique Gazebo transport partition and ROS domain 187. Run these acceptance jobs sequentially: domain 187 is shared by these runners and must be otherwise unused. Cleanup performs managed lifecycle shutdown, signals the launch leader, then removes remaining processes only in its owned process group.

## Build and run in Ubuntu/WSL

```sh
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths . --packages-select portfolio_navigation --executor sequential
source install/setup.bash
python3 check_environment.py
python3 test_velocity_guard.py
python3 run_integration.py --backend core --output ../../08_TEST_RESULTS/robotics/ros_core
python3 run_integration.py --backend gazebo --output ../../08_TEST_RESULTS/robotics/ros_gazebo
```

Use system Python after sourcing ROS; the portfolio scientific virtual environment does not supply ROS Python bindings. The package consumes `../robot_core` directly, so retain the directory structure in a release. The integration runner launches the full stack, waits for actual scan/odom/map data, verifies TF, submits a Nav2 goal, requires successful action completion and saves logs. Failure is a nonzero exit with no fallback backend.

`test_velocity_guard.py` uses real ROS messages against the compiled C++ process to check speed clamping, a near obstacle, missing scans, stale scan timestamps and future timestamps. `test_navigation.py` also runs against an already-launched stack using `--backend core|gazebo --output report.json`.

The Gazebo dynamics backend uses `map -> odom -> base_footprint -> base_link -> laser`. Base height is 0.14 m and laser height above the body is 0.12 m. The kinematic backend uses `map -> odom -> base_link -> laser` with coincident planar laser coordinates. Never run both backends in the same domain with identical robot topics.

Acceptance requires 361-beam scan/odometry data, a SLAM map containing free and occupied cells, actual TF lookups, active lifecycle nodes, successful Nav2 action completion, goal error ≤0.3 m computed from the SLAM/TF map-frame pose, sampled LiDAR clearance ≥0.22 m, planner rejection of an outside-map goal with error 204, and clean managed shutdown. The Gazebo run achieved 0.2412 m goal error and 0.6531 m minimum sampled clearance. This checks the estimated pose separately from the action's success flag; it is not an independent simulator-ground-truth localization error. These simulation checks do not establish physical robot accuracy or continuous-time collision guarantees. Two Gazebo mission runs passed; exact timestamps/trajectories are not promised bit-for-bit across scheduling variations.

Saved outputs include map PGM/YAML, odometry JSON, resolved Nav2 parameters, acceptance/shutdown JSON and process logs. `visualize.py <result-directory>` produces a scientific PNG with the SLAM map and odometry shown in their own coordinate frames. It requires the scientific Python environment with Matplotlib. No map/odom coincidence is assumed for plotting.

The original short acceptance mission does not establish endurance or fault/replay qualification. Additional executable qualification and its finite criteria are in [QUALIFICATION.md](QUALIFICATION.md), with an [Arabic study supplement](STUDY_QUALIFICATION_AR.md). Independent review/public release remains a separate master gate. Replace the reserved maintainer address in package.xml before publication; upstream license notices and config provenance are retained in `third_party/`.

## Additional qualification

Use the same sourced system ROS Python environment and a freshly built package. Each output directory must be new. Run one heavy Gazebo process at a time; these scripts reserve ROS domains 187 (mission), 188 (controlled planner), 189 (bag replay), and 190 (standalone guard test). Do not use those domains for other tasks concurrently.

```sh
python3 test_physics_balance.py --output ../../08_TEST_RESULTS/robotics/my_balance
ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 python3 test_velocity_guard.py
python3 test_planner_qualification.py --output ../../08_TEST_RESULTS/robotics/my_planner
python3 run_qualification.py --output ../../08_TEST_RESULTS/robotics/my_endurance
python3 test_bag_replay.py --bag ../../08_TEST_RESULTS/robotics/my_endurance/mission_bag --output ../../08_TEST_RESULTS/robotics/my_replay
```

The endurance test uses Gazebo Transport 13 / messages 10 Python bindings installed with this Harmonic environment for named simulator truth; these are separate from ROS TF. It records ten sequential goals, estimated and simulator endpoint errors, body tilt, travelled distance, scans, and rosbag2 SQLite streams. The body battery COM is offset rear/down to remain inside its wheel/caster support triangle. The native balance regression protects against the original tipping defect. No ground-truth pose replaces odometry or SLAM.

The physical backend's C++ guard now requires a fresh `odom -> base_footprint` transform (≤0.3 s, no future timestamp), as well as fresh commands/scans. This fixed frame pair is specific to the Gazebo backend. The controlled planner test demonstrates why the guard is necessary: upstream planning may accept a cached latest robot pose even after its publisher stops. Request-frame TF errors and actuator freshness are tested separately.

The configuration allows 2 seconds for action acknowledgment, uses a 0.35 rad/s rotation target and conservative acceleration limits, and processes SLAM scans at a 0.1-second minimum interval. The goal deadline and positional acceptance bounds remain unchanged. Replay compares ordered raw serialized content within each topic, not cross-topic callback order or deterministic re-execution of asynchronous SLAM.

The corrected `endurance_06` run passed ten goals over 133.16 simulated seconds and 13.6183 m of odometry travel. Maximum endpoint errors were 0.249663 m (SLAM estimate) and 0.247333 m (initially aligned simulator truth); peak body tilt was 0.0113903 rad. Both bag replays matched all 108,803 messages across eight topics exactly. Controlled planner tests, two real guard tests and the updated core-backend regression also passed. See [REVIEW_HANDOFF.md](REVIEW_HANDOFF.md) for the evidence inventory and finite limits. `visualize_qualification.py <result-directory>` generates the separate-frame scientific overview using Matplotlib.

References: [Gazebo moving robot](https://gazebosim.org/docs/harmonic/moving_robot/), [Gazebo sensors](https://gazebosim.org/docs/harmonic/sensors/). Plugin parameters were also checked against installed Jazzy package files.

## Fable review follow-up

The goal checker is explicitly pinned to 0.25 m / 0.25 rad. The qualification now additionally compares the full aligned position vectors (<=0.05 m) and wrapped yaw (<=0.10 rad) at timestamp-paired poses, with one saved initial SE(2) alignment. `python3 -m unittest test_pose_metrics` covers equal scalar endpoint distances with opposite position errors, and a known rotation/translation with angle wrapping. `pose_agreement.png` adds separate position/yaw plots to the ordinary overview.

SLAM Toolbox 2.8.5's default translation-only scan gate skipped pure-turn updates. The Gazebo launch enables its supported `check_min_dist_and_heading_precisely` setting with the original 0.1 m / 0.1 rad update thresholds. DART now uses its supported Bullet collision detector with the original cylindrical wheel geometry; the previous default ODE contact detector produced excessive measured rotation. Scan-density/correlation tuning experiments did not remove the drift and were reverted; their failed evidence is retained.

DiffDrive now limits linear and angular acceleration/braking to ±0.5 in their respective SI units. The native balance probe commands the full 0.3 m/s clamp and requires measured movement, turning, cruise speed and finite speed ramps as well as low tilt. A guard zero command requests braking; it does not make the rigid body stop instantly. The mission route is not a controlled close-obstacle approach test. The near-obstacle guard test uses synthetic ROS scan input and validates its zero-command response, not an integrated Nav2 collision-monitor safety guarantee. See QUALIFICATION.md for exact bounds and primary implementation references.


Current follow-up evidence is `08_TEST_RESULTS/robotics/fable_followup_20261026/`. Run 08 passed 10/10 goals over 133.70 simulated seconds and 13.25849 m, with maximum estimated/truth endpoint errors 0.246281/0.272980 m. Its new aligned pose-vector error peaked at 0.04978855 m and yaw error at 0.04927382 rad. **Repeatable 5 cm localization is not established:** run 07 with the same final physical/SLAM settings failed at 0.05838625 m. Both results remain visible; no bound was relaxed. Full follow-up details and the additional diagnostic limitation are in REVIEW_HANDOFF.md.

Reading `acceptance.json` after the third review: every report now carries three verdicts. `status` is unchanged and strict (PASS only when ten goals completed, every goal met the added 0.05 m / 0.10 rad agreement target, and all four metric checks passed; a FAIL still raises after the file is written). `original_scope_status` is PASS only when ten goals completed and the tilt/duration/distance/clearance checks passed; the 0.3 m / 0.35 m endpoint limits are asserted per goal in the route before a goal is recorded. `pose_agreement_status` is PASS, FAIL (a failed/malformed goal or an overfull route) or NOT RUN (fewer than ten goals and none failed, including zero goals). A navigation-complete mission that misses the added target therefore reads `status: FAIL`, `original_scope_status: PASS`, `pose_agreement_status: FAIL`; nothing in the thresholds changed, and the added-target repeatability result stays FAIL. `status_definitions` inside the file repeats these rules. See the "Status record" subsection of QUALIFICATION.md for the contract wording history.

The second review added evidence safeguards in `qualification_report.py`: before launching, qualification requires the source and installed world SHA256 to match and records `world_preflight.json`; after the route, it saves complete `acceptance.json` before rejecting failed mission metrics. Missing/nonfinite metrics fail explicitly. `python3 -m unittest -v test_qualification_report test_qualification_processes test_pose_metrics` tests stale installation rejection and failure-report retention without Gazebo. A separate configured SLAM-node probe explicitly confirmed `/slam_toolbox` parameter `check_min_dist_and_heading_precisely=True`; its command log and versioned upstream source are in `08_TEST_RESULTS/robotics/fable_review2_20261026/`. These changes do not alter navigation settings or acceptance limits, and no new physical mission is claimed for this reporting-only fix.

Early route/action/endpoint failures also retain a partial FAIL acceptance report. Missing or unreadable source/installed worlds retain a FAIL preflight report. The qualification runner records shutdown and owned-cleanup status even when the client times out; errors continue to propagate. Seven focused tests, including an actual bounded child-process timeout, pass in `08_TEST_RESULTS/robotics/failure_paths_20261026/tests.log`. No additional full mission was run for these reporting changes.

## Version 2 SLAM configuration (2026-10-28)

The Gazebo launch now takes `slam_params_file` (default `config/slam_params_v1.yaml`, the frozen v1
values that were previously inlined, so default behaviour is unchanged). `config/slam_params_v2.yaml`
adds only `scan_buffer_size: 100`, selected by a predeclared offline rule on the already-observed
run 07/08 bags and then qualified prospectively in three fresh ten-goal missions that all passed the
unchanged 0.05 m / 0.10 rad per-goal pose-agreement contract (maximum 0.0137 m). Run it with
`python3 run_qualification.py --output <new dir> --slam-params-file "$(pwd)/config/slam_params_v2.yaml"`;
the runner archives the file used as `slam_params.yaml` beside `slam_configuration.json`.
`python3 -m pytest -q test_slam_configuration.py` checks the two files, the launch argument and the
installed copies. Evidence and limits (simulation only; v1 run 07 remains FAIL) are in
`QUALIFICATION.md` ("Version 2 configuration") and `08_TEST_RESULTS/completion_20261028/h_slam_v2/`.
