# H qualification contract — 2026-10-26

These criteria are fixed before execution. A missing result is NOT RUN, a failed assertion is FAIL, and an unavailable prerequisite is BLOCKED. Hardware remains unverified. Results belong under `08_TEST_RESULTS/robotics/qualification_20261026/` in unique run directories.

## Repeated navigation in actual Gazebo

Run the existing original rigid-body room/robot, ROS bridges, SLAM Toolbox, Nav2 and C++ velocity guard. Execute ten sequential goals in two loops. Require every action to succeed and independently measure map-frame endpoint error ≤0.3 m. Also capture the named robot's native Gazebo world pose, align map to world once at the stationary start with a rigid SE(2) transform, and require endpoint error against that simulator ground truth ≤0.35 m. Preserve both error measures separately. Require ≥120 simulated seconds from first to last recorded odometry, ≥12 m cumulative odometry distance, and all finite sampled LiDAR returns ≥0.22 m. Preserve maps, trajectory, per-goal timing/errors and process logs. This is a bounded endurance scenario, not an unlimited reliability claim or continuous collision proof.

## Controlled planner failures with the real Nav2 server

Use a deterministic synthetic occupancy grid and actual Nav2 Navfn/global costmap, with the robot outside a closed occupied ring and a free goal inside it. Require a reachable control goal to succeed, the enclosed free goal to abort with NO_VALID_PATH=208, and a missing goal-frame transform and a timestamp-stale dynamic goal transform to abort with TF_ERROR=202. A fresh transform for the same reachable coordinates must restore successful planning. The grid and transforms are explicit test inputs, not simulated Nav2 responses. These test request-frame faults; they do not establish the response to loss of every internal TF edge during motion.

## Record/replay integration

Record real mission topics with installed rosbag2. With Gazebo and Nav2 stopped, replay the recorded bag twice into an isolated ROS domain. Require nonzero scan, odometry, transform, clock and safe-command streams. For every recorded topic require received message count and ordered raw CDR SHA256 to exactly match the original bag. Save both reports and bag metadata. This proves transport/content replay, not bit-identical re-execution of asynchronous SLAM or control.

## Qualification findings and added regression criteria

The initial turning failures required a physical-model check. A native Gazebo test applies the same forward/turn/stop commands and requires maximum absolute body roll/pitch ≤0.2 rad. The original model failed at 0.50884 rad; moving the rear battery COM to (-0.05, 0, -0.02) m relative to the body placed it inside the support triangle and passed at 0.02172 rad. The mission also enforces this tilt bound. Wheel radius and track-width parameters were not calibrated to ground truth or changed.

An additional experiment established that upstream planning can use a cached latest robot transform during publisher outage. The planner characterization reports this limitation explicitly; it is not a stale-transform safety pass. The custom C++ guard now independently requires an odom→base_footprint transform no older than 0.3 s and rejects future timestamps. Its real ROS test keeps commands/scans fresh while removing, aging or future-dating that TF, requires zero output by 0.6 s, then requires recovery with fresh TF. No physical safety certification is implied.

## Process and resource bounds

Only one heavy Gazebo job runs at once. A mission has an overall 720-second wall-clock bound and individual goals a 90-second bound. Each child stack has an owned process group and bounded cleanup. Evidence directories must be new, preserving previous failures. No hardware, publication, external services or host network settings are involved.

## Fable follow-up contract — fixed before new runs, 2026-10-26

The installed SimpleGoalChecker defaults are now explicit: xy tolerance 0.25 m, yaw tolerance 0.25 rad, stateful true. Existing endpoint thresholds remain unchanged. Additionally, every completed goal must have aligned estimated-versus-simulator position-vector discrepancy <=0.05 m and wrapped yaw discrepancy <=0.10 rad. These are localization agreement checks, not the difference between two scalar distances to the goal. One initial stationary SE(2) alignment is saved and used unchanged for all goals. Pair the latest map pose with the nearest native simulator pose, require timestamp skew <=0.02 s, and save both stamps and poses. At 0.3 m/s the maximum permitted skew contributes <=0.006 m positional displacement.

The native balance probe commands 0.3 m/s (the full guard clamp), followed by -0.8 rad/s turning and a stop; maximum absolute roll/pitch remains <=0.2 rad. Gazebo DiffDrive linear and angular acceleration limits are explicitly [-0.5,+0.5] m/s^2 and rad/s^2 respectively, matching the smoother. Parameters are documented in the [Gazebo Sim 8.15 DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html). They bound command evolution, not every rigid-body acceleration or certified stopping distance.

Evidence goes into new `08_TEST_RESULTS/robotics/fable_followup_20261026/` directories. Rebuild, guard tests, geometric metric tests, native balance, a fresh ten-goal recorded mission and both exact-message replays are required. Historical qualification logs stay unchanged. The integrated mission demonstrates navigation in this room; its route is not a controlled close-obstacle stop test. The real ROS guard test with synthetic 0.2 m scan returns covers commanded stopping, not an integrated physical obstacle approach or Nav2 collision-monitor qualification.

Follow-up diagnostic correction: the balance probe now measures elapsed time from the first received native pose and requires actual native travel >=0.8 m, absolute turn >=3 rad, and average straight cruise speed in [0.25,0.35] m/s during seconds 4–6. These prevent a stationary or late-subscribing run from passing solely on low tilt. Run `balance_01` is superseded by this stronger probe; it is not evidence for movement coverage.

Fresh mission runs `endurance_01` and `endurance_02` failed the unchanged 0.10 rad yaw criterion. Run 02's two-second post-goal diagnostic retains ~0.164 rad error. The installed SLAM Toolbox 2.8.5 parameter `check_min_dist_and_heading_precisely` was False. Its [versioned source](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp) shows the default translation-only gate skips pure-turn scans. Enable this supported parameter to consider either heading or translation; keep travel thresholds, speeds and all acceptance limits unchanged. Subsequent mission evidence tests that correction rather than relaxing the failed criterion.

The unchanged position criterion exposed remaining drift in runs 03 (0.06036 m) and 04 (0.05357 m), while yaw passed. The final candidate uses 0.05 m / 0.03 rad scan-admission thresholds and a 0.005 m correlation grid with 0.03 m smear deviation. These tune the simulated low-noise scene, not a claim of robustness to arbitrary real sensors. Sensor rate/ray count, route, physical motion limits and acceptance bounds remain unchanged. Full mission outcomes, including failed attempts, are preserved.

Balance also checks measured native speed in 0.1-second windows centred at elapsed 2.3 s and 7.3 s: each must lie in [0.08,0.22] m/s, between rest and full cruise. This verifies finite ramping/braking rather than merely accepting acceleration XML fields. The guard's zero output is a requested stop; with bounded DiffDrive deceleration, physical motion continues while braking. The guard test does not assert instantaneous rigid-body stopping.

Contact-model diagnosis supersedes those tuning trials. The default DART/ODE contact detector produced 7.379 rad native rotation for an eight-second -0.8 rad/s command, against 6.4 rad from independent kinematics. Rounded ellipsoid tires changed that response but increased tilt and straight-line slip, so the original cylinders/inertia/radius/track were restored. DART with its supported Bullet collision detector yielded 6.402 rad and 0.29997 m/s cruise. This changes contact detection within DART, not the odometry formula or simulator pose source; the precise ODE internal contact mechanism is not proven. [Gazebo Sim 8 physics configuration](https://gazebosim.org/api/sim/8/physics.html) documents this option.

The final candidate restores original 0.1 m/0.1 rad scan thresholds and upstream correlation defaults, retaining rotation-aware scan admission. Native balance additionally requires integrated absolute rotation to be within 0.3 rad of 6.4 rad, derived from the commanded -0.8 rad/s for 8 s with symmetric acceleration/braking and enough settling time. The 0.3 rad bound allows command scheduling/contact deviations; it would reject the observed ODE overshoot. Existing pose-error, endpoint, timing, distance, clearance and tilt limits are unchanged. A fresh final balance and ten-goal mission test this physical correction.

## Final recorded outcome and status distinction

`fable_followup_20261026/endurance_08` completed all 10 goals and original navigation/endpoint/clearance/tilt/duration/distance checks: PASS. Its added 5 cm / 0.10 rad pose check also passed for that run. The same final physical/SLAM configuration failed the 5 cm diagnostic in run 07, so **repeatable 5 cm accuracy remains FAIL / not established**; no threshold was raised. This additional accuracy target is distinct from the original required navigation functionality. Both recorded replays of run 08 passed exact per-topic count/order/content for 106333 messages across 8 topics. Replay does not rerun SLAM/control or prove deterministic cross-topic callback order. The final evidence/source manifest and review disposition are saved in the follow-up directory.

### Status record — contract wording and the "diagnostic" relabel (third review, 2026-10-28)

The 0.05 m / 0.10 rad agreement check entered this document as a requirement, not as an observation. The section heading above reads verbatim: "Fable follow-up contract — fixed before new runs, 2026-10-26", and its contract sentence reads verbatim: "Additionally, every completed goal must have aligned estimated-versus-simulator position-vector discrepancy <=0.05 m and wrapped yaw discrepancy <=0.10 rad." After run 07 failed that requirement (goal 3, 0.05838625 m, `endurance_07/acceptance.log`), later text relabelled it. The previous paragraph of this file says "failed the 5 cm diagnostic in run 07", and REVIEW_HANDOFF.md states verbatim: "**Additional 5 cm diagnostic repeatability remains FAIL / not established.**" (its "Fable follow-up — previous candidate" section). The relabel was inappropriate: it would soften a predeclared gate after a failure had been observed, which is exactly the kind of post-hoc reinterpretation this contract forbids. Both texts are kept in place as history. The record stands as follows: the harness verdict for the combined follow-up contract (navigation AND pose agreement AND metric checks) is FAIL, because run 07 is a counterexample at the final configuration; no threshold was changed; and the original navigation scope PASS of run 08 is reported through the separate `original_scope_status` key described below, not by renaming the added target.

Evidence breadth is also recorded honestly (committee item H-REP-1): at the final physics/SLAM configuration (Bullet contact detector plus rotation-aware scan admission) only run 08 completed the ten-goal route, because run 07 was stopped by the per-goal pose assertion in force at that time. Original-scope evidence at that configuration is therefore a single completed mission. `qualification_20261026/endurance_06` passed the original scope on the superseded physics/SLAM configuration and does not add a second sample at the final one. No additional mission is claimed by this documentation change.

Report keys. `write_and_require_acceptance` in `qualification_report.py` now writes three verdicts into every `acceptance.json`, with a `status_definitions` entry repeating the rules. `status` is unchanged: PASS only if ten goals completed AND every goal's `pose_agreement_pass` is true AND all four metric checks (body tilt, duration, distance, clearance) passed; otherwise FAIL, and an AssertionError is raised after the file is written. `original_scope_status` is PASS only if ten goals completed and all four metric checks passed; the per-goal endpoint limits (≤0.3 m estimated, ≤0.35 m aligned simulator truth) are asserted in `test_endurance.run_route` before a goal is appended, so a goal present in the report already met them. `pose_agreement_status` is PASS (ten goals, all true), FAIL (any completed goal false, the key missing/non-boolean, or more than ten goals), or NOT RUN (fewer than ten goals and none failed, including zero goals). A fatal execution failure that reaches `preserve_acceptance_failure` with no report writes `status: FAIL`, `original_scope_status: FAIL` and a `pose_agreement_status` derived from any saved `goals.json` by the same rule; an existing valid report keeps its recorded statuses and only gains `status: FAIL` plus the failure text. For a navigation-complete mission that misses the added target the file therefore reads `status: FAIL`, `original_scope_status: PASS`, `pose_agreement_status: FAIL`. Classification tests for these combinations are in `test_qualification_report.py`; they read the written file back.

## Evidence safeguards after the second review

`run_qualification.py` compares the source SDF and installed package SDF SHA256 before starting any stack. It writes both paths/hashes and PASS/FAIL to `world_preflight.json`, rejects mismatches with a rebuild instruction, and archives the verified installed bytes. This closes the gap between the balance probe's source world and the mission's installed world. Do not rebuild or edit the world during a run.

`mission_metrics` now computes measurements without asserting limits. `write_and_require_acceptance` writes the complete per-goal/metric report first, then rejects failed tilt, duration, distance, clearance or pose agreement. Missing/nonfinite metrics cannot pass; nonfinite metric values become JSON null with their keys recorded. Route/action/endpoint exceptions now also produce a partial FAIL acceptance report from saved goals before lifecycle cleanup; the original exception propagates. A report written by metric evaluation is preserved with its full trace. Focused real-file tests and re-evaluation of saved run 08 metrics passed. The latter is an archival report check, not another mission.

The independent minimal lifecycle-configured SLAM probe in `fable_review2_20261026/explicit_slam_parameter_probe.log` records the exact node/name query and `Boolean value is: True`. It uses literal overrides extracted from the unchanged launch, and saves package version and upstream 2.8.5 source. It does not retrospectively prove a historical node's runtime state; historical settings and execution remain supported by their launch/configuration and logs. Original navigation acceptance and the unresolved additional 5 cm repeatability result are unchanged.

A later focused failure-path pass extends preflight reporting to missing/unreadable world files and adds `qualification_processes.py` to save `shutdown.json` after owned cleanup even on client timeout. Shutdown reports distinguish client failure, stack crash detection, and cleanup failure. The real timeout test verifies the child exits and its owned process group no longer exists. Seven lightweight tests pass; no new mission, historical balance-world hash, or SLAM negative-control experiment is claimed. Filesystem write failures or abrupt host termination cannot guarantee report persistence.


## Version 2 configuration (2026-10-28)

A separately versioned SLAM Toolbox configuration was developed and qualified prospectively; no
historical text, threshold, route, physics, alignment policy or v1 verdict above changes. Evidence:
`08_TEST_RESULTS/completion_20261028/h_slam_v2/` (`PROTOCOL.md`, `PROSPECTIVE_PROTOCOL.md`,
`SELECTION.json`, `DEVELOPMENT_RESULTS.md`, `freeze.json`, `missions/`, `STATUS.json`, `REPORT.md`).

Protocol. The already-observed bags of runs 07 and 08 were declared development data. Each was
filtered to its input topics with the recorded `map->odom` correction removed and replayed, with exact
per-topic ordered-CDR input integrity and an initial-gauge check, into a fresh `async_slam_toolbox_node`
per candidate; the unchanged per-goal metric (`pose_metrics.aligned_pose_error` under the recorded
initial alignment) was evaluated at each recorded goal stamp. A baseline replay first had to reproduce
the historical values within 1e-6 (observed deviation <= 4.4e-16 m). Six candidates and the selection
rule (eligible only if all 14 goals pass 0.05 m / 0.10 rad on both bags; smallest maximum position
error; yaw tie-break; lower index) were declared and hashed before the first replay. Selected:
`c1_buffer100`, i.e. `scan_buffer_size: 100` (Karto's buffer rule 10 m / 0.1 m), with a 14-goal maximum
of 0.0097 m / 0.0045 rad (v1 baseline 0.0584 m). Candidate `c3` (0.05 m / 0.05 rad scan admission
alone) was ineligible at 0.0805 m on bag 08.

Source changes. `config/slam_params_v1.yaml` carries the frozen v1 overrides previously inlined in
`launch/gazebo_navigation.launch.py` (byte-identical to the hashed harness overrides);
`config/slam_params_v2.yaml` adds only `scan_buffer_size: 100`. The launch declares `slam_params_file`
with the v1 file as default, so default behaviour is unchanged; `run_qualification.py --slam-params-file`
passes it through and archives the used file as `slam_params.yaml` with `slam_configuration.json`;
`CMakeLists.txt` installs `config/`; `test_slam_configuration.py` (six pure-Python tests) checks parsing,
the frozen typed v1 values, that v2 differs only in the selected key, the launch argument, the runner
pass-through and installed-equals-source. Build, 27 light tests and ruff passed; SHA-256 of every
relevant source and installed file was frozen before the first mission.

Outcome. Three fresh prospective ten-goal missions with v2 (`mission_v2_01..03`, headless, serial,
unchanged limits) all read `status: PASS`, `original_scope_status: PASS`, `pose_agreement_status: PASS`:
maximum aligned position errors 0.0088 m, 0.0137 m, 0.0095 m and maximum yaw errors 0.0060, 0.0079,
0.0027 rad (limits 0.05 m / 0.10 rad); 138.10 / 136.38 / 135.92 simulated seconds, 13.27 / 13.28 / 13.25 m,
minimum clearance 1.163 / 1.168 / 1.164 m, peak tilt 0.053 / 0.057 / 0.063 rad; world preflight and
owned-process cleanup PASS in each. A parameter dump of the live node during mission 1 shows
`scan_buffer_size: 100`. Under the fixed rule (all three must pass) the **v2 repeatability verdict is
PASS**. Both exact-message replays of the mission 1 bag matched all 111,724 messages across 8 topics.

Limitations. Simulation only. Bags 07/08 chose v2 and cannot qualify it. A finite passing set of three
missions supports only the declared simulated conditions (this room, route, physics, sensor model and
host) and is not a general accuracy guarantee. **v1 verdicts are unchanged: run 07 remains FAIL for v1
and v1 5 cm repeatability remains FAIL / not established.** v2 is selected explicitly through
`slam_params_file`; nothing above is relabelled.
