# Validation and limitations

Public source snapshot prepared on 9 November 2026 from the reviewed 28–29 October
implementation checkpoint `37b38317c38829d685ba0c4c430984eea5f19fa6`.
Fresh publication-check results are in `publication_checks.json` alongside this document.
GitHub Actions reports the actual remote result separately.

## Recorded engineering qualification

Ten areas (A/B/C/E/F/G/H/J/K/L) met their finite, documented software/simulation
requirements. This is not full industrial qualification. D and I remain research
prototypes with failed quality gates, despite passing implementation tests.

| Area | Recorded result | Limit |
|---|---|---|
| FPGA | Golden vectors, UART/AXI regressions, four detected mutations, six identical replay artifacts | No physical board, timing closure, CDC sign-off or confidential-RTL equivalence |
| Measurement | Analytic / Monte Carlo cross-check and notebook; software PTP packet exchange | No accreditation; shared host clock is not independent-clock synchronization |
| G | Deterministic C++ navigation and sanitizer evidence | Planar ideal sensors and synthetic position observations |
| H v2 | Three new ten-goal Gazebo missions; max position errors 0.0088 / 0.0137 / 0.0095 m against 0.05 m limit | One declared room/route/sensor/physics configuration; v1 historical 0.058386 m FAIL retained |
| D | Historical deduplicated recall 80% versus 85% target: FAIL | Later observed-data re-split is not fresh holdout evidence |
| D research cycle 3 | KSDD2 nested training-only recall 80.1%, precision 66.8%: FAIL | Separate local research experiment, not the shipped logistic baseline; official test remained sealed |
| I | 138 implementation tests passed in completion session | Tests do not imply successful trajectory estimation |
| I V1 | VIO ATE 0.195580 m versus VO 0.134484 m; ratio about 1.455 exceeds 1.2: FAIL | Observed development data |
| I V2 | Stereo refinement leaves 0.2889 px against its 0.25 px gate: FAIL | Initialization rejected, no trajectory score |
| I fresh MH_02 | Frozen evaluation rejected moving initialization: FAIL | Trajectory metrics NOT RUN |
| J / K | PyBullet arm mission; MQTT/SQLite/HTTP and software fault tests | Educational arm, local telemetry; no production scale claim |
| L | Public-model 64-waypoint PX4/Gazebo mission and calibrated depth replay; simulated coverage 96.3% within 4 cm | Depth-based reconstruction, not RGB-only photogrammetry or real inspection accuracy |

For H, explicitly select `config/slam_params_v2.yaml` as documented in the module;
the default remains v1. Historical failures were not overwritten to make v2 pass.

## Evidence boundaries

The original complete source archive passed 17 aggregate extraction checks and all
seven subset verifications. The robot subset reported 26 passed / 1 skipped (the
installed-copy comparison requires a colcon build). Actual full H missions were
separately built and run. Publication layouts are checked again because directories
and dependencies changed. Current public tests cover the public files.

Original raw bags, licensed datasets, research trial outputs and operational review
transcripts remain local. Historical paths mentioning `08_TEST_RESULTS` in module
notes describe that private evidence workspace or outputs to generate locally; those
files are not claimed to be downloadable here. Current public evidence is selected
and sanitized. Source hashes are recorded in `SOURCE_PROVENANCE.json`.

Claude Code coordinated implementation and independent review agents, with actual
model metadata `claude-fable-5-1` and High effort requested. Review corrections were
applied and affected tests rerun. This was AI review, not third-party certification.
