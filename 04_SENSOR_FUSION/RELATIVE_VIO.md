# Relative-pose visual–inertial estimator

This implementation replaces the failed accumulated-absolute-VO update for the
new qualification experiment. `run_euroc.py` remains an explicitly historical
baseline. The default `run_relative_euroc.py` performs autonomous quiet-start initialization,
metric stereo odometry and a 21-error-state stochastic-clone ESKF. It never reads
reference position, orientation, velocity or bias to initialize or update the
estimator. Outside the estimator, the command-line driver uses only the first
reference timestamp to label the previously observed development prefix;
reference states are read by the scorer after estimation. A file-based replay
test deletes the reference CSV entirely and still runs the estimator successfully.

Generic replay identifies the input format as `EUROC_ASL` and its provenance as
`UNVERIFIED_INPUT_FILES`: matching the layout does not establish a public dataset
identity. The reserved runner separately records the supplied acquisition
manifest path and SHA-256 in `experiment_manifest.json`. Public-source claims
must cite that acquisition evidence; the runner does not reverify every image
against the manifest or certify an official dataset byte identity.
`reset_policy` states the absence of a state-reset code path; it is not a measured
reset counter. Earlier reports' literal `state_resets: 0` described that same
source property and should not be interpreted as instrumentation.

## Frames, state and covariance

The nominal inertial state is `(p_WB, v_WB, R_WB, b_a, b_g)`. The right-local
orientation error satisfies `R_true = R_nominal Exp(delta_theta)`. A six-error
previous-pose clone is appended to the 15 inertial errors. Cloning uses
`P_aug = A P_current A.T`; keeping cross covariance is essential. A relative
measurement cannot remove uncertainty in a common world translation.

For clone `(p_c,R_c)` and current `(p,R)`, predict
`t = R_c.T (p-p_c)` and `R_rel = R_c.T R`. The residual is measured translation
minus `t`, followed by `Log(R_rel.T R_measured)`. Nonzero Jacobian blocks are
`H_t,p=R_c.T`, `H_t,pc=-R_c.T`, `H_t,theta_c=[t]x`,
`H_R,theta=I`, `H_R,theta_c=-R_rel.T`. A finite-difference test perturbs actual
poses independently to verify these signs and frames.

Propagation uses midpoint attitude for specific force and the frozen linear
model, Van Loan discretization, and the augmented transition `diag(Phi,I6)`.
Correction uses a Joseph covariance update and right-local orientation reset
Jacobians on both the current state and clone. The six-dimensional innovation
gate is 22.458 (the existing chi-square gate); rejected measurements do not reset
state or covariance. There is no recovery reset in this estimator.

## Camera observations and limits

The additional `vio/flow_stereo.py` frontend uses subpixel forward/backward LK,
stereo depth, robust PnP and a current-right-image geometric check. Descriptor
reacquisition seeds LK after a tracking gap; four-view validation rejects a
false1.34m reacquisition observed during V1 development. Actual failed frames
stay in the inertial trajectory. `run_flow_development.py` uses this frontend;
all V1 frames are development now, including the original reserved prefix.
The ORB full replay's1.199866m reserved ATE failure remains historical evidence.

The existing ORB/stereo/PnP frontend provides calibrated metric motion. Body
motion is formed from two body poses, so camera-to-IMU extrinsics are included.
In the default disjoint filter, only disjoint image pairs update it:
`(0,1), (2,3), ...`; after a lost
frame the pending pair spans the gap. The previously accepted image in one pair
is not reused in another update. This avoids the direct shared-image error
between adjacent displacement measurements, while retaining the state/clone
correlation. The standalone VO comparison still uses all successful images.

`run_flow_development.py --fusion noisy` instead selects
`NoisyFramePoseFilter`: every adjacent successful pair contributes, with explicit
latent per-image pose noise. Persistent covariance has 21 states; a fresh image
adds six temporary errors during correction. The fixed share is beta=0.5 and
the pair noise budget is unchanged. See [NOISY_FRAME_MODEL.md](NOISY_FRAME_MODEL.md)
for the generative assumption, rejection/marginalization semantics and limits.

The diagonal observation covariance (translation sigma 0.01 m, rotation sigma
0.005 rad) is an engineering noise model, not a covariance derived from PnP
feature residuals. Unmodeled calibration error, visual bias and environmental
correlations remain. Neither this implementation nor a PSD covariance alone
proves statistical consistency. This is loose coupling, not landmark-based
MSCKF, optimization-based VIO, or a GTSAM equivalence claim.

## Initialization

`ReplayConfiguration` selects the frontend, measurement sigmas, initializer and
filter factory.
`vio/replay_initialization.py` also provides a moving-start path: calibrated
metric visual body poses from the first acquired5second window align gravity,
velocity and biases with bracketed IMU. Weak excitation fixes accelerometer bias
to its zero prior mean and preserves variance1m²/s⁴; it is not a calibrated zero.
The conditional-fit Jacobian supplies velocity/attitude/bias cross covariance,
plus declared independent allowances. This engineering prior is PSD, without a
claim of full posterior recovery. See [DYNAMIC_INITIALIZATION_PROTOCOL.md](DYNAMIC_INITIALIZATION_PROTOCOL.md).
The filter installs that prior before cloning and excludes every initialization
image from subsequent updates. A real-image test deletes the truth CSV and
verifies the strict timestamp separation. The reserved runner tries quiet start
first and catches only its explicit unavailable-window exception before moving
initialization; a rejected moving fit does not trigger a reset or truth fallback.
The reviewed moving covariance computes perturbed gravity tilts in the aligned
baseline frame, avoiding a spurious world-yaw/bias covariance from independently
realigning each perturbation. A nonidentity 110-degree analytic regression failed
before that correction and passed afterward.

Search only the first ten seconds of IMU for a one-second interval with at least
100 samples, gyro standard-deviation norm <=0.02 rad/s, accelerometer
standard-deviation norm <=0.25 m/s², mean gyro norm <=0.15 rad/s, and acceleration
magnitude within 0.4 m/s² of gravity. Gravity alignment defines roll/pitch; yaw
and position are arbitrary. Mean angular rate estimates gyro bias. Only the
radial accelerometer bias is initialized; tangential bias and tilt cannot be
separated at rest. Initial velocity is zero. Constant velocity, sufficiently
small constant angular rate, and constant specific-force acceleration may
masquerade as rest, sensor bias or tilt. The quiet-start operating assumption
must hold; these checks cannot prove rest. No qualifying interval produces an
explicit failure.

Development record: the first variance bound of 0.20 m/s² rejected all early
quiet intervals in the already observed prefix. It was changed to 0.25 using
only prefix IMU statistics (quiet window near 1.5 s); the search was bounded to
ten seconds to prevent silently initializing near the sequence end. No reserved
trajectory errors were used. An exact initialization/camera timestamp equality
also exposed an empty integration interval; the runner now skips that interval.

## Reproduction and evaluation

From `04_SENSOR_FUSION`, using the pinned project dependencies:

```sh
OPENBLAS_NUM_THREADS=1 python -m pytest -q
OPENBLAS_NUM_THREADS=1 python run_relative_euroc.py \
  --mav0 datasets/V1_01_easy_mirror/mav0 --frames 400 \
  --output ../08_TEST_RESULTS/sensor_fusion/new_development_run
```

For the fixed experimental candidate on the already observed full V1 sequence:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python run_flow_development.py \
  --mav0 datasets/V1_01_easy_mirror/mav0 --frames 0 \
  --fusion noisy --initialization moving \
  --output ../08_TEST_RESULTS/sensor_fusion/new_noisy_moving_run
```

Development options are `--fusion {disjoint,noisy}` and
`--initialization {quiet,moving}`, defaulting to disjoint/quiet. The separate
`run_flow_evaluation.py` fixes noisy fusion and `automatic_start`; it takes
`--mav0`, `--output`, `--freeze` and `--acquisition-manifest`. There is no
`--initialization automatic` CLI flag. Its pre-evaluation freeze must match the
actual source hashes, beta and original observation sigmas. It tries quiet start,
then moving start only when the quiet window is unavailable. Initialization can
fail before any trajectory/report is produced; keep the nonzero exit, execution
log and experiment manifest, rather than interpreting missing metrics as success.

Use `--frames 0` only after acquiring the entire sequence. Outputs must be fresh
directories; prior evidence is never overwritten. Source data are not packaged
or redistributed. `report.json` records initialization, per-frame gates,
covariance eigenvalues, biases and separate development/reserved metrics.
`trajectory.npz` contains integer timestamps and all inertial poses, including
frames where camera tracking failed. Scoring retains those failed-camera frames
for VIO error. VO/VIO comparison uses their common successful-image timestamps
and reports missing coverage. A regression test injects a large inertial error
only at a missing-camera frame to ensure it cannot be hidden from the score.
The trajectory and an explicit unscored report are saved before scoring, so a
scoring exception does not discard the replay. A segment without one-second
timestamp pairs records NOT RUN. The command exits nonzero unless the requested
development (400) or reserved/full (0) segment passes its numerical criteria.

ATE fits only one rigid SE(3) transform, never scale. One-second RPE compares
actual relative SE(3) motions, reporting translation meters and rotation degrees.
There is also the historical one-sample displacement metric, clearly named.
The predeclared acceptance and data discipline are in
`QUALIFICATION_PROTOCOL_20261026.md`. Development run02: 371 evaluated frames,
ATE 0.0698739 m versus VO 0.0817181 m; one-second translation RPE 0.0555565 m,
rotation RPE 0.711330 degrees. This is development evidence, not reserved-segment
qualification. The old raw 16 m error used a reference-initialized world frame
and a different time span; it is not directly comparable to this aligned ATE.

## Frozen candidate outcomes

All V1 rows below are development evidence. Full-sequence SE(3) alignment fits no
scale; each run has its own initialization window and truth-overlap interval.

| Actual-image run | Truth frames | VIO ATE (m) | VO ATE (m) | 1 s VIO RPE (m / deg) | Result |
|---|---:|---:|---:|---:|---|
| Noisy, quiet start | 2842 | 0.214191 | 0.135218 | 0.049444 / 0.647972 | FAIL |
| Noisy, moving start, gauge fix | 2770 | 0.195580 | 0.134484 | 0.052672 / 0.708987 | FAIL |

Both pass the absolute limits but fail the frozen criterion that common-frame
VIO ATE be at most 1.2 times VO ATE. Quiet replay exactly reproduces the cached
trajectory and diagnostics. Moving replay estimates all 2,790 post-initialization
frames: four visual failures remain in the fused trajectory, with 2,783 accepted
updates and two rejected updates. `reset_policy` describes source behavior,
not a measured reset count. Original failures and bounded calibration grids are
preserved; no configuration passed the grid and none was selected from it.

Local evidence under `../08_TEST_RESULTS/sensor_fusion/qualification_20261026/`:
`noisy_frame_actual_v1_01/actual_vs_cached.json` and
`noisy_moving_gauge_fixed_v1_01/validation_summary.json`, each with plots and
source snapshots. Evidence/data are excluded from source-only release archives.

The frozen V2 attempt `v2_reserved_01` failed on the first stereo pair during
moving initialization, before truth-state scoring: **execution FAIL; trajectory
metrics NOT RUN**. All 4,571 acquired input files had passed SHA verification.
Post-test inspection found valid tracks rejected by the fixed 0.5-pixel vertical
stereo residual gate (median about 1.64 pixels), not a lack of detected texture.
The published ORB-SLAM3 rectification also rejects the same pair; computed
rectification rotations agree with it within about 6e-14. The declared-calibration
projection check is internally consistent, so no calibration-loader defect is
established. These checks do not prove physical calibration accuracy. Root cause remains unresolved; no
threshold/model change or fresh qualification is claimed. The attempt status is
`v2_reserved_01/qualification_status.json`; inspection is in
`v2_initialization_diagnostic/`. The consumed initialization frames have now been
examined post-test, which must be disclosed in any later evaluation protocol.

Theory: [Solà](https://arxiv.org/abs/1711.02508),
[stochastic cloning](https://authors.library.caltech.edu/records/enrkd-zam24),
[consecutive displacement correlation](https://intra.ee.ucr.edu/~mourikis/papers/MourikisRoumeliotis_ICRA06.pdf).
