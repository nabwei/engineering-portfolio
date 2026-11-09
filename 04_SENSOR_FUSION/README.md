**Public status: research prototype. V1 comparison and V2/MH_02 initialization remain FAIL.**

See the repository [validation summary](https://github.com/nabwei/engineering-portfolio/blob/main/docs/VALIDATION.md) for current limits. Historical evidence paths below refer to local generated outputs.

# Stereo-inertial estimation (I)

An independent educational stereo visual-odometry frontend and error-state filters.
No private implementation or calibration is included.

The autonomous relative-pose estimator is documented in
[RELATIVE_VIO.md](RELATIVE_VIO.md). The frozen experimental candidate combines
subpixel stereo tracking, autonomous initialization and the
[noisy-frame pose model](NOISY_FRAME_MODEL.md). Its full V1 development runs gave
ATE 0.214191 m with quiet initialization and 0.195580 m with moving initialization
after the reviewed covariance fix. Both **FAIL** the unchanged comparison with
standalone VO; the latter VO ATE is 0.134484 m. V2's first frozen attempt **FAILS
at moving initialization**, before trajectory scoring. No successful fresh public
qualification is claimed; calibration/epipolar-residual diagnosis remains open.

The original ORB relative run reproduced all 2,862 frames exactly, but its
reserved ATE 1.199866 m exceeds the frozen 1 m target (stereo 1.520752 m).
All V1 is now observed development data. The 15-state absolute-VO implementation
and `run_euroc.py` results below remain historical baselines with their original
FAIL. See the relative-estimator guide for current CLI options and evidence.

From this directory, with Python 3.12 and `requirements.txt` installed:

```sh
python -m pytest -q
python run_demo.py --output ../08_TEST_RESULTS/sensor_fusion/demo
```

The demonstration renders 31 synthetic stereo pairs, uses ORB descriptors,
rectified stereo triangulation and RANSAC PnP for metric camera motion, then
fuses poses with 100 Hz simulated accelerometer/gyroscope samples. Known initial
velocity and attitude are supplied. No ground-truth position enters pose updates.
Outputs include metrics, numerical trajectories and a PNG plot.

## Historical absolute-VO mathematical contract

`vio/eskf.py` uses the right-local orientation error and nominal state
`[p_W, v_W, R_WB, b_a, b_g]`. The 15-vector error is
`[delta_p, delta_v, delta_theta, delta_b_a, delta_b_g]`.
Acceleration is specific force in m/s²; angular velocity is rad/s; time is seconds.
Gravity is fixed to `[0,0,-9.80665]` in the world frame.
Noise values are continuous-time amplitude densities. `discretize` uses Van Loan
matrix exponential on a frozen linear dynamics matrix. The nominal integrator
holds attitude and force at the interval start; rapid motion needs a higher-order
integrator. The Joseph covariance update and orientation reset maintain numerical
symmetry; tests check positive semidefiniteness.

`vio/stereo.py` requires already rectified grayscale images with known positive
baseline, intrinsics, and camera-to-body rigid transform. `vio/euroc.py` loads
EuRoC radial-tangential pinhole calibrations and rectifies raw stereo pairs.
Low-texture/PnP failure raises
`ValueError`; the demo reports failed frames and continues inertially. This is not
a robust general-purpose VIO or a tightly coupled landmark filter.

`vio/timing.py` retains integer nanoseconds and provides a standalone known-offset
utility, `aligned_camera_times`. The replay runners do **not** call this utility
or read a camera/IMU time-offset field from YAML. They assume the EuRoC camera and
IMU timestamps already share a synchronized clock, intersect the two camera
timelines, and interpolate bracketed IMU samples in `integrate_interval`.
No temporal offset is estimated. Utility boundary tests do not establish offset
correction in the replay pipeline. The integrated EuRoC-format fixtures are
explicitly synthetic, not public sensor data. The moving homography fixture
checks visual-update wiring with known attitude and bias priors, zero angular
rate, gravity-only IMU and an omitted constant initial velocity. It does not
qualify general visual-inertial fusion or autonomous initialization. Its restricted
prior was selected after two failed development fixtures: the default tilt prior
allowed gravity/tilt to explain translation, and a low velocity variance retained
too much initial velocity error. Those attempts remain in the test evidence.

IMU propagation requires each numerical substep to satisfy `0 < dt <= 0.1`
seconds. A larger substep raises `ValueError` and aborts replay; it is not treated
as a visual dropout. The reserved runner records this exception as `FAIL` before
re-raising it. This bound is **not** a raw-sample-gap detector: camera timestamps
can split a long IMU gap into smaller interpolation intervals. Inputs must have
adequate continuous IMU coverage; the replay integration path does not separately
enforce a maximum raw IMU sample gap. The separate moving initializer does reject
raw IMU gaps above 20 ms inside its bracketed initialization window, through
`vio/dynamic_initialization.py`. That startup check does not protect subsequent
replay. Neither path should be described as robust to inertial loss.

The frozen optical-flow stereo frontend gates each track at `|vertical residual|
<= 0.5` pixels and requires at least 12 surviving tracks. Pair medians in the
later diagnostic are descriptive summaries, not the gate itself. Estimator and
frontend bytes remain unchanged relative to the two V2 candidate/reporting
freezes; earlier ORB-era freezes precede disclosed estimator development and do
not have identical source.

## EuRoC replay adapter

`run_euroc.py` accepts an extracted ASL-format `mav0` directory containing cam0,
cam1, imu0, and state_groundtruth_estimate0 CSV/YAML files and camera PNGs:

```sh
python download_euroc_mirror.py --frames 400
python run_euroc.py --mav0 datasets/V1_01_easy_mirror/mav0 --frames 400 --provenance PUBLIC_EUROC --output ../08_TEST_RESULTS/sensor_fusion/euroc
python plot_euroc.py ../08_TEST_RESULTS/sensor_fusion/euroc
```

The downloader selects a bounded prefix of recorded V1_01_easy stereo frames
from a pinned community mirror. Official acquisition attempts and mirror provenance
limits are documented in `EUROC_ACQUISITION.md`. Provenance is explicitly asserted by the
caller; folder structure alone cannot prove data origin. Synthetic fixtures must
use `--provenance SYNTHETIC_EUROC_FORMAT`.

The adapter reads integer timestamp columns without float conversion, requires
strictly ordered IMU records, intersects exact stereo timestamps, and clips to
the overlap with IMU/ground truth before selecting the requested prefix. Counts
are reported. It interpolates IMU at midpoint integration intervals split at IMU
sample boundaries; no extrapolation is allowed. Ground-truth quaternion columns
are w,x,y,z, converted to x,y,z,w for SciPy Slerp. The initial position, velocity,
orientation **and biases are supplied from ground truth**, so this is assisted
initialization, not autonomous VIO. Later ground truth is used only for scoring.

`T_BS` is body-from-sensor. Stereo rectification uses right-from-left geometry;
the rectified left camera-to-body rotation includes the inverse rectification
rotation. The visual origin is anchored using the first stereo pair at
initialization. A failed first frame aborts rather than silently shifting
that origin. Later visual failures are recorded while inertial propagation
continues. The current adapter requires imu0 and body frames to coincide and
supports only a positive horizontal rectified baseline.

## Optional bounded recovery

The default `--recovery baseline` preserves the original estimator. To evaluate
the separate operational fallback, use the same replay command with
`--recovery bounded` and a new output directory. `vio/recovery.py` does not use
reference trajectories. After one second of continuous pose rejection it requires
at least five coherent visual observations covering 0.5 seconds, then resets
position and attitude to the current visual pose and estimates velocity with a
centered linear fit to that history. Current IMU biases are retained.

It rejects visual increments above the explicit 10 m/s or 10 rad/s engineering
envelope; a gap above 0.25 seconds clears eligibility. A fit residual above 0.25 m
also blocks recovery. These are fixed demonstration assumptions, not measured
hardware limits. At most three recoveries occur per replay; exhaustion is reported.
An isolated rejected observation does not authorize a reset.

Each reset discards cross-covariances and retains or increases marginal variances
to standard-deviation floors of 0.5 m position, 1 m/s velocity, 0.2 rad attitude,
0.2 m/s² accelerometer bias and 0.05 rad/s gyroscope bias. These floors express an
uncertain operational reset; they are not calibrated confidence bounds. Reports
include reset time, position/attitude/velocity discontinuities, each update
decision and separate standalone visual metrics. This is degraded
reinitialization, **not a statistically consistent correction for correlated VO**.
Coherent but wrong visual motion can still fool it. Missing imagery, severe
motion, budget exhaustion and autonomous initialization remain unresolved cases
for this historical fallback; the separate relative estimator adds autonomous
quiet/moving initialization with the limits documented in its guide.

Run independent synthetic fault comparisons with:

```sh
python run_recovery_scenarios.py --output ../08_TEST_RESULTS/sensor_fusion/recovery_synthetic_scenarios.json
```

## Historical absolute-VO evidence and limitations

Results are in `../08_TEST_RESULTS/sensor_fusion/` and the adjacent logs.
ATE uses rigid SE(3) alignment without scale fitting. Raw RMSE is also reported.
Relative displacement RMSE is explicitly a one-sample displacement statistic,
not full rotational SE(3) RPE. Straight trajectories do not constrain all alignment
rotations well. Visual errors are temporally correlated; the loose-coupled filter
does not model those correlations, so its covariance is not an accuracy guarantee.

The 6D pose gate is 22.458 (chi-square, 6 DOF, approximately 99.9%). Large orientation
errors fall outside the small-error linearization assumptions even if a pose passes
this gate. This historical implementation has no automatic initialization,
loop closure, relocalization, rolling-shutter model, online calibration or GTSAM
comparison. Full recorded-sequence development runs exist for the newer relative
estimator; they do not establish successful reserved-sequence qualification.

A bounded recorded-data replay has run; its results and provenance are documented
in `EUROC_ACQUISITION.md`. It is not autonomous initialization or EuRoC qualification.
The baseline 400-frame, 19.95-second trial **fails tracking accuracy**: 12.94 m aligned ATE,
16.00 m raw RMSE and 235 rejected pose updates over only 5.06 m of reference path.
The earlier 100-frame startup trial was much better and is not representative.
Diagnostic traces and the failing trajectory are retained for investigation.
With the optional bounded fallback and unchanged parameters, raw RMSE is 0.3821 m
and ATE 0.1853 m, with two reported resets. Standalone visual odometry remains
better (raw RMSE 0.3206 m, ATE 0.1102 m). The fallback avoids this runaway; it does
not establish consistent fusion or superior accuracy. Reset discontinuities reach
1.451 m and require downstream consumers to handle the explicit reset event.
EuRoC data is now hosted by ETH Research Collection under non-commercial terms;
the local mirror subset and downloaded sensor files are excluded from releases.
See https://projects.asl.ethz.ch/datasets/euroc-mav/ . Mathematical background:
Joan Solà, *Quaternion kinematics for the error-state Kalman filter*,
https://arxiv.org/abs/1711.02508 .
