# Moving-start metric visual–inertial initialization protocol

Status: independent module integrated through `vio/replay_initialization.py`, under review and recorded-data development. Public entry points are `MovingWindow(times_ns, body_poses, imu_records)` and `initialize_moving(window) -> (State, report)` in `vio/dynamic_initialization.py`. This is a small independent calibrated metric alignment, not OpenVINS or VINS-Mono and not a new accuracy claim for the portfolio estimator.

## Frozen input and assumptions

Input contains at least50 successful calibrated body poses spanning at least5seconds. Each pose is visual-world-from-body in an arbitrary but fixed **metric** visual frame; camera-to-IMU extrinsics have already been applied. Integer nonnegative int64 nanosecond timestamps must strictly increase. IMU records are `(timestamps, samples)` with Nx6 columns `[gyro_x,gyro_y,gyro_z,specific_force_x,specific_force_y,specific_force_z]`, in the same calibrated body frame. They must bracket the window. No ground-truth argument or file access exists in this module.

Calibration, metric stereo scale and sensor time offset are assumed known. Body-frame gyroscope/accelerometer biases are constant across the window. Visual rotations/positions are observations, not exact truth, but this simplified initializer does not optimize them or recover their full covariance. The position residual scale .01m is a fit weight, not a claim that correlated stereo pose errors are independent Gaussian measurements. In particular all displacements share the first pose error. Visual gaps above .25s and bracketed IMU gaps above .02s reject the window; these fixed interpolation limits are protocol choices, not universal sensor limits.

Before any reserved evaluation, development uses analytic synthetic trajectories and only V1 data. V2 ground-truth states/errors are forbidden as initializer inputs or tuning feedback. Acceptance thresholds below are fixed before that evaluation. Any later change needs a new documented development cycle.

## Gyroscope bias

For observed body orientations R_i and R_j in visual world, the measured body-to-body increment is D_ij=R_i^T R_j. Split every interval at actual IMU timestamps and pose endpoints. Linearly interpolate gyro samples at each subinterval midpoint and integrate:

```
D_imu(b_g) = product_k Exp((omega_mid,k - b_g) * dt_k)
e_ij = Log(D_imu(b_g)^T * R_i^T * R_j)
```

The product is chronological left-to-right and composes body increments on the right. Optimize three constant bias components with soft-L1 loss, .01rad robust scale, zero initial guess. Adjacent visual rotations constrain the gyro bias without gravity or any truth state. Report raw per-interval rotation RMS/max, residual Jacobian singular values and evaluation count. Reject nonconvergence, nonfinite output, RMS above .03rad, max above .15rad or a bias Jacobian with rank below3. Robust fitting does not excuse a grossly inconsistent interval: the raw maximum gate remains active.

## Translation and gravity

Let R(t) map body vectors into the arbitrary visual frame, f(t) be measured specific force, b_a a constant body-frame bias, and g gravity in visual coordinates:

```
a_visual(t) = R(t) [f(t) - b_a] + g
J(t) = integral_0^t R(s) f(s) ds
K(t) = integral_0^t R(s) ds
A(t) = integral_0^t (t-s) R(s) f(s) ds
B(t) = integral_0^t (t-s) R(s) ds
p(t)-p(0)-A(t) = t*v0 + 0.5*t^2*g - B(t)*b_a
```

The minus sign before B is essential. The nine unknowns are `[v0, g, b_a]`. The visual metric scale is never fitted away. R(s) is Slerp-interpolated from observed body rotations. IMU force is interpolated at midpoint samples. Integration knots are the union of IMU times and visual times, so quadrature never crosses an observation endpoint. Under piecewise constant midpoint R and Rf, each subinterval updates position integrals from the *previous* velocity integrals:

```
A_next = A + J*dt + 0.5*(R*f)*dt^2
B_next = B + K*dt + 0.5*R*dt^2
J_next = J + (R*f)*dt
K_next = K + R*dt
```

The data-only rank analysis below selects the bias policy before fitting. At practical rank9, a regularized linear solve supplies the initial guess; nonlinear least squares minimizes position residuals divided by .01m, a zero-mean b_a prior with sigma1m/s², and `(norm(g)-9.80665)/.01`. At practical rank below9, b_a is fixed to its documented zero prior mean and only six unknowns `[v0,g]` are fitted; this is an explicit conservative assumption, not a zero-variance calibrated bias. The gravity norm is a soft constraint with units m/s²; the accelerometer prior is deliberately weak, explicit and still capable of determining a low-excitation solution. No robust loss hides translation jumps. Reject nonconvergence/nonfinite output, raw3D position RMS above .05m, max above .2m or gravity norm discrepancy above .05m/s². No rejected fit resets an existing estimator: this module has no estimator reference.

## Observability and endpoint state

The data-only design matrix `[tI, .5*t²I, -B(t)]` is analyzed **without** bias/gravity priors. Report its column-normalized singular values, numerical rank and practical rank using a relative1e-3 singular-value threshold. Below rank6 reject insufficient alignment geometry. Below practical rank9 label the solution `prior_conditioned`, set `bias_identified_from_data=false`, and use `fixed_zero_prior_mean_due_to_rank`. The weakly identified unconstrained bias is never injected into the returned state. If fixing bias to zero cannot satisfy the unchanged residual/gravity gates, reject the window rather than silently relaxing the gates. Even a small radial bias can cause this in unrotated motion; constant orientation cannot supply a reliable unrestricted3D bias estimate. Constant orientation makes gravity and body accelerometer bias inseparable: good residuals alone cannot identify lateral bias. Full practical rank is only a local design diagnostic, not proof that visual poses are accurate, nor a posterior covariance certificate.

At final time T:

```
v_end = v0 + g*T + J(T) - K(T)*b_a
```

Choose the minimum rotation Q mapping fitted gravity direction to `[0,0,-1]`. Return endpoint position zero (a new local origin), velocity Q*v_end, orientation Q*R_end and the two estimated body-frame biases. Global yaw and position remain gauge freedoms. Report Q, fitted gravity and both initial/endpoint visual velocities so the frame choice is auditable. No full initializer covariance is returned. The report requires accelerometer-bias variance at least1m²/s⁴ (sigma1m/s²) on every axis, including when the returned bias mean is fixed at zero. The old ESKF default .04m²/s⁴ is inconsistent with this initializer prior and must not be reused unchanged. An unresolved1m/s² transverse bias corresponds to a tilt scale of about0.102rad via sigma_ba/g; this is a warning about coupled gravity/bias uncertainty, not a complete diagonal covariance prescription. The integrating caller must justify velocity/attitude uncertainty and gravity–bias cross-correlation; this module does not claim posterior covariance recovery or certify any guessed covariance as conservative. All bias outputs are labeled uncalibrated.

## Finite verification

Synthetic observations are generated from analytic nonzero translation/acceleration, multiaxis Euler rotation with analytically differentiated body angular velocity, known biases and an arbitrary visual-frame rotation/translation. Tests compare bias, endpoint velocity and gravity in body coordinates to eliminate global-yaw ambiguity. A second visual frame must preserve body velocity and biases. Constant-velocity/unrotated cases check both successful zero-prior conditioning with unresolved lateral bias and explicit rejection when a radial bias makes that model inconsistent with gravity. They never require the algorithm to falsely recover unobservable bias. Parameterized failures cover invalid timestamps, missing IMU bracket, long gaps, invalid rigid poses, nonfinite measurements, gross visual position error and inconsistent angular motion. Inputs must not be mutated.

Run from `04_SENSOR_FUSION` with the existing Linux environment:

```sh
../.venv/bin/python -m pytest -q tests/test_dynamic_initialization.py
```

Recorded V1 development checks use windows0:101 and400:501 of the revision2 flow replay (each5s), plus corresponding V1 IMU samples. These precede the known revision2 failure near frame1713. No V2 input is read. Such a fit is an implementation smoke check without a truth-based accuracy claim; a failure must remain recorded, with no relaxation using reserved data.

## References and difference from upstream

[OpenVINS initialization documentation](https://docs.openvins.com/namespaceov__init.html) describes a more complete dynamic initialization and covariance recovery. [Qin, Li and Shen, VINS-Mono](https://arxiv.org/abs/1708.03852) provides background on visual/inertial initialization and tightly coupled estimation. These were consulted as primary references; no upstream implementation was copied and successful execution here does not inherit their validation or accuracy. This module uses already metric calibrated stereo poses and a separate bias/gravity alignment rather than their complete estimators.

## V1-only policy diagnostic and uncertainty handoff

The original unrestricted weak-prior fits remain saved. The first5s has practical data rank6 and estimated bias `[.194,1.765,.170]`m/s² despite7.76mm position RMS. The moving20–25s window has rank8. Before opening V1 training truth, both unrestricted and fixed-zero states were serialized to `v1_prior_comparison_states.json`. Truth was then read only for post-fit development scoring in `v1_prior_comparison_scores.json`; it is never an initializer argument. On the first window, fixing the prior mean changes tilt error8.06°→2.45° and endpoint body-velocity error0.065→0.080m/s. On the moving window, the unrestricted fit has better truth errors than fixed zero (tilt2.65° vs3.49°, velocity0.072 vs0.151m/s). Therefore the selected policy is explicitly **rank-based**, not whichever option wins that window's truth score. It prevents unsupported large bias injection but is not guaranteed to minimize trajectory error.

All source, synthetic and V1 diagnostic evidence is under `08_TEST_RESULTS/sensor_fusion/dynamic_initialization_20261026/`. The policy must be frozen and independently reviewed before any reserved evaluation. The adapter now installs the coupled covariance below before cloning, and starts replay strictly after the last initialization image. `run_flow_development.py --initialization moving --frames 0` forces this path on V1; `run_flow_evaluation.py` tries the existing quiet start and falls back only on `QuietStartUnavailable`. Missing files, bad geometry or rejected moving fits remain errors. The first acquired synchronized image starts a fixed5second window (through the first timestamp at or after5seconds); unavailable pre-acquisition images are not fabricated. No image inside that window may be skipped to rescue a fit.
## Coupled15-state engineering covariance

The separate API `initialization_covariance(window, state) -> (P15, report)` adds an initialization prior without changing the fitted state or the existing filter. Error ordering is `[delta_p_world, delta_v_world, delta_theta_right_body, delta_ba_body, delta_bg_body]`. It validates the window again and verifies that the supplied endpoint velocity/orientation match the conditional fit at the supplied accelerometer-bias mean.

For each bias component, perturb the mean by ±1e-4m/s². Hold that perturbed bias fixed and refit the same six velocity/gravity variables, with the unchanged .01m position weighting and .01m/s² gravity-norm weighting. With H=[F,-B], the conditional target is `target+B*bias`; the sign must not be reversed. Keep the baseline yaw gauge fixed: with baseline alignment `Q_base`, form `Q_perturbed = tilt(Q_base*g_perturbed)*Q_base`, where `tilt` is the minimum rotation taking its argument to downward gravity. Apply this same alignment to perturbed endpoint velocity and orientation. Do not independently recompute a minimum rotation from the original visual frame: for a substantially tilted visual frame that introduces a changing world-yaw gauge into the covariance. Central differences use world velocity and `Log(R_baseline^T*R_perturbed)` for the **right/body attitude error**, not Euler angles or a world-frame rotation difference. The bias rows of J are identity; position and gyro-bias rows are zero.

```
D = diag(repeat([.1^2, .5^2, .15^2, 0, .03^2], 3))
P15 = D + J_b * (1.0 I3) * J_b^T
```

This construction is PSD and carries the actual conditional fit's velocity–attitude–accelerometer-bias cross terms. Accelerometer-bias variance is1m²/s⁴ even if the selected mean is fixed zero; velocity and attitude have additional independent .5m/s and .15rad allowances. The report includes the full Jacobian, step, diagonal variances, state ordering and limitations. These allowances are declared engineering priors, not a posterior recovered from all correlated visual/IMU errors, and PSD alone does not prove statistical consistency. Gyro/visual uncertainty is covered only by the declared independent allowances. The policy applies around both zero and nonzero fitted bias means; no recorded ground truth is used.

An independent constant-orientation analytic test verifies sign and frame of the cross terms at both zero and110° arbitrary visual-frame tilt. With h=R^Tg, the transverse tilt response is `A=-skew(h)/||g||²`; the velocity response includes the corresponding gravity-alignment rotation and the scalar radial constrained-fit derivative. The test compares those derived responses with actual covariance blocks. The110° case reproduced Fable's review finding before correction: an erroneous yaw/bias term of−0.14563057 instead of zero. Evidence is retained in `08_TEST_RESULTS/sensor_fusion/dynamic_initialization_20261026/gauge_correction/before_fix.log`. This correction changes the covariance gauge calculation, not the endpoint state, fit parameters, noise allowances or acceptance gates. A second test covers the full-rank nonzero-bias mean, symmetry/positive definiteness, variance floors, input immutability and rejection of a mismatched endpoint state.

Integration order matters: place P15 into the current state covariance **before** cloning the initial pose, or call `clone_current_pose()` after replacement. The clone's cross-covariance must come from the same initial prior. The root-owned constructor/adapter is responsible for that integration; this module does not alter them.
