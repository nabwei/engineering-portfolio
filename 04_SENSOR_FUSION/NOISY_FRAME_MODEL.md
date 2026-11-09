# Adjacent increments with an explicit noisy-frame pose model

This model was developed in bounded V1 experiments and then frozen for a V2
attempt. All V1 is already observed development data; no V2 trajectory errors
inform this model. Existing numerical
acceptance criteria remain unchanged. The default pair standard deviations remain
0.01 m and0.005 rad, with a fixed correlated share beta=0.5; there is no beta grid.

The motivation is the shared-frame correlation identified by
[Mourikis and Roumeliotis (2006)](https://intra.ee.ucr.edu/~mourikis/papers/MourikisRoumeliotis_ICRA06.pdf).
The implementation below is a lower-dimensional pose-noise approximation, not a
reproduction of their feature-error augmentation or proof of PnP consistency.

## Generative assumption and covariance budget

Let the physical body pose be `(p_i,R_i)`. Define an unobserved noisy image pose

`Q_i = (p_i + R_i rho_i, R_i Exp(theta_i))`,

where `eta_i=[rho_i,theta_i]` is independently Gaussian for each successfully
processed image, with covariance `Lambda`. The observed relative transform is
`Q_(i-1)^-1 Q_i` plus an independent pair-specific residual in the translation /
right-local rotation residual chart, with covariance `W`. It is the relative
transform that is observed; no absolute integrated visual pose is fused.

For true relative translation `t` and rotation `R`, first-order image noise is

`delta_t = -rho_prev + skew(t) theta_prev + R rho_current`,

`delta_theta = -R.T theta_prev + theta_current`.

Thus adjacent residuals share the same latent image noise. At zero motion their
covariance is `2 Lambda + W`, and lag-one cross covariance is `-Lambda`. At finite
motion the appropriate old/current Jacobians transport that cross covariance.

For the existing reference pair covariance `R_pair`, set
`Lambda = beta R_pair /2` and `W=(1-beta)R_pair`, with beta=0.5 fixed. This preserves
the existing near-stationary marginal pair variance; finite-motion orientation /
translation coupling is retained by the pose geometry. The white residual floor
allows pair-specific errors instead of pretending all PnP error comes from an
independent six-dimensional disturbance on each image.

## Persistent and temporary state

Persist the physical15 error state and the previous **noisy** image pose clone6,
including their cross covariance. Propagation reuses the existing midpoint ESKF
and `diag(Phi,I6)` transition. On a successful next image, append its fresh image
error6 with covariance `Lambda`, independent of the predicted joint state.

At its zero prior mean, the augmented observation Jacobian consists of the
existing relative-pose physical/clone blocks and fresh-image blocks
`H_t,rho=R_rel`, `H_theta,theta_image=I`. Correct the temporary27-dimensional joint
Gaussian with the Joseph update and residual covariance `W`. Inject the physical
state correction using the existing right-local reset convention.

The corrected new image-noise mean is generally nonzero and must not be discarded.
Construct its noisy pose `Q_new` from the corrected physical state and this
conditional image-noise mean, then marginalize the old clone. The augmentation
from `[physical15, eta_new6]` has clone blocks

`J_p,p=I`, `J_p,theta=-R skew(rho)`, `J_p,rho=R`,

`J_R,theta=Exp(theta_image).T`, `J_R,theta_image=J_right(theta_image)`.

The resulting21-dimensional covariance preserves both current physical/image
correlation and the uncertain noisy reference for the following increment.
Central differences verify both the27-dimensional observation and this nonlinear
clone map, including a nonzero image-noise mean.

An innovation rejected by the unchanged22.458 gate leaves physical state and
physical covariance unchanged. It advances the image reference with fresh
unconditioned noise, allowing the next adjacent increment without resetting the
physical pose. This outlier gating is a robust approximation; conditioning on
the rejection event is not modeled as an exact Gaussian posterior. A frontend
failure creates no new successful image and retains the previous reference.

## What this does and does not establish for the actual frontend

A relative-PnP covariance alone does not identify `Lambda`, `W`, or beta. In a
feature-level derivation, errors from one image are projected differently into
its incoming and outgoing relative estimates. Feature selection changes, stereo
depth uncertainty and different pair geometries need not admit one common
six-dimensional per-image pose error. The iid pose proxy is therefore a declared
engineering model, not an exact reduction of the implemented PnP frontend.
Fixed calibration errors, persistent scene bias and longer-range correlations
remain unmodeled. A white floor does not mathematically bound every such effect.

Under the declared linear-Gaussian model, a test compares the recursive posterior
with an independently assembled batch posterior over initial position, velocity
and every individual image error. This is stronger than checking positive
semidefiniteness alone. It does not demonstrate statistical consistency on EuRoC.

A moving initializer that uses image frames must finish before the first new
fusion image. Reusing its last image as an independent noise prior would omit
initial-state/image cross covariance. The moving adapter must propagate with IMU
to a strictly later image; broader calibration correlations remain a limitation.

## Original bounded experiment protocol

First run `tests/test_noisy_frame.py`. Then replay the unchanged full V1 four-view
observations through `run_noisy_frame_development.py`, at fixed beta and default
pair covariance. Preserve every failed camera frame in fused scoring. If the
unchanged criteria pass, verify with actual images using the shared frontend loop
before freezing a reserved experiment. A failure remains FAIL; do not compensate
by quietly changing beta, the gate, or the accepted numerical error limits.
The later decision to evaluate the fixed default model despite failed quality
was explicitly experimental; it did not turn this protocol's failed gate into a
pass or select a winning configuration from the failed grid.

## Recorded result and one subsequent calibration experiment

The initial fixed-default-noise V1 replay produced ATE0.214191 m, versus0.301443 m
for disjoint updates and0.135218 m for standalone VO. RPE was0.049444 m /0.647972°;
2855 updates were accepted and2 rejected. The baseline-relative gate remained
FAIL. Beta was not tuned.

Before subsequent results, one new-model-specific V1 covariance grid is declared:
pair-standard-deviation factors `[1,.5,.25]`, keeping beta=.5, process model,
initializer and every acceptance criterion fixed. Factor1 must exactly reproduce
the completed noisy-frame run's poses, visual observations, timestamps,
diagnostics and metrics before lower factors run. Choose lowest full-development
ATE only among candidates passing all existing criteria; otherwise select none.
No extension of this grid is authorized. Actual-image verification and reserved
evaluation remain later gates. This calibration is independent of V2 errors.

The bounded grid completed with no passing candidate: factors1/.5/.25 produced
VIO ATE0.214191/0.179159/0.228483 m, with2/8/15 rejected updates respectively.
The original-noise replay exactly reproduced poses, visual observations,
timestamps, diagnostics and metrics. Every numerical gate remained unchanged,
and the selected configuration is explicitly absent. The model is an implemented,
tested development alternative, not a qualified replacement for the baseline.

The fixed original-noise model was subsequently retained as an explicitly
experimental candidate, independently of the failed grid selection. A full
actual-image V1 replay reproduced the cached trajectory, visual poses, timestamps,
diagnostics and metrics exactly. Its quality result remains FAIL. Evidence is
`../08_TEST_RESULTS/sensor_fusion/qualification_20261026/noisy_frame_actual_v1_01/actual_vs_cached.json`.
The revised plot in that run's `plots_v2` directory clips the NIS display below
zero without changing observations or metrics; the original plot is preserved.

The additional turning multi-update test independently central-differences
27-state image geometry and the retained-state retraction, then conditions the
Gaussian prior in information form. It compares four successive nonzero-residual
updates, including the fresh-image conditional mean and full retained covariance.
Small corrections bound the difference between exact numerical retraction and
the implementation's first-order attitude reset. This supplements the scalar
batch posterior test; it does not establish feature-level statistical consistency.

## Final frozen execution status

The actual quiet-start replay remains ATE 0.214191 m versus VO 0.135218 m.
After independent review corrected a world-yaw gauge artifact in the moving
initializer covariance, one full actual-image moving-start replay produced ATE
0.195580 m versus VO 0.134484 m, and one-second RPE 0.052672 m / 0.708987 degrees.
It retained all 2,790 estimates, including four visual failures; 2,783 updates
were accepted and two rejected. Both runs **FAIL** the unchanged comparison
against VO. Different initialization windows mean these are not identical
evaluation intervals. No further calibration followed the reviewed fix.

`run_flow_development.py --fusion noisy --initialization moving --frames 0`
selects this actual-image path; `quiet` selects the other initializer. The
reserved runner `run_flow_evaluation.py` fixes noisy fusion and tries quiet then
moving initialization through `automatic_start`, using its source/parameter
freeze and supplied acquisition manifest. `reset_policy` describes the absence
of a state-reset code path, not a measured event counter. Generic input provenance
is `UNVERIFIED_INPUT_FILES` with format `EUROC_ASL`; public-source identity needs
the separately retained acquisition evidence.

The frozen V2 attempt **FAILS during moving initialization**, on its first
stereo pair. There is no scored V2 trajectory: metrics are **NOT RUN**, not zero
error. Input hashes passed verification; post-test diagnostic inspection found
vertical stereo residuals incompatible with the unchanged 0.5-pixel gate.
Published reference rectification produces the same first-pair rejection; no
calibration-loader bug is established. Root cause is unresolved and these checks
do not establish physical calibration accuracy or a corrected frontend.
The original failure remains in
`../08_TEST_RESULTS/sensor_fusion/qualification_20261026/v2_reserved_01/qualification_status.json`;
the reviewed V1 run is in the neighboring `noisy_moving_gauge_fixed_v1_01`.
No successful fresh public qualification is claimed. Later use of inspected V2
initialization images must be disclosed rather than relabeled unseen evidence.
