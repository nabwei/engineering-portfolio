# VIO qualification protocol — recorded before new estimator experiments

The old absolute-VO ESKF and bounded reset results remain historical FAIL evidence.
The first 400 truth-overlapping V1_01_easy stereo frames have already been observed
and are development data. Frames after that prefix are a reserved evaluation
segment; no parameter selection may use their ground-truth errors. Report all
failures and every parameter revision. The sequence comes from the pinned public
community mirror documented in EUROC_ACQUISITION.md, not university data.

Implement a relative-pose ESKF with a correlated previous-pose clone. Use disjoint
image pairs so two updates do not reuse the same image measurements. Initialize
gravity and gyro bias from a detected quiet IMU interval, with zero velocity and
arbitrary world origin/yaw. Ground truth must only enter scoring. If no quiet
interval is found, fail initialization explicitly. This is a loose-coupled VIO,
not a reproduction of MSCKF or a claim of statistically consistent feature SLAM.

Required checks: finite-difference relative-pose Jacobian; covariance positive
semidefiniteness and common-position uncertainty preservation; inertial motion
and bias tests; synthetic outlier/dropout tests; complete post-initialization
public sequence with integer timestamp alignment and calibration; repeated
deterministic output; independently reviewed source and Arabic derivation.

Predeclared numerical acceptance for the reserved segment: SE(3)-aligned ATE
RMSE <= 1.0 m, one-second translation RPE RMSE <= 0.30 m, one-second rotation
RPE RMSE <= 5 degrees, no state resets or nonfinite estimates. Also report the
same-camera standalone stereo baseline; VIO ATE must not be more than 20% worse
than that baseline. These are portfolio experiment criteria, not published
EuRoC leaderboard claims. A rejected criterion stays FAIL; do not relabel the
method as validated because unit tests pass. Public-sequence failures motivate
a new explicitly disclosed development cycle and a different reserved dataset.

Theory references: Solà, https://arxiv.org/abs/1711.02508 ; Roumeliotis/Burdick,
https://authors.library.caltech.edu/records/enrkd-zam24 ; correlated consecutive
displacements: https://intra.ee.ucr.edu/~mourikis/papers/MourikisRoumeliotis_ICRA06.pdf .
The last paper motivates disjoint images here; this implementation does not
implement its full image-error augmentation.
