# I: visual-inertial estimation source audit

Classification: NEW for an independent stereo-inertial estimator; EXISTING stereo reconstruction is deliberately not duplicated as a portfolio repository.
The source audit identified existing fringe-phase and stereo-geometry work. Private repository identities and implementation details remain only in the excluded local audit. The independently specified IMU error-state filter and temporal estimator extend the portfolio without copying private calibration, reconstruction source, or images.

Scope: educational 15-state right-error ESKF, metric stereo/PnP camera motion, timestamp scheduling, calibration input, covariance and innovation gating, deterministic synthetic image/IMU experiment, trajectory comparison. Public EuRoC parsing is a separate capability; a synthetic experiment is not a public-dataset benchmark.

Frame convention: R_WB maps body vectors to world; gravity is [0,0,-9.80665] m/s². Nominal state p_W, v_W, R_WB, accelerometer bias b_a, gyroscope bias b_g. Error state order [dp,dv,dtheta,db_a,db_g], right-local orientation perturbation. Camera-to-body extrinsic maps camera points to body. IMU timestamps use integer nanoseconds until a difference is formed.

Validation must include stationary gravity cancellation, controlled acceleration/rotation, bias observability limits, covariance PSD/symmetry, analytic versus finite difference Jacobian, visual geometry, timestamp boundary failures, gross outlier rejection, and deterministic image-driven end-to-end replay. No real flight/hardware accuracy claim.
