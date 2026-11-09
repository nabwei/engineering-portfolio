# VIO version 2 — stereo extrinsic self-refinement at initialization (2026-10-28)

Version 2 adds exactly one estimator change to the frozen noisy-frame candidate
([NOISY_FRAME_MODEL.md](NOISY_FRAME_MODEL.md)): a bounded, gated self-refinement of the
right-camera-from-left-camera rotation estimated from raw-image correspondences of the
initialization window (`vio/stereo_refinement.py`). Fusion model, sigmas (0.01 m, 0.005 rad),
beta 0.5, the moving/quiet initializers, the 22.458 innovation gate and the frontend's
0.5 px per-track vertical gate are unchanged. Every historical V1/V2 FAIL record stays as is.
Protocol, amendment and evidence: `../08_TEST_RESULTS/completion_20261028/i_vio_v2/`.

## Why

The frozen V2 attempt failed on its first stereo pair: every LK track had about 1.6 px
vertical residual under the supplied calibration, and the retained raw-epipolar audit showed
the same offset already in raw ORB correspondences (V2 signed median -1.68 px ideal-pixel
epipolar distance; V1 about 0.03 px). A constant vertical offset of 1.6 px at f = 458 px is a
0.2 deg relative pitch. Self-calibration from image correspondences is the standard remedy
when a supplied extrinsic is inconsistent with the images; here it is restricted to the three
rotation parameters that the epipolar constraint identifies, with baseline length, intrinsics
and distortion fixed, so metric scale is not re-estimated from images.

## Algorithm

1. Window: the same timestamps the moving initializer uses (first acquired synchronized pair
   through the first stamp at or after +5 s), subsampled with stride 5; even sampled pairs
   form the FIT set, odd sampled pairs the HELD-OUT set (never seen by the optimizer).
2. Correspondences: raw images, ORB (1500 features, edgeThreshold 12, fastThreshold 8),
   brute-force Hamming kNN(2), mutual 0.75 ratio test; pixels undistorted to normalized
   coordinates with `cv2.undistortPointsIter` (50 iterations, 1e-12) using the supplied models.
3. Fit: `delta` (3 small angles, right camera frame) with `R_rl' = Exp(delta) R_rl`,
   `E = [t_rl]x R_rl'`; signed Sampson distance scaled by the left `fx` to ideal pixels;
   `least_squares(loss="soft_l1", f_scale=1 px, x0=0, tol 1e-12)`.
4. Gate statistic: sparse rectified `y_left - y_right` under `cv2.stereoRectify` of the
   geometry under test; `m` = maximum over the four image quadrants (raw left pixel halves)
   of `|median signed residual|`, each quadrant needing >= 50 held-out correspondences.
   A zero-mean localization noise cannot move `m`; pitch, roll and vergence offsets can.
5. Decision (identical for every sequence): NOT_EVALUATED (supplied retained) when fewer than
   6 fit pairs / 600 fit / 300 held-out correspondences or a sparse quadrant; APPLIED iff
   `|delta| <= 1.0 deg`, `m_refined <= 0.25 px` and `m_refined <= m_supplied`; otherwise
   RETAINED_SUPPLIED if `m_supplied <= 0.25 px`; otherwise an explicit `ValueError` — no
   silent fallback and no gate change.
6. Application: `stereo_rectification(mav0, delta)` rebuilds the maps and `Calibration`
   with `R_rl'` and the unchanged `t_rl`; `body_from_camera = T_BS_left @ [r1^T]` keeps the
   left camera's body extrinsic (the independent body-frame control in
   `committee_20261028/i_experiment_review_delta/` is untouched by this change).
7. Reporting: `report["stereo_refinement"]` carries status, per-axis angles, counts, pair
   timestamps, supplied/refined statistics on fit and held-out sets, optimizer status and the
   gate dictionary. `run_flow_development.py --stereo-refinement on` and the frozen runner
   `run_flow_evaluation.py` (always on; `REQUIRED_SOURCE` includes the module; the freeze must
   repeat `algorithm_version = "v2_stereo_self_refinement"` and the exact gate dictionary).

Justification of the 1.0 deg bound: one degree shifts rectified rows by about 8 px, far
outside the frontend's operating assumptions, and a rigid VI-Sensor rig cannot plausibly
drift that far between sessions; a larger correction indicates a wrong calibration file or
sequence and must fail loudly. The observed V2 offset (about 0.2 deg) is well inside.

Precision (synthetic, 900 points, 0.1 px noise): pitch about 0.0005 deg, roll about 0.006 deg,
vergence about 0.01 deg (observable only through the x*y term). Zero-noise recovery is exact.
A residual vergence error of 0.01 deg changes disparity by about 0.08 px, i.e. a depth scale
error of 0.2-0.7 % at 12-34 px disparities; this is reported, not hidden.

## Limitations

- This is self-calibration from image correspondences, not a physical calibration
  certificate; it corrects the rotation that best explains the epipolar residual and cannot
  distinguish a true extrinsic drift from any other effect with the same signature.
- Only rotation is refined; translation direction, intrinsics and distortion errors are
  absorbed into the rotation or remain.
- V1 and V2 are observed development data; results on them are regression evidence only.
- The fresh frozen evaluation requires an untouched sequence from the declared mirror.

## Default-off behaviour

`ReplayConfiguration.refine_stereo` defaults to `False` (`run_relative_euroc.py` line 29) and
the refinement block runs only `if settings.refine_stereo:`; `run_flow_development.py` exposes
`--stereo-refinement {off,on}` with default `off`. With the default, `stereo_rectification(mav0)`
builds exactly the historical maps: the closing self-review compared them bit-for-bit with the
`d02b122` version of `vio/euroc.py` on the real V1_01 and V2_01 calibrations, and the V1
innovation diagnostic reproduced the frozen V1 trajectory bit-equal through the modified code.
The window is delimited by the IMU timestamp coverage (same rule as the moving initializer);
no IMU measurement enters the fit. Only `run_flow_evaluation.py` (frozen runner) turns the
refinement on unconditionally, and its freeze must repeat `ALGORITHM_VERSION` and the exact
`GATES` dictionary or it is refused.

## Results (development data, 2026-10-28; evidence `../08_TEST_RESULTS/completion_20261028/i_vio_v2/`)

All numbers come from receipted runs on already observed data and are development evidence
only. No gate was changed after any result was seen.

| Run | Refinement outcome | Trajectory result | Verdict |
|---|---|---|---|
| V1_01_easy, `--fusion noisy --initialization moving --frames 0 --stereo-refinement on` (`dev_v1_refined_01`) | `NOT_EVALUATED`: the held-out top-left quadrant has 4 correspondences (< 50; counts 4 / 1709 / 683 / 2055); supplied calibration retained; fitted-but-not-applied 0.0032 / 0.0343 / 0.0029 deg | identical to the frozen V1 record: VIO ATE 0.195580 m, VO ATE 0.134484 m, common-frame ratio 1.455, RPE 0.0527 m / 0.709 deg, 4 visual failures, deltas 0.0 | **FAIL** on the unchanged 1.2x VIO/VO gate, exactly as before |
| V2_01_easy, same configuration (`dev_v2_refined_01`) | `ValueError` before initialization: correction 0.2418 deg (0.213 / -0.110 / 0.027), refined held-out max-quadrant offset 0.2889 px, supplied 1.8498 px, limit 0.25 px | none | **FAIL** (initialization still fails, now with a declared, measured reason) |
| Fresh frozen evaluation (first attempt) | — | — | BLOCKED at the time of the stream's closing: the pinned `pepijn223` mirror lists only `V1_01_easy.zip` |
| MH_02_easy, frozen `run_flow_evaluation.py` (`fresh_MH_02_easy_01`, `PROTOCOL_AMENDMENT_02.md`, acquired untouched from the second pinned mirror after the independent review) | `NOT_EVALUATED` (sparse held-out quadrant; supplied calibration retained) — from the post-test diagnosis, because the runner's failure path does not keep the report | none: quiet start unavailable (vehicle already moving at the first reference stamp), moving initializer's visual/IMU translation-alignment gate rejected the fit although all 101 window frames tracked (1.97 m visual path) | **FAIL** before trajectory scoring; metrics NOT RUN; see `FRESH_MH_02_EASY_01.md` |

Interpretation of V2 (`v2_window_residual_diagnostic.json`): the rotation-only fit removes the
bulk of the residual (held-out signed median +1.60 px to -0.03 px) but leaves a top-to-bottom
gradient (y-thirds -0.48 / -0.03 / +0.31 px) that is not monotonic in disparity. That structure
is not the signature of a rotation or baseline-direction parameter; it is more consistent with an
intrinsic/distortion inconsistency between the supplied `sensor.yaml` (byte-identical for V1 and
V2) and the V2 images, but this remains a hypothesis from one observed window, not an established
physical cause. An exploratory 5-DoF fit (rotation plus baseline direction) gained only 2.4 % in
cost at the price of a physically implausible 2.55 deg baseline tilt (about 4.8 mm) and was not
adopted. (Wording hedged after independent review, 2026-10-28; see
`../09_CLAUDE_REVIEWS/completion_20261028/peer_reviews/I_REVIEW.md`.)

Closing verification: full suite 123 passed (103 baseline + 17 refinement + 3 freeze tests),
ruff clean; self-review found no defect (`closing/SELF_REVIEW.md`). Historical FAIL records are
untouched.
