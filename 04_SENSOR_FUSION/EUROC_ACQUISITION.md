# EuRoC acquisition — recorded mirror sequence acquired

The 26 October acquisition now includes all 2,891 stereo pairs at or after the
first reference timestamp, plus complete IMU/reference CSV and calibration:
5,790 files. It resumes the old prefix and verifies each ZIP entry CRC and file
SHA-256. The successful resumed acquisition transferred 856,199,840 bytes;
earlier partial/failed attempts are recorded separately. Images preceding the
first reference timestamp are not acquired. This permits complete post-startup
evaluation, not a claim to process every image of the original archive.
Full-sequence evaluation is running; no synthetic result is labeled as
public-data performance. The earlier acquisition and baseline history follows.

On 2026-10-25 the official landing page at https://projects.asl.ethz.ch/datasets/euroc-mav/ directs downloads to ETH Research Collection DOI https://doi.org/10.3929/ethz-b-000690084. The publication is Burri et al., *The EuRoC micro aerial vehicle datasets*, IJRR 2016, https://doi.org/10.1177/0278364915620033 . The official landing page notes independent camera exposure and ground-truth synchronization limitations.

Acquisition attempts:

1. The former official HTTP/HTTPS V1_01_easy individual ZIP endpoints timed out (15–20 second bounded requests); no archive obtained.
2. The current ETH landing page returned a JavaScript shell through PowerShell. Its public DSpace API successfully returned the actual original bitstreams. Metadata is preserved as `../08_TEST_RESULTS/sensor_fusion/euroc_bundles.json` and `euroc_bitstreams.json`.
3. The official Vicon room 1 archive is 6,042,263,426 bytes, exceeding the task's 2 GB acquisition budget. Its content endpoint is https://www.research-collection.ethz.ch/server/api/core/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/content .
4. `probe_euroc_archive.py` attempted bounded HTTP Range access to the ZIP directory, with strict byte-range verification and a 2 GB transfer ceiling. The content service returned **HTTP 429 Too Many Requests** on the initial range. The probe stopped; the full archive was not downloaded. `euroc_range_probe.log` preserves this failure (Python ZipFile wraps the HTTP error as BadZipFile; the original 429 is visible in the traceback).
5. Read-only mirror metadata checks initially found two empty Hugging Face repositories and one MH_01 bag of 2,673,818,914 bytes, above budget.
6. A public community mirror at https://huggingface.co/datasets/pepijn223/euroc-mirror provided a range-capable V1_01_easy ZIP. The download pins revision `d05bd6a92a5f456562153f83d65805fea27cb57d`. Its advertised archive size is 1,149,702,102 bytes. Only selected calibration/CSV records and a bounded image prefix were retrieved; no downloaded code was executed.

`download_euroc_mirror.py` checks exact HTTP206 Content-Range responses, enforces a
1.5 GB per-run transfer ceiling, rejects paths outside the dataset directory, verifies ZIP
CRC on extracted records and records SHA-256 for every selected file. It resumes
only matching-size, matching-CRC files. The acquisition manifest preserves the
mirror's advertised whole-archive SHA-256, but that hash was **not verified**
because the full archive was not downloaded. Neither was byte equivalence to the
official ETH archive verified. No dataset is included in releases.

The final acquisition uses 16 MiB range caching (within the 20 MB maximum single
request) to avoid repeated redirect overhead. WSL DNS failed on the first new
attempt; the same standard-library downloader succeeded under native Windows.
An interrupted 1 MiB-cache attempt is retained in the logs; existing complete
files were CRC-checked before reuse. Final manifest:
`datasets/V1_01_easy_mirror/acquisition_manifest.json`; acquisition logs:
`08_TEST_RESULTS/sensor_fusion/qualification_20261026/` relative to portfolio root.

The actual cam0/cam1 intrinsics, radial-tangential distortion coefficients and
752×480 resolution match the independently maintained OpenVINS EuRoC calibration:
https://github.com/rpng/open_vins/blob/master/config/euroc_mav/kalibr_imucam_chain.yaml
(examined Git blob `7012144b5af92d877c0bd6375ec55e046521b463`). This supports
dataset identity, not archive authenticity. Evidence: `euroc_calibration_identity.json`,
`euroc_mirror_metadata.json`, `euroc_mirror_directory.json` and acquisition manifests
under `../08_TEST_RESULTS/sensor_fusion/`.

The ETH record declares **In Copyright — Non-Commercial Use Permitted**:
http://rightsstatements.org/page/InC-NC/1.0/ . The mirror has no independent dataset
card/license declaration; it does not expand those rights. Local educational
evaluation uses the recorded data under the original restriction. Downloaded data,
images and sensor metadata remain ignored and excluded from release archives.

The adapter itself is implemented and tested with explicitly synthetic EuRoC-format files: real PNG file reads, YAML calibration, raw stereo rectification, 61 integer-nanosecond IMU records, ground-truth quaternion interpolation and seven-frame replay. This verifies the adapter and integration contracts, **not** accuracy on EuRoC. Evidence: `euroc_extension_tests.log` (34 passing tests including the prior suite), `euroc_adapter_demo/metrics.json` (`SYNTHETIC_EUROC_FORMAT`).

The recorded replay is deliberately assisted at initialization by ground-truth
position, velocity, attitude and **both IMU biases**. Later ground truth is used
only for scoring. Noise densities come from the sensor metadata; visual covariance
and pose gate remain the existing fixed values. There was no public-data tuning.
This result must not be compared directly to autonomously initialized,
full-sequence VIO benchmarks.

## Recorded result: execution PASS, tracking accuracy FAIL

The 400 stereo pairs span 19.949999872 seconds and 5.060253 m of reference path;
the x/y/z spans are 1.358868 / 2.308002 / 0.653709 m. This includes actual motion,
not only startup stationarity. The first 100-pair result is retained to expose
how an apparently successful short test can conceal later failure.

| Measurement | First 100 pairs (4.95 s) | First 400 pairs (19.95 s) |
|---|---:|---:|
| SE(3)-aligned position ATE RMSE | 0.005973 m | 12.941036 m |
| Unaligned position RMSE | 0.012927 m | 16.002577 m |
| One-sample displacement RMSE | 0.000937 m | 0.169347 m |
| Accepted pose updates | 100 / 100 | 165 / 400 |
| Visual frontend exceptions | 0 | 0 |
| Final unaligned position error | 0.044673 m | 46.059004 m |

The longer result is an accuracy **FAIL**: the estimator diverges while actual
motion remains bounded. A positive covariance eigenvalue and no exception do not
establish tracking accuracy. The remaining 235 pose estimates were rejected by
the existing gate; they are not missing images or exceptions. Source changes
after this trial add diagnostics only. No ground-truth-tuned threshold or hidden
reset was introduced to improve the score.

The diagnostic repeat reproduces identical trajectories and metrics. Rejection
begins at 8.249999872 seconds and continues to the end. At the final frame,
standalone visual position error is 0.56334 m with 732 PnP inliers, while the
fused estimate is wrong by 46.05900 m. This identifies a fusion/gating failure
in addition to visual drift. The fixed covariance assumption ignores correlated
visual drift; its contribution is plausible, but a unique cause has not been
proven. No coordinate/sign defect was established by this diagnostic review.

Evidence: `euroc_mirror_100/metrics.json`, `euroc_mirror_400/metrics.json`,
`euroc_mirror_400/motion.json`, `euroc_mirror_400/trajectory.png` and
`PUBLIC_REPLAY_REVIEW.md` under `../08_TEST_RESULTS/sensor_fusion/`.
The first acquisition transferred 47,442,514 bytes; the extension transferred
135,828,128 bytes, totaling 183,270,642 bytes plus small metadata probes.
The final manifest lists 808 selected files (800 images plus eight metadata files).

## Optional bounded recovery comparison

The baseline remains the CLI default and the original failed evidence is retained.
`--recovery bounded` enables a separately documented degraded reset policy. Its
parameters were fixed before the new recorded run; no later ground-truth pose,
velocity or bias enters recovery. The same 400 frames and initial assisted state
produce these results:

| Measurement | Baseline | Bounded recovery | Standalone visual odometry |
|---|---:|---:|---:|
| SE(3)-aligned ATE RMSE | 12.941036 m | 0.185283 m | 0.110173 m |
| Raw position RMSE | 16.002577 m | 0.382060 m | 0.320624 m |
| One-sample displacement RMSE | 0.169347 m | 0.077380 m | 0.003941 m |
| Final raw position error | 46.059004 m | 0.563196 m | 0.563340 m |

There are 358 ordinary accepted updates, 40 ordinary rejections and two degraded
reinitializations. The two reset position jumps are 0.711120 m and 1.451038 m;
velocity jumps are 0.815423 m/s and 2.029225 m/s. The discontinuities contribute to
the displacement error and are not excluded from scoring. Maximum fused position
error is 1.382046 m. Standalone VO remains more accurate on the aggregate metrics.

This is a **PASS for the bounded fallback's operational comparison**, not a claim
of statistically consistent fusion, full-sequence qualification or smooth motion.
Evidence is in `euroc_mirror_400_recovery/`, `recovery_tests.log` and
`recovery_synthetic_scenarios.json`; the latter contains independent generated
IMU fault cases with correlated visual drift, not extra recorded benchmark data.
