# Frozen inspection qualification protocol — 2026-10-26

Recorded before extracting new features, fitting new candidates or evaluating new
test predictions. Existing KolektorSDD data only; no new downloads or external
training data. Original baseline artifacts remain unchanged.

## Prior exposure and partitions

The original seed-2026 physical-item split remains fixed. Its test labels,
predictions and images were already inspected in the previous failure analysis:
69 TN, 0 FP, 5 FN, 6 TP. That partition is therefore **previously observed**, not
fresh independent qualification evidence. Prior qualitative knowledge that weak,
edge-originating cracks can be missed motivates the feature family below.

Combine the original training and validation partitions into a development pool.
Never use original test rows for feature normalization, model fitting, threshold
selection, fold assignment, candidate selection or parameter revision. All views
of one physical item remain in the same fold. Deterministic group-stratified folds
balance positive-image counts first, then image counts, using seed 20261026.

## Predeclared candidates and selection

Retain the 18-feature original baseline as a candidate. Add one fixed feature
family at native image resolution: multiscale vertical/square black-hat responses,
horizontal response pooling, row concentration and connected-component extent.
Include edge regions; do not use annotation pixels as features. Annotation masks
only supply image-level training/evaluation labels. This remains classification,
not a segmentation-accuracy claim.

Exactly six candidates:

1. Original features, balanced logistic regression, L2 penalty 0.05.
2–4. Enhanced features, balanced logistic regression, L2 penalties 0.005/0.05/0.5.
5–6. Enhanced features, OpenCV random trees, maximum depth 6/12, 200 trees,
minimum sample count 3, square-root feature subsampling, balanced class priors.

Within a training partition, generate item-disjoint out-of-fold scores with three
folds. Choose each threshold by maximum F2 among thresholds with precision >=0.80;
if none qualifies, retain the maximum-F2 threshold and mark the constraint failed.
Choose the candidate by feasible constraint first, then F2, recall and declared
candidate order. No original-test labels or outcomes enter this process.

Use four outer development folds: all candidate/threshold selection takes place
inside each outer training fold. Report pooled outer-fold predictions as the
development estimate of the entire selection procedure, with each fold's counts.
Then select the final candidate using five grouped development folds, refit it on
all development items, freeze its threshold and evaluate the previously observed
test partition descriptively. Report the original baseline alongside it.

## Acceptance and reporting fixed before experiments

Internal demonstration gate: recall >=0.85 and precision >=0.80 on both pooled
nested development predictions and the previously observed test comparison, with
mean enhanced-feature extraction below 0.5 s/image on this machine. This is a
chosen engineering exercise target, not a supplied production specification or
guarantee. Report FAIL if any component misses it; do not relax it after results.

Save exact folds, candidate scores, thresholds, confusion matrices, prediction
rows, input hashes, timings, model files, failure examples and protocol hash.
Report binomial confidence intervals and the limited defect/item sample sizes.
Reproduce the final selection/predictions from scratch a second time; compare
numeric artifacts, excluding wall-clock timings. Do not revise the candidates or
threshold objective after viewing the final test report. Verified implementation
defects may be corrected transparently with before/after evidence and reruns.

Automated tests cover synthetic defect response, image rejection, grouping,
selection isolation, metric orientation, model serialization and determinism.
Review production code, tests and documentation with the guard skills. A future
fresh-item industrial evaluation remains necessary regardless of this outcome.

---

## Erratum appended 2026-10-28 (protocol text above unchanged)

The `protocol_sha256` recorded in `run1/frozen_selection.json` and both
`qualification_report.json` files (`bb00e76f…ae42`) is the hash of this file before
this section was appended; the frozen snapshot retains those bytes. Nothing above
was edited.

The protocol's premise that "all views of one physical item remain in the same fold"
and that the original seed-2026 split is item-disjoint was satisfied for folder names
but not for image content: test folder `kos27` and development folder `kos34` contain
byte-identical images (eight equal SHA-256 values in the recorded manifests). The
development pool therefore included the pixels of 8 of the 80 test images, including
one of the 11 test defects. The procedure above was executed as written and its
outputs are retained; its observed-test figures are contaminated by that duplication
and are now labelled as such in `README.md` and `FAILURE_ANALYSIS.md`.

Correction policy, fixed before any new run: `records()` performs a content-hash
audit and either rejects identical content across item folders (default) or merges
the linked folders into one split group when `--duplicate-policy merge` is given;
the audit is saved as `duplicate_audit.json`. The frozen candidate, threshold and
recorded predictions are not revised. A deduplicated recount of the recorded
observed-test predictions is produced by `dedup_observed_test.py` and is a corrected
descriptive statistic for the same previously observed partition, not fresh
qualification evidence. Any future run under the merge policy draws a different
partition and is a new, separately reported experiment.
