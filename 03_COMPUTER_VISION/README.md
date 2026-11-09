**Public status: research prototype. Accuracy qualification remains FAIL.**

See the repository [validation summary](https://github.com/nabwei/engineering-portfolio/blob/main/docs/VALIDATION.md) for current limits. Historical evidence paths below refer to local generated outputs.

# Industrial inspection and grouped qualification

Project D implements a small supervised binary classifier using OpenCV preprocessing, black-hat morphology and connected component statistics. A regularized logistic model maps these features to a defect score. This is a reproducible baseline, not a claim of production inspection accuracy.

From this directory, with the portfolio environment installed:

```sh
../.venv/bin/python -m pytest tests -q
../.venv/bin/python download_dataset.py
../.venv/bin/python run_inspection.py --dataset datasets/KolektorSDD --output ../08_TEST_RESULTS/vision_measurement/inspection_new --duplicate-policy merge
```

Download/extract the official archive described in DATASET.md into `datasets/KolektorSDD` first. Dataset pixels are not included in source releases. The split manifest contains SHA-256 for every input image, item folder, content-checked split group, labels, partitions and scores. Normalization is fitted only on training images; the decision threshold maximizes F1 only on validation images. Held-out labels are used only for reporting.

**Erratum 2026-10-28 (evaluation integrity).** Earlier versions of this file stated "Physical items are disjoint across partitions" and reported the results below as item-disjoint with no leakage. That statement was true of the folder names and false of the image content: the eight images under test item `kos27` are byte-identical to the eight images under development item `kos34` (identical SHA-256 values are visible in `../08_TEST_RESULTS/vision_measurement/inspection/split_manifest.csv` rows 209–216 versus 265–272, and in `qualification_20261026/run1/test_predictions.csv` versus `development_manifest.json`). The code never read test labels, but 8 of the 80 test images, including 1 of the 11 test defects, were present in training/validation. All recorded results are retained unchanged below and in the evidence tree; they are now labelled as contaminated by this duplication. See "Duplicate content audit and policy" for the guard that now prevents it and the script that recounts the frozen predictions without the duplicated rows.

## Duplicate content audit and policy

Folder-name grouping is necessary but not sufficient. `audit_duplicates.py` hashes every `kos*/*.jpg`, lists every group of byte-identical images, and marks the groups whose members live in more than one item folder. `records()` in `run_inspection.py` applies an explicit policy to that audit before any split or fold is drawn, and both runners write `duplicate_audit.json` beside their outputs:

- `--duplicate-policy reject` (default): identical content in two item folders stops the run with a `ValueError` naming the images. Nothing is trained.
- `--duplicate-policy merge`: items linked by identical content are fused into one split group (for example `kos27+kos34`) so the content stays on one side of every split and grouped fold. The merged group is written to every manifest; the original folder name is kept in the `item` column.

There is no hidden, dataset-specific exception: the official KolektorSDD archive triggers `reject`, and the disclosed way to run it is `--duplicate-policy merge`. Because merging changes the group list, the seed-2026 permutation and therefore the partitions differ from the historical split; a new run under either policy is a new experiment, not a reproduction of the contaminated numbers.

```sh
../.venv/bin/python -I audit_duplicates.py --dataset datasets/KolektorSDD
../.venv/bin/python run_inspection.py --dataset datasets/KolektorSDD --output ../08_TEST_RESULTS/vision_measurement/inspection_new --duplicate-policy merge
../.venv/bin/python -I dedup_observed_test.py --run ../08_TEST_RESULTS/vision_measurement/qualification_20261026/run1
../.venv/bin/python -I dedup_observed_test.py --baseline ../08_TEST_RESULTS/vision_measurement/inspection
```

The audit command exits 1 when identical content crosses item folders. The executed audit finds 16 duplicate-content groups involving 32 images: eight pairs link `kos01` with `kos26`, and eight link `kos27` with `kos34`. Eight groups crossed the historical development/test boundary; the other eight also matter for development folds. The guard detects identical file bytes, not re-encoded, resized or near-duplicate images.

`dedup_observed_test.py` removes historical test rows whose SHA-256 appears in that run's development manifest, without retraining or threshold changes. The executed recount removes eight `kos27` rows (seven normal, one defect), leaving 72 rows with selected-model confusion `[[62,0],[2,8]]`, recall 0.80 and precision 1.0; the baseline confusion is `[[62,0],[5,5]]`. The review committee's earlier 61 true-negative estimate was an arithmetic error; both independently executed recounts establish 62.

On Windows substitute `../.venv-win/Scripts/python.exe` for the interpreter. Tested Python dependencies are pinned in `../05_MEASUREMENT_TIMING/requirements-tested.txt`. The downloader verifies the recorded archive hash before extracting and refuses archive paths that escape the dataset directory.

Outputs include `inspection_report.json`, `split_manifest.csv`, `detector.npz`, and `heldout_cases.png`. Confusion matrices use rows=true, columns=predicted with order normal/defect. Precision is TP/(TP+FP), recall is TP/(TP+FN). A zero denominator yields zero. Errors are included by image ID and score; the montage shows failures first. Image content derives from the separately licensed dataset.

The 18 features summarize black-hat intensity and component size/density at fixed thresholds. They can confuse normal texture and lighting changes with defects, and miss weak or wide defects. The model classifies entire images; no pixel segmentation quality claim is made. Camera calibration is not applicable to this classification-only dataset. Its source provides no calibration board/metadata for geometric measurements.

Unreadable images, wrong dimensions/types and missing masks raise ValueError; no synthetic image silently replaces a missing dataset. Automated tests use explicit synthetic fixtures only to isolate behavior such as crack response, folder-level split leakage, cross-folder identical content (rejected by default, merged on request, transitive merging, within-folder duplicates reported but allowed) and the deduplicated recount. The actual demonstration uses the public industrial dataset.

## Multiscale extension, 2026-10-26

The original pipeline above remains reproducible. `multiscale.py` adds native-resolution
black-hat kernels at five scales/orientations, horizontal response pooling, row
concentration and component extent, including image edges. Together with the original
18 features, this produces 158 image features. Annotation masks supply binary labels
only; no annotation pixels enter feature extraction.

`selection.py` compares six predeclared logistic/random-tree candidates. All views
of a split group stay together (a split group is an item folder, or a merged set of
item folders under `--duplicate-policy merge`). In the historical runs the original
train/validation partitions formed a 319-image, 40-folder development pool whose
item `kos34` duplicates test item `kos27` byte for byte (see the erratum above). Four outer folds assess the entire selection
procedure; each outer training set chooses its candidate and threshold using three
inner grouped folds. A final five-fold development selection freezes the model and
threshold before scoring the 80-image original test partition. Threshold selection
maximizes F2 subject to precision >=0.80, with an explicit fallback when infeasible.

The old test images and results were already inspected in the earlier failure
analysis. They are a **previously observed descriptive comparison**, not fresh
independent qualification evidence. The protocol and six candidates were frozen
before this experiment in [QUALIFICATION_PROTOCOL_20261026.md](QUALIFICATION_PROTOCOL_20261026.md).

```sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
../.venv/bin/python qualify_inspection.py --dataset datasets/KolektorSDD --output ../08_TEST_RESULTS/vision_measurement/qualification_new --duplicate-policy merge
../.venv/bin/python inspect_image.py --model-directory ../08_TEST_RESULTS/vision_measurement/qualification_new --image datasets/KolektorSDD/kos08/Part2.jpg
```

Use a new output directory: qualification refuses an existing one. On the official
archive the command also needs `--duplicate-policy merge` (or it stops at the duplicate
guard before training). The first command returns exit code 1 when its extra engineering
target fails, while retaining the complete report/model/predictions. This is the measured
outcome below, not a missing implementation or a crashed evaluation. Single-image inference
needs no annotation. A missing/unreadable image raises `ValueError`; it is not classified
as normal. Inference verifies the model file's SHA-256 against `frozen_selection.json` before
loading it, then checks that its feature dimension matches the selected extractor.
A modified model or mismatched extractor fails explicitly before scoring. Metadata must select a known feature family, a finite numeric threshold in [0, 1] and a non-empty `model_file` name without path components; NaN, infinity, booleans, out-of-range thresholds, unknown families and a missing, empty, non-string or path-like model name raise `ValueError` before model/image scoring. Both probability endpoints are valid. The metadata remains a trusted local artifact, not a signed authorization or safety certificate.
For bit-exact repeated training, use the same pinned runtime and set the BLAS thread
variables above **before** starting Python. In PowerShell, use
`$env:OPENBLAS_NUM_THREADS='1'; $env:OMP_NUM_THREADS='1'`. The qualification runner
also sets OpenCV to one thread. Bitwise equality across different platforms or
library versions is not claimed.

| Evaluation (historical runs, folder-name grouping, kos27 = kos34 duplication present) | TN / FP / FN / TP | Precision | Recall | F1 |
|---|---|---:|---:|---:|
| Original observed-test baseline, as recorded | 69 / 0 / 5 / 6 | 1.0000 | 0.5455 | 0.7059 |
| New nested development procedure, as recorded | 274 / 4 / 4 / 37 | 0.9024 | 0.9024 | 0.9024 |
| New observed-test comparison, as recorded | 69 / 0 / 2 / 9 | 1.0000 | 0.8182 | 0.9000 |
| New observed-test comparison, excluding the 8 test images identical to development | 62 / 0 / 2 / 8 | 1.0000 | 0.8000 | 0.8889 |

The rows marked "as recorded" are retained exactly as the historical runs wrote them. Their
test partition contained eight images that also sat in the development pool, so they
overstate held-out performance; the `kos27/Part0` true positive (score 0.99986) is a
memorised duplicate of `kos34/Part0`, and in the original baseline it is a true positive
only because its score equals the validation-selected threshold exactly (the threshold is
that image's own validation score). The executed deduplicated row is 62 / 0 / 2 / 8 (recall 0.8000, precision 1.0000) over 72 rows. The selected model is multiscale logistic regression
with L2 penalty 0.05 and frozen threshold 0.5867173478388384; neither is retuned by the
recount. The first run averaged 0.4143 seconds/image for feature extraction on this
machine. Timing varies with system load. The predeclared additional target of >=0.85
recall and >=0.80 precision **FAILS** on observed-test recall as recorded (0.8182) and
fails by a wider margin once the duplicated defect is excluded (0.80 in the executed recount); it was
not relaxed afterward. The original request supplied no numerical acceptance threshold:
the implemented/evaluated pipeline and this extra quality target are separate claims.

Evidence is in the `run1/` and `run2/` folders under
`../08_TEST_RESULTS/vision_measurement/qualification_20261026/`:
`frozen_selection.json` records every candidate's development scores, `nested_development.json`
records inner/outer row assignments, `test_predictions.csv` and `development_manifest.json`
retain image hashes, and `selected_model.npz`/`.xml` stores the reusable model.
Nominal binomial intervals are reported with the explicit caveat that item views are
correlated. The 11 test defects (10 after deduplication) provide limited statistical
certainty. Failure images retain the dataset's CC BY-NC-SA 4.0 attribution and stay
outside source packages. Historical manifests written before 2026-10-28 have no `group`
column and name the split group in `item`; new manifests carry both columns plus
`duplicate_audit.json`, and `frozen_selection.json` / `qualification_report.json` record
the duplicate policy and an audit summary.

### Historical errata, 2026-10-28

- README (this file, earlier text): "Physical items are disjoint across partitions" and "item-disjoint" results — false for image content; corrected above.
- `QUALIFICATION_PROTOCOL_20261026.md` lines 15–19 and `FAILURE_ANALYSIS.md`: described the development pool and test partition as item-disjoint; errata appended to both files, original text retained.
- `inspection_report.json` `dataset` field of the historical baseline run: "item-disjoint custom split" — the file is frozen evidence and is not edited; this note is the correction.
- Prior review statements of "no leakage" (Fable milestone 2, the A/B/C–D follow-up review, the Codex disposition) were correct about the code path and incorrect about the data; they are outside this directory and are corrected by the master documentation owner.
- The `heldout_cases.png` montages of the historical runs print `pred=True`/`pred=False`; the title now prints 0/1. Historical PNGs are unchanged.


## Executed correction, 28 October 2026

The repaired workflow passes 43 tests. The actual default runners reject cross-item
content duplication before training. The audit found two connected pairs of item
folders, `kos01+kos26` and `kos27+kos34`, making 48 split groups from 50 folders.
The second pair crossed the historical development/test boundary; the first pair
also matters for content-safe development folds.

The standalone recount confirms historical matrix [[62,0],[2,8]] and 80% recall.
The explicit `merge` baseline and enhanced workflow then executed, with the latter
repeated twice. Both corrected runs have matrix [[69,1],[1,9]], precision/recall/F1
0.90. Nested development precision is 0.93023 and recall 0.95238. All model arrays,
prediction signatures, frozen selections, manifests and non-timing report fields
repeat exactly. Independent hash checks find no exact-content overlap across the
new development/test boundary or selection folds.

These are new splits of already inspected data. Their descriptive target PASS
neither erases the old 85% target FAIL nor establishes a fresh holdout result.
The two current mistakes are a false positive at `kos42/Part7` and a missed defect
at `kos50/Part4`; there was no result-driven retuning in this correction run.
Evidence: `08_TEST_RESULTS/committee_20261028/vision_policy/execution.json` and
`reproduction.json`, with the protocol recorded before execution. The image-level
binomial intervals assume independence, which correlated item views do not satisfy.
