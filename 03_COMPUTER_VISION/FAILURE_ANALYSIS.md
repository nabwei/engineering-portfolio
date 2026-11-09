# Held-out failure analysis

First run, seed 2026, 80 held-out images: TN=69, FP=0, FN=5, TP=6. Precision=1.0, recall=6/11=0.54545 and F1=0.70588. These are small-sample empirical values, not deployment guarantees. The classifier misses nearly half of the labeled test defects. It must not be used to accept parts without further validation.

The five misses are kos08/Part2, kos32/Part2, kos38/Part0, kos38/Part1 and kos50/Part4. The montage was visually inspected. Several contain thin approximately horizontal cracks extending from an image edge, amid strong surface texture. Global feature summaries discard spatial arrangement, and resizing can attenuate thin cracks. Border exclusion can discard part of an edge-originating crack. These are plausible mechanisms suggested by visual inspection, not causal conclusions from an ablation study.

The validation-selected threshold is approximately 0.7303. Failed cases have scores between approximately 0.276 and 0.469; a lower threshold would recover some defects and increase false positives. The threshold has not been retuned using test labels. The current baseline and its original held-out results remain frozen for reproducibility.

Next experiments should use new validation partitions or nested group cross-validation: compare full-resolution features, spatially localized features, oriented line filters, and a learned segmentation model. Investigate border cropping through controlled ablations. Preserve physical-item grouping. For an industrial acceptance target, specify the tolerated missed-defect rate, select a threshold under that criterion using validation data, then evaluate once on fresh held-out items. No numerical target was supplied for this baseline.

## Frozen extension experiment, 2026-10-26

The new development-only nested grouped experiment recovers kos32/Part2,
kos38/Part0 and kos38/Part1 while retaining zero false positives on the previously
observed test partition. It still misses kos08/Part2 (score 0.478974) and
kos50/Part4 (score 0.026717), below the frozen threshold 0.586717. Confusion is
[[69,0],[2,9]], precision 1.0, recall 0.81818 and F1 0.9.

The saved montage was visually inspected. Both remaining misses show partial
edge-originating dark lines among textured surfaces. Their different scores suggest
that lowering one threshold is not a comprehensive repair; no threshold was changed
after observing these errors. Global/pooled summaries still discard the full spatial
configuration, and visual similarity alone cannot establish the causal failure.

The nested development procedure achieved [[274,4],[4,37]], precision/recall 0.90244.
Outer-fold recall ranged from 0.8 to 1.0, showing variability hidden by a single
aggregate number. The final development-selected candidate differs from some outer
fold winners; that is why nested procedure performance is reported separately from
the selected candidate's more optimistic development selection score.

The extra predeclared 85% recall target fails on 9/11 observed-test defects. The
nominal 95% recall interval there is approximately [0.4822,0.9772], before accounting
for within-item dependence. This does not establish production reliability. Fresh
items and an agreed application-specific false-negative cost are needed for a new
industrial qualification claim. The original baseline and all failed target evidence
are preserved; no test-driven parameter revision was made.

## Erratum, 2026-10-28: the test partition was not content-disjoint

Everything above is retained as written. It assumed that grouping by folder name made
the 80-image test partition disjoint from development. The recorded manifests show
otherwise: all eight images of test item `kos27` are byte-identical to the eight images
of development (validation) item `kos34`. Consequences for the analyses above:

- Original baseline: the validation-selected threshold 0.7303149860036773 is the
  validation score of `kos34/Part0`; the identical `kos27/Part0` therefore scores exactly
  the threshold in the test partition and counts as a true positive by the `>=` tie rule.
  Without that row the baseline's observed-test defects are 10, of which the frozen
  predictions find 5, not 6/11.
- Frozen extension experiment: `kos27/Part0` scores 0.999859, far above the frozen
  threshold, because the model was fitted on the same pixels. It is the ninth true
  positive counted above. Excluding the eight duplicated rows leaves 72 test rows; the
  executed recount of `run1/test_predictions.csv` is TN 62, FP 0, FN 2, TP 8
  (recall 0.80). `dedup_observed_test.py --run .../qualification_20261026/run1` recounts
  the recorded predictions without retraining and its output is the record; the review
  committee's figure of TN 61 was an arithmetic error; independent executed recounts establish TN 62.
- Nested development estimate [[274,4],[4,37]]: the duplicate partner of `kos34`
  (`kos27`) lay outside the development pool, so this pair did not cross a development
  fold; the executed content audit also finds eight pairs connecting `kos01` with `kos26` inside development. There are 16 cross-item duplicate groups in total, eight crossing the historical development/test boundary. See `../08_TEST_RESULTS/committee_20261028/vision_duplicate_audit.json`.
- The two remaining misses `kos08/Part2` and `kos50/Part4` are unaffected; the 85%
  target fails as recorded and fails further after exclusion.

The misses analysed above remain the real failure cases. The historical model and its threshold remain unchanged. The separately declared observed-data re-split uses the content-hash guard described in
`README.md` and `DATASET.md`, which rejects or explicitly merges identical content
before any partition is drawn.
