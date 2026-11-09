# Dataset provenance and scope

Official resource: https://www.vicos.si/resources/kolektorsdd/

Downloaded 2026-10-25 from https://data.vicos.si/datasets/KSDD/KolektorSDD.zip

Archive size: 101831129 bytes. SHA-256:
`65dc621693418585de9c4467d1340ea7958a6181816f0dc2883a1e8b61f9d4dc`

KolektorSDD is distributed as 399 surface images in 50 item folders, including 52 images with visible defects. The folder count is not a count of distinct image content: the eight images in folder `kos27` are byte-identical (equal SHA-256) to the eight images in folder `kos34`; only their label bitmaps differ as files, and both carry one defect image. This was established on 2026-10-28 from the recorded manifests (`../08_TEST_RESULTS/vision_measurement/inspection/split_manifest.csv`, `qualification_20261026/run1/test_predictions.csv` and `development_manifest.json`). The executed content audit also finds eight identical-image pairs between `kos01` and `kos26`: 16 cross-item content groups, 32 involved images and 383 distinct file hashes overall. Only the eight kos27/kos34 groups crossed the historical development/test boundary; the others also affect development folds. This is exact file-byte detection; near-duplicates or re-encoded copies are outside its scope. Attribution: Kolektor Group and ViCoS Lab; Domen Tabernik, Samo Šela, Jure Skvarč, Danijel Skočaj, *Segmentation-Based Deep-Learning Approach for Surface-Defect Detection*, Journal of Intelligent Manufacturing, 2019, DOI https://doi.org/10.1007/s10845-019-01476-x.

Dataset license: CC BY-NC-SA 4.0, https://creativecommons.org/licenses/by-nc-sa/4.0/. The original archive and extracted images remain ignored local dependencies. The derived `heldout_cases.png` montage carries the same attribution and CC BY-NC-SA 4.0 terms and must not be relicensed as MIT. Code does not include dataset pixels. Public release should link the source and keep the license notice with any montage.

This implementation uses custom 60/20/20 splits at seed 2026 over split groups, not the authors' official three-fold benchmark. Preprocessing resizes to 256x704 to bound runtime; published benchmark recommendations use 512x1408. Scores cannot be compared directly to published benchmark scores. No geometric accuracy is claimed; camera calibration is inapplicable without a calibration target or camera metadata.

## Duplicate policy (disclosed, 2026-10-28)

Earlier text here described the splits as "physical-item-disjoint". That held for folder names only. Because `kos27` and `kos34` share identical content, the historical seed-2026 split placed the same eight images in the test partition and in the validation partition, and the historical numbers in `README.md` and `FAILURE_ANALYSIS.md` are contaminated accordingly (retained, labelled). The loader now audits SHA-256 content before splitting and applies one of two explicit policies selected on the command line:

- `reject` (default): stop with an error if identical content appears in more than one item folder. The official archive stops here.
- `merge`: fuse the linked folders into one split group (the actual dataset produces `kos01+kos26` and `kos27+kos34`), so the content stays on one side of every split and fold; the merged group and the original folder are both recorded in the manifests.

No folder name is special-cased in code. Merging changes the number of split groups and therefore the seed-2026 permutation, so a run under `merge` is a new experiment with a different partition, not a reproduction of the historical split. `duplicate_audit.json` is written beside every run.

Audit: an existing public phase-shift demonstration implements fringe synthesis and phase extraction, and a separate private existing repository implements stereo geometry. Neither is an industrial classifier, and neither was copied. Project D is NEW. See `../05_MEASUREMENT_TIMING/AUDIT.md` for the inspection record.
