# Ownership and third-party boundaries

The root MIT license applies to independently authored portfolio code and
documentation owned by the contributors. The owner selected MIT on 28 October
2026. It does not relicense private/university research, acquired datasets,
upstream packages, or separately licensed assets and notices.

- The public blade geometry retains its MIT license and pinned provenance in
  `06_UAV_DIGITAL_TWIN/assets/public_blade/`.
- ROS navigation references installed upstream software; retained license texts
  and attribution are in `02_ROBOTICS_NAVIGATION/ros2_navigation/third_party/`.
- KolektorSDD data and derived images retain their dataset terms, described in
  `03_COMPUTER_VISION/DATASET.md`. Dataset images are excluded from source bundles.
- EuRoC acquisition and attribution are documented in
  `04_SENSOR_FUSION/EUROC_ACQUISITION.md`; acquired data are excluded.
- PX4 and Gazebo assets are external dependencies. Their licenses are not replaced
  by the portfolio license; see `06_UAV_DIGITAL_TWIN/px4_integration/README.md`.
- Local raw evidence and private source-audit records are excluded from public
  source bundles. A recovery Git bundle is not a publication package.

The owner authorized publication of this source portfolio on 9 November 2026. Dataset and third-party permissions remain separate.
