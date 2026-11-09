# Upstream configuration attribution

The generated `nav2_params.yaml` evidence files derive from the installed `nav2_bringup/params/nav2_params.yaml`, package version 1.3.13 (Ubuntu package `1.3.13-1noble.20261006.042503`). Nav2 contributors retain their copyright. The installed package declares Apache-2.0; its notice and full license are preserved here. Source: https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup .

Local modifications on 2026-10-25 are made by `configure_navigation.py`: selection and limits of the RegulatedPurePursuit controller, movement progress thresholds, collision-monitor frame/height, and velocity-smoother limits. All other serialized settings come from the installed upstream defaults. Generated YAML is a derivative configuration, not original portfolio authorship.

Qualification changes on 2026-10-26 add a 2000 ms behavior-tree action handshake timeout, conservative angular/acceleration limits, and a 0.1 s SLAM minimum processing interval. Native Gazebo model/truth tests and controlled planner fixtures are original portfolio code. The test does not modify upstream binaries.

SLAM Toolbox 2.8.5 (Ubuntu package `2.8.5-1noble.20260915.204151`) supplies `config/mapper_params_online_async.yaml` at runtime. The portfolio launch files reference that installed file and override frame names, clock mode, lifecycle management and movement thresholds. The package declares LGPL; the upstream Jazzy license is LGPL-2.1 and is preserved here. Source: https://github.com/SteveMacenski/slam_toolbox/tree/jazzy .

Retrieved license blob identifiers: Navigation2 `ce2ed3dff819a38de11b17883e38dace87fb4922`; SLAM Toolbox `8000a6faacf471c537530805ab29523c7732e11a`. The owner selected MIT for original portfolio source on 28 October 2026; publication still requires explicit approval. This notice does not relicense upstream code. Binary environments are not bundled in source-only candidates.

## Verbatim upstream SLAM Toolbox source file retained in the evidence tree

A verbatim copy of upstream slam_toolbox 2.8.5 `src/slam_toolbox_common.cpp` is retained ONLY in the local evidence tree, at `08_TEST_RESULTS/robotics/fable_review2_20261026/upstream_slam_toolbox_2_8_5_common.cpp`. It was saved beside the configured-node parameter probe so that the probe record can cite lines 413–418 (declaration/read of `check_min_dist_and_heading_precisely`) and lines 801–808 (rotation-or-distance scan admission when the parameter is true). The copy is NOT part of this ROS package, is NOT included in any source package or candidate archive, and is not compiled, imported or referenced by any build file here. Upstream location: https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp .

Two licence facts about that file are recorded without deciding between them. First, the file's own header (lines 1–18 of the retained copy) carries a Creative Commons Public License notice and reads, verbatim:

```
/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */
```

Second, the installed Ubuntu package and the upstream Jazzy tree declare LGPL-2.1 for slam_toolbox, and that licence text is preserved in this directory (see above). This notice states both facts and the upstream URL; it does not claim which licence governs the file, does not relicense it, and does not apply the portfolio MIT licence to this upstream file. If the evidence tree is ever redistributed, the copied file should be removed or replaced by the URL and line citation above.
