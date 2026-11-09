# Reading guide

Start with one runnable example, inspect its tests, and then follow the implementation.
The order below builds from timing and kinematics toward integrated simulations.

| Topic | Start here | Questions to investigate |
|---|---|---|
| FPGA timing | [Specification](https://github.com/nabwei/verilog-golden-regression/blob/main/01_FPGA_EMBEDDED/SPECIFICATION.md) | How are reset priority, rollover and consistent reads tested? |
| Measurement | [Uncertainty and timing](https://github.com/nabwei/measurement-testbench/tree/main/05_MEASUREMENT_TIMING) | Which assumptions enter the covariance matrix and coverage factor? |
| Mobile robots | [C++ robot core](../02_ROBOTICS_NAVIGATION/robot_core/README.md) | How do kinematics, localization, planning and control connect? |
| ROS 2 navigation | [Integration](../02_ROBOTICS_NAVIGATION/ros2_navigation/README.md), [qualification](../02_ROBOTICS_NAVIGATION/ros2_navigation/QUALIFICATION.md) | Which frame contains each pose? What changes between SLAM v1 and v2? |
| Industrial vision | [Inspection pipeline](../03_COMPUTER_VISION/README.md) | How do duplicate images affect evaluation? Why is the quality gate still failing? |
| Sensor fusion | [Relative VIO](../04_SENSOR_FUSION/RELATIVE_VIO.md), [noise model](../04_SENSOR_FUSION/NOISY_FRAME_MODEL.md) | How are shared-frame correlations represented? When should initialization reject data? |
| Arm planning | [PyBullet arm](../02_ROBOTICS_NAVIGATION/arm/README.md) | What does finite-resolution collision checking miss? |
| Telemetry | [MQTT service](../02_ROBOTICS_NAVIGATION/fleet/README.md) | How are duplicates, missing sequences and delayed messages handled? |
| UAV inspection | [PX4 integration](../06_UAV_DIGITAL_TWIN/px4_integration/README.md) | What does depth reconstruction establish, and what remains unverified? |

For each module, reproduce a successful run and a failure case. Check the units,
reference frames and assumptions before interpreting a plot or an accuracy number.
Current results and open issues are in the [validation summary](../docs/VALIDATION.md).

## Technical vocabulary

| English | Deutsch |
|---|---|
| uncertainty propagation | Unsicherheitsfortpflanzung |
| clock synchronization | Taktsynchronisation |
| coordinate frame | Koordinatensystem |
| state estimation | Zustandsschätzung |
| path planning | Pfadplanung |
| collision checking | Kollisionsprüfung |
| reproducibility | Reproduzierbarkeit |
