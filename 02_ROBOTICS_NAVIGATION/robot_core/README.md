# G — Differential-drive robot core

Independent C++17 planar simulator with separate kinematics, world/collision, LiDAR, estimation, planning and control modules. The existing `nabwei/pid_regler_c_main` controller was inspected at commit `e27cc8fac68e863b2615947cc295cedea257c80b`; it already implements derivative-on-measurement PID, trapezoidal integration and conditional antiwindup. This project adds robot kinematics/EKF/geometric control and does not recreate that PID.

## Reproduce (Ubuntu / WSL)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/robot_demo trajectory.csv
python3 render.py trajectory.csv replay.svg
```

No third-party C++ libraries are required. Validated compiler: GCC 13.3.0 on Ubuntu 24.04. ASan/UBSan build uses `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"` in a separate build directory.

## Implemented contracts

- SI units: positions/ranges in metres, time in seconds, wheel rates in rad/s, heading in radians.
- Exact constant-twist integration uses a sinc limit for zero yaw rate. `wheel_twist` converts independent left/right wheel rates.
- Occupancy-grid A* uses four-connected cells and a Euclidean admissible heuristic. Robot-disc clearance and conservative swept-segment checks prevent corner crossing and tunneling. Failure returns an empty path.
- Pure Pursuit computes curvature to a lookahead point, clamps angular velocity, and rotates in place for targets more than 60 degrees off heading.
- EKF state is `[x,y,heading]`; nonlinear motion prediction and a position-only observation use the Joseph covariance update. Position observations are a synthetic external positioning sensor, **not LiDAR scan matching**. Process noise is illustrative, not an identified physical noise model.
- LiDAR casts grid-tested rays at `resolution/50` increments; obstacle ranges are quantized upward by at most one increment. The finite step is part of the model.
- A collision or mission timeout throws, causing a nonzero test exit. Invalid geometry/time input is rejected.

The demo follows a two-wall route, adds seeded position noise, exports actual and estimated positions and minimum LiDAR range, and must reach the goal within 0.15 m. Tests include analytical straight/circular motion, rotation, LiDAR wall distance, swept collision, detours, unreachable goals, position correction and covariance symmetry/positive diagonal. Sanitizers and exact repeated replay hashes are saved in `../../08_TEST_RESULTS/robotics/`.

Scope limits: planar kinematics, no wheel dynamics/slip, no SLAM, no ROS 2 integration, no real sensor accuracy claim. Fixed-seed reproducibility is verified within the tested compiler/runtime; C++ normal-distribution implementations may differ across standard libraries.
