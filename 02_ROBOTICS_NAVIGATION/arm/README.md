# J — Planar arm planning and control

Original two-joint URDF, PyBullet collision checking and forward dynamics, seeded RRT* with parent selection/rewiring and descendant cost propagation, cubic segment interpolation, and bound-constrained receding-horizon joint control. The controller uses a velocity-integrator prediction model; PyBullet supplies the actual controlled plant. This is an educational planar arm, not an industrial manipulator certification.

From this folder, using the portfolio Python environment:

```sh
../../.venv/bin/python -m unittest -v test_arm test_planning
../../.venv/bin/python run.py --output ../../08_TEST_RESULTS/robotics/arm
```

Dependencies: NumPy, SciPy, PyBullet 3.2.7, Pillow. Versions are captured in the workspace environment lock. No external robot assets are downloaded; `arm.urdf` is original.

`planning.py` works in joint-space radians with bounds ±2.8, 0.25-radian growth steps, 0.65-radian rewiring radius, and collision samples at no more than 0.025 rad joint-space distance. This discretization is finite-resolution collision checking, not a continuous collision proof. Clearance is 15 mm. Adjacent links share a hinge; only arm-obstacle collisions are tested because there are no nonadjacent arm links. The demonstration uses 450 iterations and a fixed random seed; finite-budget RRT* can fail and raises explicitly.

`control.py` solves an eight-step box-constrained quadratic problem with SciPy L-BFGS-B every 50 ms of simulated time. Velocity limits are ±1.2 rad/s. Every predicted control edge and actual reached configuration is collision-checked. The simulation records joint replay and an image; goal joint-space error must be below 0.02 rad.

Tests compare analytical forward kinematics with PyBullet, catch collision-query mutation of physical state, test routing around a forbidden region/blocked endpoints, and verify velocity limits and error reduction. Numerical optimizer success is checked; simulation failures are never converted into PASS.

Reference: [PyBullet official quickstart](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html).
