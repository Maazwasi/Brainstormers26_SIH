# PHASE 0.5 REPORT

## Root cause and fix

`LocalController.lookahead_target()` unconditionally accessed `route[first+1]` even when `route` held one point. `task_assignment()` accepts a one-point route, so a robot outside that point's goal tolerance could crash. The function now returns `None` for an empty route and returns the sole/final target when fewer than two uncompleted points exist. The existing `tick()` completion branch handles empty and reached routes before invoking lookahead. Multi-point projection and monotonic forward progress are unchanged.

## Physical footprint

The active Gazebo SDF collision elements are one 0.60 × 0.45 m chassis box and four wheels. Each wheel is a radius 0.085 m, length 0.04 m cylinder, centered at x = ±0.18, y = ±0.235, rolled 90° so its cylinder length extends in Y. No lidar, bumper, or sensor-mount collision geometry exists.

Measured union of collision bounds in the robot frame:

- X: −0.300 to +0.300 m
- Y: −0.255 to +0.255 m
- Overall: 0.600 × 0.510 m

The chassis corners are 0.375 m from the centre, so a circular fallback needs a 0.750 m diameter. Configuration previously said 0.360 m. It now records the actual bounds and an eight-vertex polygon enclosing the chassis and wheels. The Nav2 local and global costmaps use that polygon. The existing scalar graph collision clearance changes from 0.360 to 0.375 m. This is the circumscribed physical radius, not a predictive conflict distance.

Separate margins were retained: `stage6.footprint_margin = 0.45 m`, `stage6.safety_margin = 0.55 m`, `stage6.safe_deceleration = 0.65 m/s²`, and obstacle stop distance `0.85 m`. They were not used to inflate the Gazebo physical model.

Search of active `src` files found no remaining `0.36` robot-size or `robot_radius: 0.22` assumptions. Other decimal occurrences are unrelated coordinates or colours. The legacy Waffle model remains untouched and is not launched by the active fleet.

## Files changed in this phase

- `src/edge_ai_nav/edge_ai_nav/ros_nodes/local_waypoint_controller.py`: cardinality guard in lookahead.
- `src/amr_simulation/config/warehouse_sih_demo.yaml`: physical bounds, polygon, circular fallback diameter, graph clearance.
- `src/amr_simulation/config/nav2_params.yaml`: local/global costmap polygon.
- `src/edge_ai_nav/test/test_lookahead_cardinality.py`: seven route-cardinality and forward-progress tests.
- `src/edge_ai_nav/test/test_footprint_geometry.py`: collision-to-configuration test.

Pre-existing uncommitted changes to several of these files were preserved. `SWARMX_V2_BASELINE.md` was created in Phase 0, before this phase.

## Validation

- Build: `colcon build --packages-select amr_simulation edge_ai_nav --symlink-install` — PASS, both packages.
- Complete existing and new test suite: **66 passed** (58 baseline + 8 new).
- Focused lookahead tests: **7 passed**.
- Footprint test parses every active SDF collision and checks the configured bounds, fallback radius, graph clearance, and both Nav2 polygons: **PASS**.
- Graph with 0.375 m clearance: initialized with 36 nodes / 36 edges — PASS.
- Live ROS 2/Gazebo: ALPHA single-goal straight motion to `[-9.2, 0]` — completed, about 0.896 m physical odometry, no controller exit.
- Live ROS 2/Gazebo: ALPHA three-point route `[-9.2,0] → [-8,0] → [-8,1]` — intermediate waypoints and turn observed; final task completed, then normal charging return started. No `IndexError`, controller crash, backward lookahead, or spinning observed during these runs.

The initial Gazebo GUI/server process exited cleanly during setup. The same world was restarted as a headless Gazebo server for the live motion checks. The original launch's bridges and controllers remained alive. Shutdown of some ROS bridge processes later reported signal-related errors; this did not interrupt the successful route observations.

**PHASE 0 BASELINE GATE: PASS for the two requested defects.** Phase 1 was not started.
