# SWARMX scale/speed upgrade — experimental, not accepted yet

## Current size correction — 2× original (2026-10-04)

The operator requested the midpoint between original 1× and experimental 3×.
Current chassis: **1.80 × 1.40 × 0.40 m**; wheel-inclusive collision footprint:
**1.80 × 1.52 m**. Wheel radius 0.22 m; separation 1.40 m. Mass scales ×8,
inertia ×32; chassis mass 280 kg. LiDAR offset (0.24, 0, 0.66) m;
IMU offset (0, 0, 0.50) m. Navigation radius 1.18 m + 0.15 m margin;
peer hard gap 3.18 m retains the original 0.82 m pair buffer. LiDAR stationary
stop distance is 1.30 m, preserving the 0.64 m front buffer.

All five share this model. Physical geometry, navigation hull, sensor TF and
runtime safety distances are updated together. Warehouse layout is unchanged.
The operator revised the target to **1.80 m/s (3× original speed)**.
Default speed remains **0.60 m/s** until physical speed gates pass.
Stages: 0.60, 0.90, 1.20, 1.50, 1.80 m/s, then concurrent fleet validation.
The latest 3× short-turn traction gate passed, but does not validate 2×.
No earlier physical PASS is transferred to the resized platform.

### Current 3× speed validation checkpoint

- Candidate: 2× geometry, target 1.80 m/s; not yet accepted at final speed.
- Current-controller 0.60 m/s: motion, priority, forward reroute and occupied-bay
  charging approach PASS (`SWARMX_LARGE_20261004-153939_0.6_*.json`).
- Fresh 0.90 m/s motion PASS (`SWARMX_LARGE_20261004-160442_0.9_motion.json`):
  peak 0.900 m/s, measured stop 0.511 m, peak lateral speed 0.167 m/s.
  Other gates and higher speeds remain pending at this checkpoint.
- Moving substantial corrections brake before angle-as-needed alignment.
  Long final legs now cruise normally, then brake using remaining distance,
  reaction travel and terminal tolerance; the final metre remains slow.
- Conservative stopping estimate at 1.80 m/s: 2.492 m at 0.65 m/s², plus
  0.630 m reaction travel. Physical DiffDrive deceleration is 0.80 m/s².
  Stationary LiDAR stop threshold 1.300 m rises to 4.422 m at 1.80 m/s.
  The head-on two-robot safety envelope rises from 3.180 m at rest to
  9.425 m when both robots travel at 1.80 m/s.
- The initial 0.60 m/s attempt failed on sideways slip; its failed report is
  preserved. The 15:39 attempt at 0.90 m/s completed its route but hung before
  exporting measurements; it is explicitly INCOMPLETE, not a physical PASS.
  Validator evidence is now exported before bounded ROS cleanup. Resume uses
  the unchanged current-controller 0.60 m/s passes, not original V2 evidence.

## Historical 3× implementation and evidence (superseded geometry)

Baseline commit: `2db4adf`. Original model, world, configuration and launcher
remain available. The larger model is NOT the default demo.

1. Original chassis visual/collision: **0.90 × 0.70 × 0.20 m**.
2. Enlarged chassis visual/collision: **2.70 × 2.10 × 0.60 m**.
3. Uniform linear scale: **3.0**, all five identities use one identical model.
   Mass scales ×27 and inertia ×243 for constant density. Chassis mass 945 kg;
   each wheel 40.5 kg. Sensor masses/inertias and all physical offsets also scale.
4. Actual model collision XY bounds, including rotated wheel cylinders:
   **x=[−1.35,+1.35], y=[−1.14,+1.14] m**; full **2.70 × 2.28 m**.
   Tested by reading collision geometry, not by assuming the chassis bounds.
5. Navigation polygon: (−1.35,−1.05),(−1.23,−1.14),(1.23,−1.14),
   (1.35,−1.05),(1.35,1.05),(1.23,1.14),(−1.23,1.14),(−1.35,1.05).
   This is the original physical enclosing polygon ×3, not a visual-only scale.
6. Fallback diameter **3.54 m**, radius **1.77 m**. The exact bounding-box
   turning radius is 1.76695 m.
7. Navigation centre clearance **1.92 m** = 1.77 m physical circle + a separate
   **0.15 m** obstacle margin. Peer hard gap **4.36 m** retains the original
   **0.82 m** pair comfort buffer. The original **0.64 m** forward-obstacle
   buffer is also retained. Inflation is not added to the physical model.
   Original 2.70 m inner aisles fail the turning/navigation envelope of 3.84 m
   by **1.134 m** (using the exact circle; rounded footprint requirement 3.84 m).
   Original pickup shelf-face gap 1.35 m fails new centre clearance by **0.57 m**.
8. Shelf sizes and 50 × 40 m bounds are unchanged. Shelf columns move to
   x=7.225,17.225,32.775,42.775 m; all four side aisles are **4.20 m** wide.
   Central main gap remains **9.75 m**; cross-row gaps remain **5.90 m**.
   Pickups move south to y=24.4,17.4,10.4 m; drop locations are unchanged.
   Corner pallets move to y=1.5/38.5 m to unblock the enlarged outer paths.
   Five top chargers: x=15.4,20.2,25,29.8,34.6 m; y=37.4 m; pitch **4.80 m**.
   Own entry lines remain y=32.5, giving **4.90 m** clearance below occupied
   bays. Rear rail moves to y=39.6 and widens to 22.6 m. Bay-to-rail gap 2.11 m.
9. Original validated maximum forward speed: **0.60 m/s**.
10. Requested final maximum: **2.40 m/s**; not yet physically accepted.
11. Exact requested speed multiplier: **4.0**, not +400% / 5.0.
12. Controller acceleration **0.65 m/s²**; physical acceleration **0.80 m/s²**.
13. Physical deceleration **0.80 m/s²**, conservative planning **0.65 m/s²**.
    Immediate zero safety commands request physical braking;
    they do not teleport the robot to zero velocity. The motion gate measures
    an actual stop from near each tested speed using the existing enable service.
14. Conservative braking estimate at 2.40 m/s: **4.431 m** at 0.65 m/s²;
    physical-plugin ideal estimate **3.600 m** at 0.80 m/s². These are estimates,
    not final measurements. Reaction allowance **0.35 s** covers the 10 Hz
    controller, 5 Hz peer publication and provisional local decision allowance.
15. Candidate angular controller cap **0.50 rad/s**, physical cap **0.60 rad/s**,
    physical angular acceleration **±0.60 rad/s²**. Still subject to live gates.
    Proportional angle-as-needed steering is retained; no fixed-90° primitive.
16. Straight lookahead **1.20 + 0.80×current speed**, capped at **3.60 m**;
    sharp bends shorten it to 0.55 m. Targets stay monotonic on the active leg.
    Every real bend is preserved; centreline arrival tolerance is 0.18 m.
    Reaction-time centre sweeps outside the footprint-safe graph are refused.
17. Prediction horizon **12 s**, now with acceleration-aware travel times and
    the actual body extent in occupancy windows. Upcoming zones within that
    horizon are considered even before they become the nearest zone. Original
    single-next-zone behavior remains unchanged for nonexperimental profiles.
18. Dynamic peer envelope: 3.54 + 0.82 + projected relative closing speed×0.35
    + both projected approach braking distances v²/(2×0.65). At head-on
    2.40+2.40 m/s this is **14.902 m**; at rest it is **4.36 m**.
    A stopping-time relative sweep prevents a nearby but safely waiting robot
    outside the winner's path from triggering a large radial emergency stop.
19. Incremental stages: **0.60,0.90,1.20,1.50,1.80,2.10,2.40 m/s**.
    Current acceptance: **PENDING**. `SWARMX_LARGE_STAGE_PROGRESS.json` is written
    by the live runner; failures stop it before starting any higher speed.
20. Five-AMR high-speed test: **NOT RUN / NOT ACCEPTED** until earlier gates pass.
21. Minimum separation: preliminary enlarged 0.60 m/s runs sampled **4.20 m**.
    Initial motion static gap **2.36 m**; corrected charger-layout crossing
    static gap **2.51 m**. These preliminary results do not certify later revisions.
22. Tests: original 141 tests passed after opt-in edits; geometry/safety tests
    check the real model, exact world/graph match, 420 route combinations,
    occupied-neighbour charger clearance, braking/relative speed and scan strips.
    Initial motion PASS; crossing initially exposed late nearest-zone prediction
    and boundary contraction stop/start cycling. Failed attempts are retained.
    A preliminary future-zone crossing passed priority/clearance/completion but
    showed yield cycling in logs, so it is NOT accepted as a clean encounter.
    The tightened yield latch is being revalidated with an oscillation failure gate.
    The stricter motion gate exposed corner chasing; this remains a FAIL until
    the corrected speed-to-distance approach and angular-settling gate pass.
    A measured 0.60 m/s stop covered **0.227 m** in that failed motion attempt;
    braking passed, but it does not override the failed corner behavior.
23. Limitations: no final speed or fleet acceptance is claimed. Collision gates
    use sampled physical odometry plus conservative circle margins; they are not
    contact-sensor measurements. Battery remains simulated. LiDAR is horizontal
    at z=0.99 m after exact scaling: it cannot see objects entirely below that
    plane; known pallets remain protected by the physical-map graph. A speed
    reduction is not being substituted for the requested 2.40 m/s final target.
24. Exact commands, in **Ubuntu Terminal**, after stopping any other Gazebo:

    ```bash
    cd /home/maaz-wasi/amr_ws
    source /opt/ros/lyrical/setup.bash
    colcon build --packages-select amr_simulation edge_ai_nav --symlink-install
    source install/setup.bash
    python3 validate_v2_large_stages.py --visible
    ```

    Experimental viewing only (default cap 0.60 m/s, no accepted 4× claim):

    ```bash
    bash run_swarmx_v2_large.sh enabled:=false use_rviz:=true
    ```

    Dashboard: http://localhost:8091/fleet. Original demo rollback:

    ```bash
    ./run_swarmx_v2_demo.sh enabled:=false use_rviz:=true
    ```
