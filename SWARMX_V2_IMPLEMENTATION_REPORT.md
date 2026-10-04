# SWARMX V2 FINAL IMPLEMENTATION REPORT

Updated 2026-10-04. PASS applies to the specified tested simulation gates, not
an unconditional collision-free guarantee or real-hardware qualification.

1. **Status: PASS — implementation and tested demo gates complete.** Baseline,
   0.50 and 0.60 m/s motion/fleet checks PASS; focused priority, same-priority,
   yield/resume, recovery, reroute/rejoin, fresh encounters and charging PASS.
   Charging-departure regression fixed and physically revalidated. Final
   default is 0.60 m/s; Gazebo GUI, RViz and live dashboard are running.
2. **Created files in this worktree:** V2 warehouse YAML, world SDF and xacro,
   robot xacro, Nav2 YAML; `swarmx_v2_demo.launch.py`, V2 RViz configuration,
   `run_swarmx_v2_demo.sh`, `validate_v2_fleet.py`, `validate_v2_occupied_bay.py`,
   `path_safety.py`, V2 graph/geometry/robot/steering/priority/proximity/yield/
   LiDAR tests, lookahead and footprint tests, phase/checkpoint reports.
   Exact created/untracked paths are enumerated in `SWARMX_V2_WORKTREE_FILES.txt`;
   retained evidence includes `SWARMX_V2_VALIDATION_EVIDENCE.json`,
   `SWARMX_V2_IMU_GATE.json` and `SWARMX_V2_VISUAL_SYNC_GATE.json`.
   Other pre-existing untracked work includes the legacy P2P demonstration
   helper/launcher/tests and Phase 1 preview launcher. Preserved without removal.
3. **Modified files:** README; graph, controller, peer-state, negotiation,
   prediction, task-bidder, observer, dashboard backend and its HTML/CSS/JS;
   legacy spawn helper/launch, package setup, negotiation tests. Existing
   legacy warehouse/Nav2 changes are preserved. Exact modified paths are in
   `SWARMX_V2_WORKTREE_FILES.txt`. No commit or push is implied.
4. **Tests:** 141 passed across `src/edge_ai_nav/test` and
   `src/edge_ai_nav/tests`. Original baseline was 69.
5. **Build:** `amr_simulation` and `edge_ai_nav` PASS. `git diff --check` PASS.
6. **Graph:** 54 named nodes in the approved 50 × 40 m warehouse. Twelve
   pickups, seven drops, five individual south charging entries, central
   corridor and four side corridors linked through four cross aisles.
7. **Main/alternate paths:** central edges use normal distance cost; side
   edges use multiplier 1.08. A* considers real alternatives when zones or
   edges are unavailable. Transit uses y=8 rather than crossing other drops.
8. **Mapping:** ALPHA 1–5 use `/alpha_1`–`/alpha_5`, matching Gazebo model
   names, red/blue/green/orange/purple identity panels and isolated sensors/TF.
9. **Old dimensions:** 0.60 × 0.45 m chassis; physical x ±0.300, y ±0.255 m.
10. **New dimensions:** 0.90 × 0.70 × 0.20 m chassis, four driven wheels with
    0.11 m radius and 0.70 m separation.
11. **New collision bounds:** x ±0.45 m, y ±0.38 m, derived from model geometry.
12. **Footprint:** eight-vertex polygon matching chassis/wheel envelope;
    conservative diameter 1.18 m. Nav2 and fleet configuration agree.
13. **Graph clearance:** 0.59 m, half the conservative footprint diameter.
14. **Old maximum linear speed:** 0.40 m/s.
15. **Final launch maximum linear speed:** 0.60 m/s, accepted through staged
    0.50/0.60 motion and five-robot fleet gates plus focused high-speed checks.
    Cruise cap and measured peak are 1.5× baseline. Mean moving fleet speed
    was 0.337 m/s versus baseline 0.299 m/s; no 50% mission-time claim.
16. **Acceleration/deceleration:** physical linear ±0.8 m/s²; controller
    acceleration cap 0.65 m/s², immediate safety-stop request; physical
    angular acceleration ±1.5 rad/s². Four-wheel rolling friction 2.0,
    lateral friction 0.05, wheel-local rolling direction (1,0,0).
17. **Angular velocity:** final controller cap 0.90 rad/s, model limit 1.2 rad/s.
18. **Lookahead:** base 0.75 m, speed-adaptive up to 1.20 m, shortened to
    0.55 m at sharp bends; monotonically progresses on the active segment.
    Charging-area bends constrain the target at their corners.
19. **Steering:** proportional signed heading correction for any angle with
    continuous speed reduction. Large heading errors use hysteretic ALIGN.
    Charging bends use stationary alignment and 0.15 m/s near-corner approach.
20. **Prediction:** 12 s horizon, 0.75 s temporal buffer; minimum prediction
    speed 0.35 m/s. Unzoned crossing protection supplements named zones.
21. **Safety distance:** v²/(2×0.65) + 0.15 processing + 0.45 footprint +
    0.55 safety margins, clamped to configured 1.5–3.0 m reservation range.
    Physical occupancy stops at 2.0 m and releases above 2.2 m.
22. **Hierarchy:** safety, physically committed zone occupant, task priority,
    charging urgency, deadline, loaded state, progress, waiting time,
    clearance time, robot ID. Physical commitment is a safety exception.
23. **Winner:** adopted decision commands PROCEED on its mission route;
    physical occupancy/LiDAR can still require a safety stop.
24. **Yield:** adopted loser begins a collision-checked lateral escape early
    when it physically blocks the winner's full remaining route. Pocket motion
    is capped at 0.22 m/s. Original route/destination remain saved; clearance
    releases the pocket only after the winner's future path clears the resume
    leg. If a shelf corner prevents direct rejoin, the robot first returns
    along the checked escape leg to its saved departure point. The controller
    receives the checked resume connector plus the untouched remaining mission
    waypoints, so its old lookahead cannot aim back at a completed source point.
25. **Reroute cost:** graph-derived extra distance / 0.50 m/s estimate versus
    predicted waiting, with peer congestion penalty and 10 m detour cap.
    Complete alternate route must clear winner route by 2.35 m.
26. **Progress:** active waypoint plus cumulative route distance; projection
    stays on the active segment so overlapping return legs cannot capture it.
27. **Rejoin:** graph candidates downstream of the blocked segment, rejecting
    backward first legs, obstructed edges, blocked-zone crossings and unsafe
    downstream merges; shortest valid route retains the original destination.
28. **Lifecycle:** local encounter latch, shared authority/version adoption,
    hysteretic clearing and retained highest version reject delayed stale
    decisions. A later legitimate encounter creates a new decision epoch.
29. **Charging:** peers publish reservations/occupancy; robot chooses available
    bay by A* cost, navigates there, docks and enters CHARGING. Battery drain
    and replenishment are explicitly simulated; pose and movement are real
    Gazebo physics telemetry.
30. **Allocation:** AUTO sends pickup/drop task to decentralized local bidders;
    availability, route distance and battery determine the shared allocation.
    MANUAL remains an operator override. Dashboard never publishes cmd_vel.
31. **Dashboard:** V2 map, live odometry, routes, charging state, peer count,
    priority and conflict events; colored current route and dashed saved route
    during reroute. EDGE AI / DECENTRALIZED labels remain emphasized.
32. **Scenarios:** priority, equal-priority, yield recovery, alternate-aisle
    reroute and fresh later encounter passed physically before friction was
    corrected. Corrected-model charging-corner PASS: 2.998 m minimum robot
    gap, evidence `/tmp/swarmx-v2-charging_gap-70d4futp.json`. Corrected-model
    fleet timeout: five deliveries, four charging, 1.837 m minimum robot gap,
    1.006 m static center gap, `/tmp/swarmx-v2-fleet-dbdyoyhb.json`. Full
    corrected-model headless replay PASS: five deliveries and five CHARGING,
    1.885 m minimum robot gap, 1.006 m static center gap, 0.415 m/s peak and
    0.301 m/s mean moving physical speed, `/tmp/swarmx-v2-fleet-hfgj_ksi.json`.
    Corrected-model priority PASS: 1.925 m minimum robot gap,
    `/tmp/swarmx-v2-priority-v77kefu1.json`. Equal-priority replay exposed a
    proximity fallback reversing a cleared shared winner while the loser was
    stationary in YIELD_POCKET. Crossing/proximity overlays now ignore that
    stationary motion intention; the independent physical-body guard remains.
    The interrupted failure is retained at `/tmp/swarmx-v2-equal-zjjmsyrb.json`.
    Latest corrected-model checks after yield/rejoin fixes all PASS:
    priority `/tmp/swarmx-v2-priority-ebkskyv9.json` (2.311 m minimum gap),
    equal priority `/tmp/swarmx-v2-equal-lvsojb46.json` (2.297 m),
    recovery `/tmp/swarmx-v2-recovery-_7kgyq0h.json` (1.964 m),
    reroute `/tmp/swarmx-v2-reroute-1y7_fs2z.json` (3.0 m, exactly one detour
    and original destination reached), and fresh later encounter
    `/tmp/swarmx-v2-later-6inhrhze.json` (2.303 m). Priority winners did not
    enter coordination hold, relocation or obstacle-stop states. The resumed
    loser retains its yielding relationship during the short mission rejoin,
    preventing a cleared-conflict proximity fallback from reversing the winner.
    First 0.50 m/s fleet attempt was interrupted and retained as FAIL:
    `/tmp/swarmx-v2-fleet-07pbqfvg.json`; one delivery, four robots held near
    charging exits, minimum gap 1.858 m. Mission routes cut diagonally from
    docked bays toward MAIN_NORTH. Both mission planning and direct routing now
    preserve the existing individual south departure entries, and the
    controller protects the departure corner from lookahead cutting. This is
    physically accepted in `/tmp/swarmx-v2-fleet-fae3bk2e.json`: five deliveries,
    all five CHARGING, minimum robot gap 2.204 m, static gap 1.004 m,
    peak physical speed 0.415 m/s, mean moving speed 0.299 m/s.
    0.50 m/s motion PASS: `/tmp/swarmx-v2-motion-3aw4dnhs.json`, straight,
    30°/45°/90° heading changes and successive opposing turns, peak 0.50 m/s.
    0.50 m/s fleet PASS: `/tmp/swarmx-v2-fleet-078203bw.json`, five deliveries
    and five CHARGING, minimum robot gap 2.295 m, static gap 1.000 m,
    peak 0.50 m/s, mean moving 0.318 m/s. Yield/resume encounters occurred.
    0.60 m/s motion PASS: `/tmp/swarmx-v2-motion-3ma3f1nz.json`, same turn
    route, peak 0.60 m/s. Full fleet PASS:
    `/tmp/swarmx-v2-fleet-_p7kfkgj.json`, five deliveries and five CHARGING,
    minimum robot gap 2.140 m, static gap 0.959 m, peak 0.60 m/s, mean moving
    0.337 m/s. Cruise cap/peak is 1.5× baseline, not total-mission speed.
    A separate 35-second five-robot live IMU sample PASSed (peak yaw rate
    0.883108 rad/s, maximum absolute roll/pitch 2.711e-9 rad); retained in
    `SWARMX_V2_IMU_GATE.json`. This is a bounded sample, not contact evidence.
    Final 0.60 m/s focused checks PASS: priority
    `/tmp/swarmx-v2-priority-u0us9hnb.json` (2.300 m), recovery
    `/tmp/swarmx-v2-recovery-w7tcbmym.json` (1.967 m), reroute
    `/tmp/swarmx-v2-reroute-s8sudc65.json` (3.0 m, one detour, original DROP_07
    reached). Priority winner log has no coordination/obstacle-stop transition.
    Final normal MANUAL inspection moves ALPHA 3; all five physical poses
    agree with RViz frames/labels (maximum sampled differences 0.03/0.12 m).
    Retained result summaries are in `SWARMX_V2_VALIDATION_EVIDENCE.json`;
    GUI integration evidence is in `SWARMX_V2_VISUAL_SYNC_GATE.json`.
33. **Limitations:** slow Gazebo rendering; validation uses sampled odometry,
    not contact-sensor collision evidence. Strict footprint/robot gap gates
    bound the sampled run. Battery and displayed progress are simulated/derived.
    GUI/RViz data agreement and speed stages PASS. Desktop Gazebo/RViz
    rendering was not independently screen-captured; GUI startup and live
    RViz process were confirmed. EGL driver warnings occurred without stack
    exit. During Ctrl+C shutdown, several installed ROS/Gazebo bridges exited
    with -11 and Gazebo required launch's SIGKILL escalation; these happened
    after interruption, not during accepted missions. A clean restart passed.
    Final launch is in Ubuntu's installed Ptyxis Terminal, not VS Code.
    The V2 demo runs its local controller, not the optional Nav2 stack;
    Nav2 YAML supplies a matching footprint but is not runtime acceptance.
34. **Launch:** Ubuntu Terminal: `cd ~/amr_ws`, source
    `/opt/ros/lyrical/setup.bash`, build both packages with `colcon build
    --packages-select amr_simulation edge_ai_nav --symlink-install`, then
    `./run_swarmx_v2_demo.sh enabled:=false use_rviz:=true`.
    Dashboard: `http://localhost:8091/fleet`. Ctrl+C stops the owned launch.
35. **Recording sequence:** keep Gazebo and dashboard visible; RViz optional.
    use AUTO warehouse transfers PICKUP_03→DROP_07, PICKUP_01→DROP_01,
    PICKUP_04→DROP_06, PICKUP_02→DROP_02, PICKUP_10→DROP_03, HIGH priority 3.
    Select AUTO, Warehouse Transfer, pickup/destination, HIGH and CREATE TASK;
    submit about three seconds apart. These are the physically tested pairs.
    Show all robots moving, live decisions, completed tasks and charging.
    For focused encounters, after stopping the stack run
    `python3 validate_v2_fleet.py priority --speed 0.6 --visible`,
    `python3 validate_v2_fleet.py equal --speed 0.4 --visible`,
    `python3 validate_v2_fleet.py reroute --speed 0.6 --visible`, or
    `python3 validate_v2_fleet.py later --speed 0.4 --visible` in fresh runs. These
    stage initial poses and task routes; actual peers decide and robots move.
    They use the same validated controller/policy, never scripted winners.
36. **Freeze before recording:** model dimensions, footprint, wheel friction,
    graph clearance, prediction horizon, occupancy guard, reroute clearance,
    charging entries, and last accepted speed/angular limits. Change any only
    with an affected physical regression. Raw /tmp logs remain temporary;
    compact evidence is retained in the workspace. Final accepted limits are
    linear 0.60 m/s and angular 0.90 rad/s, with slower dock/yield profiles.
