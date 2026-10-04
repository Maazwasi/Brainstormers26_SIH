# SWARMX V2 — Phase 0 baseline

Audit date: 2026-10-03 (Asia/Kolkata).
Base commit: `79c7029333ad462fd534be56f6ee40227ea9dd65`.
Existing uncommitted implementation changes were preserved. No production code was changed during this audit.

## Implementation map

Paths below are relative to this workspace.

| Responsibility | Current implementation |
|---|---|
| Active world, shelves, static collision and visuals | `src/amr_simulation/worlds/warehouse_sih_demo.sdf` |
| Bounds, stations, waypoint graph, obstacles, spawn poses, names, colours, policy configuration | `src/amr_simulation/config/warehouse_sih_demo.yaml` |
| Active robot visual/collision geometry, wheels, sensors, inertia, DiffDrive limits | `src/amr_simulation/models/warehouse_amr.sdf.xacro` (SDF, not URDF) |
| Spawn names, namespaces, bridge substitutions | `src/edge_ai_nav/launch/five_amr_demo.launch.py::robot_actions`; `config/five_amr_bridge.yaml` |
| Active fleet composition | `src/edge_ai_nav/launch/stage6_negotiation.launch.py` includes `stage5_conflict_detection.launch.py::create` |
| Automatic decentralized bids/claims/assignment | `src/edge_ai_nav/edge_ai_nav/fleet/task_bidder_node.py::TaskBidder` callbacks `on_task`, `on_bid`, `resolve` |
| A*, mission pickup/destination planning, smoothing, alternatives | `src/edge_ai_nav/edge_ai_nav/fleet/route_graph.py::WarehouseGraph` methods `astar`, `mission_plan`, `route_to_with_cost`, `reroute` |
| Route installation/progress, lookahead, heading, velocity, battery, charging, physical yield | `src/edge_ai_nav/edge_ai_nav/ros_nodes/local_waypoint_controller.py::LocalController` |
| Live conflict prediction | `src/edge_ai_nav/edge_ai_nav/fleet/zone_prediction.py::LocalDetector`, `window`; graph `intent` |
| DDS state, cost estimates, authority, encounter adoption and release | `src/edge_ai_nav/edge_ai_nav/fleet/peer_state_node.py::PeerState`, `conflict_costs`, `negotiate` |
| Arbitration, command mapping, encounter records, stopping reservation | `src/edge_ai_nav/edge_ai_nav/fleet/negotiation.py::resolve_conflict`, `coordination_action`, `NegotiationBook`, `reservation_distance` |
| Dashboard registry, tasks, status, routes, conflict events | `src/edge_ai_nav/edge_ai_nav/visualization/fleet_dashboard_node.py` (`ROBOTS`, `create_task`, `on_status`, `on_route`, `on_edge_event`) |
| Dashboard display | `src/edge_ai_nav/edge_ai_nav/visualization/static/fleet_dashboard.{html,js,css}` |

The separate `fleet/conflict_detector.py` is legacy logic; the active Stage 6 peer node imports `zone_prediction.LocalDetector`.

## Current values

| Item | Baseline |
|---|---|
| Warehouse | 24 × 18 m, x = -12…12, y = -9…9 |
| Chassis | 0.60 × 0.45 × 0.14 m; centre z = 0.150 m |
| Overall visible model extent | Approximately 0.60 × 0.51 × 0.25 m, including wheels and lidar |
| Wheel radius / track / axle spacing | 0.085 / 0.47 / 0.36 m |
| Configured footprint diameter | 0.36 m — inconsistent with the actual body |
| Graph obstacle clearance | 0.36 m |
| Controller max linear / angular | 0.50 m/s / 0.50 rad/s |
| DiffDrive max linear / angular | 0.40 m/s / 1.20 rad/s |
| Controller acceleration | 0.65 m/s² for increasing forward command; reductions immediate at controller output |
| DiffDrive linear acceleration/deceleration | ±0.80 m/s² |
| DiffDrive angular acceleration | ±1.50 rad/s² |
| Normal / turn lookahead | 0.55 / 0.35 m (turn greater than 45°); performance factor applies |
| Steering | atan2 heading error, gain 1.25 × performance factor; 0.05 rad deadband |
| Alignment entry / exit | 0.55 / 0.20 rad; zero forward command above 0.85 rad |
| Obstacle prepare / stop / clear | 1.25 / 0.85 / 1.10 m |
| Prediction | 6 s horizon, 0.75 s overlap buffer, 0.35 m/s configured minimum prediction speed |
| Negotiation distance | Dynamic reservation clamped to 1.5…2.0 m; no single universal conflict distance |
| Braking calculation | 0.65 m/s² safe deceleration, 0.15 s processing margin, 0.45 m footprint margin, 0.55 m safety margin |
| Hold / exit margin | YAML 0.40 m each; peer node enforces minimum 0.55 m each |
| Maximum alternate detour | 2.5 m |
| Battery | Simulated; low 30%, critical 15%; drain 0.08 percentage points/m; charging 0.25 percentage points/s |

Current arbitration order: SAFETY → ACTIVE_RESERVATION → ZONE_COMMITTED → TASK_PRIORITY → EXECUTING_OVER_IDLE → WAITING_TIME → ETA → ROBOT_ID. This differs from the requested V2 hierarchy.

Reroute selection uses a graph alternate excluding the conflict zone, rejects excessive additional distance, estimates extra distance / 0.50 m/s and adds an 8 s congestion penalty when applicable. The loser reroutes if a route exists and its cost is below the wait estimate. Current rerouting does not explicitly select downstream rejoin targets on the original mission. Current lookahead projects on the active segment and retains monotonic target progress, but route replacement resets progress.

## Validation and critical gate

Executed:

1. `python3 -m pytest src/edge_ai_nav/test -q`: 43 passed.
2. `python3 -m pytest src/edge_ai_nav/tests src/edge_ai_nav/test -q`: 58 passed.
3. `colcon build --packages-select amr_simulation edge_ai_nav --symlink-install`: both packages built successfully.
4. Focused invocation of `LocalController.lookahead_target` with a valid one-point route and a robot outside goal tolerance: FAIL, `IndexError` at `self.route[first+1]` (line 327).

Root cause: task assignment accepts one-point routes, but the lookahead routine unconditionally assumes two points. Terminal completion avoids that routine only when the robot is already within goal tolerance. A one-point staging target outside tolerance can therefore terminate the controller. This is consistent with the previous BRAVO/CHARLIE staging failures, but their retained launch logs contain exit code 1 without a traceback, so historical causality is not proven.

Existing tests passing does not establish physical stability or successful rerouting. No new live simulation was launched during this audit. The previous Stage 26 result remains unvalidated.

Phase 0 status: AUDIT COMPLETE; CRITICAL BASELINE GATE FAIL. Phases 1–23 have not started. Resolve the controller route-cardinality failure before advancing. The authoritative warehouse drawing referenced in the master prompt is also absent from the supplied attachment; its shelf access-side details cannot be visually verified from the text alone.

Files changed during Phase 0: this report only. Build outputs were refreshed. No baseline defect was hidden, and no previously uncommitted work was discarded.
