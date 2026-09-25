# STAGE 6 RESULT

## Overall

`PASS`

## Stage 5 snapshot

Path: `/home/maaz-wasi/amr_ws/stage_snapshots/stage5/source.tar`  
SHA-256: `3d22a8ee3adde2c14b859049c38ec7718b7616bcd7597b76366727bbb2ba6690`

## Files created

- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/edge_ai_nav/fleet/negotiation.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/launch/stage6_negotiation.launch.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/test/test_negotiation.py`
- `/home/maaz-wasi/amr_ws/run_stage6_negotiation.sh`
- `/home/maaz-wasi/amr_ws/STAGE6_RESULT.md`

## Files modified

- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/edge_ai_nav/fleet/peer_state_node.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/edge_ai_nav/ros_nodes/local_waypoint_controller.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/launch/stage5_conflict_detection.launch.py`
- `/home/maaz-wasi/amr_ws/src/amr_simulation/config/warehouse_sih_demo.yaml`

## Negotiation architecture

Every `/amr_*/peer_state` node uses only its own state, its DDS peer table,
the local Stage 5 detector, and the same pure `choose_winner` policy. There is
no central negotiation server and no peer node publishes `cmd_vel`.

The peer node publishes only its own relative `coordination_command` topic.
The corresponding local waypoint controller consumes that local command and
remains the sole publisher of the robot's `cmd_vel`.

## Priority policy

1. Higher `task_priority` wins.
2. If priorities tie, the larger capped waiting-time bucket wins. Waiting is
   capped at 30 s with a 1 s bucket, so fleet-yield holds contribute without
   unbounded growth.
3. If still tied, the lower ETA bucket wins (0.25 s quantization).
4. If still tied, the lower lexical robot ID wins.

The result is latched by conflict ID until the observed winner has entered and
then exited the zone with the clearance margin. ETA changes cannot flip it.

## Local coordination interface

- Topic: `/amr_<robot>/coordination_command`
- Message type: `std_msgs/msg/String` containing compact JSON
- Publisher: that AMR's `peer_state` node only
- Consumer: that AMR's `local_waypoint_controller` only
- Motion authority: `local_waypoint_controller` only

Coordination states used: `NONE`, `PROCEED`, `YIELD`, `WAIT_FOR_CLEAR`,
`SAFE_WAIT`, and `RESUME`.

## Safety ordering

The controller evaluates: sensor-stale stop, LiDAR immediate obstacle stop,
coordination hold, then normal waypoint navigation. `PROCEED` never bypasses
LiDAR. The pure test explicitly verifies an unsafe front scan yields
`LIDAR_STOP` before a coordination hold.

## Golden Intersection A result

The Stage 6 scenario starts ALPHA at `(-2.5, 0)` and BRAVO at `(0, -2.5)`,
both crossing Intersection A.

Independent recorded results:

| Local AMR | Conflict ID | Winner | Loser | Reason | Latency |
|---|---|---|---|---|---:|
| ALPHA | `intersection_A:ALPHA:BRAVO` | BRAVO | ALPHA | `TASK_PRIORITY` | 0.000 ms |
| BRAVO | `intersection_A:ALPHA:BRAVO` | BRAVO | ALPHA | `TASK_PRIORITY` | 0.000 ms |

The timestamps are both from each peer node's ROS system clock; the detector's
first-detect timestamp and decision timestamp are taken in the same callback,
so no system/simulation clocks are mixed. The pair latency is the maximum local
latency: **0.000 ms** at the logged clock resolution.

ALPHA entered `WAIT_FOR_CLEAR`, held at approximately world x = `-1.40 m`, and
BRAVO received `PROCEED`. Intersection A has radius `1.05 m`; the configured
pre-zone hold margin is `0.40 m`, so the hold was outside the `1.45 m` boundary
(up to controller tolerance). BRAVO crossed, then ALPHA received zone
clearance and completed its route.

## Peer-offline safety result

During the active negotiated conflict, only BRAVO's peer-state process was
paused for three seconds. ALPHA changed to `SAFE_WAIT`; its local status was
`COORDINATION_HOLD` at x ≈ `-1.40 m`. After BRAVO's peer resumed and BRAVO
cleared the exit margin, ALPHA resumed and reached `MISSION_COMPLETE`.

## Collision and publisher checks

No robot-robot collision was observed during the complete replay. The yielding
robot's approximately 1.40 m lateral hold before BRAVO crossed provides a
conservative centre-to-centre separation at BRAVO's zone centre; AMR footprint
diameter is 0.36 m.

Live ROS topic inspection showed exactly one publisher on each tested motion
topic:

- `/amr_alpha/cmd_vel`: `local_controller`
- `/amr_bravo/cmd_vel`: `local_controller`

## Focused verification

- Pure deterministic/latch/safety tests: 4 passed.
- `colcon build --packages-select edge_ai_nav --symlink-install`: passed.
- Visible Gazebo/RViz Stage 6 replay: passed.
- Five peer nodes, five local controllers, and five AMRs were launched.

## Scope boundary

Stage 7 deadlock/livelock behavior was not implemented.

SAFE TO PROCEED TO STAGE 7 — DEADLOCK AND LIVELOCK RESOLUTION
