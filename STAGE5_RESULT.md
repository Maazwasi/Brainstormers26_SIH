# STAGE 5 RESULT

## Overall

PASS for the configured detection-only MVP scenarios. No Stage 6 behavior was added.

## Stage 4 snapshot

`/home/maaz-wasi/amr_ws/stage_snapshots/stage4/source.tar`

SHA256: `44e5c59207e722857917ebc470ddf0e09f8c66130243b5fa1543439e2667e2b7`

Source and Stage 2–4 launchers; excludes build/install/log/.git and Python caches.

## Files created

- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/edge_ai_nav/fleet/zone_prediction.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/launch/stage5_conflict_detection.launch.py`
- `/home/maaz-wasi/amr_ws/run_stage5_conflict_detection.sh`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/test/test_zone_prediction.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/test/stage5_probe.py`
- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/test/stage5_crossing_check.py`
- `/home/maaz-wasi/amr_ws/STAGE5_RESULT.md`
- Runtime captures under `/home/maaz-wasi/amr_ws/stage_snapshots/stage5_evidence/`.

## Files modified

- `/home/maaz-wasi/amr_ws/src/edge_ai_nav/edge_ai_nav/fleet/peer_state_node.py`
- `/home/maaz-wasi/amr_ws/src/amr_simulation/config/warehouse_sih_demo.yaml`

## Conflict architecture

Every existing peer node owns a separate LocalDetector and conflict dictionary.
Inputs are its own odometry/status, YAML mission intent, and its DDS-received
peer table. Each publishes its own conflict_status. Detection is opt-in in the
Stage 5 launch; Stage 4 launch does not enable it. No central detector exists.
The test supervisor enables/stops preconfigured test missions and fault-injects
a peer-process pause; it computes no conflict decision and is not in the launch.

An older unrelated conflict_detector.py already contained negotiation rules.
It was preserved, and is neither imported nor launched by Stage 5.

## Configuration

- Prediction horizon: 6.0 seconds to entry.
- Time-overlap buffer: 0.75 seconds total.
- Nominal minimum prediction speed: 0.25 m/s.
- Zone source: canonical warehouse_sih_demo.yaml.
- ETA = forward distance to zone boundary / max(abs(local linear speed), 0.25).
- Ray/circle or ray/rectangle intersection computes entry and exit distances.
  Occupancy window is [entry_distance/speed, exit_distance/speed].
- Buffered overlap = max(0, min(exits) - max(entries) + 0.75).
- Moving heading must approach the zone; intent ray must intersect it.
- Disabled/completed/stale missions withdraw intent. Stopped active missions
  still use nominal speed. Freshness uses monotonic receive age, not a latency claim.

## Conflict zones loaded

intersection_A (center 0,0; radius 1.05 m), intersection_B (5,0; radius 1.10 m),
narrow_aisle_1 (Y-aligned rectangle from configured width/length), obstacle_area
(alias of obstacle_demo_area bounds). No geometry is hard-coded in the detector.

## Conflict ID scheme

`<zone>:<alphabetically first robot>:<alphabetically second robot>`

The first-detected timestamp is retained across repeated packets. A continuing
prediction counts once; a new encounter after clearing counts again.

## ALPHA/BRAVO test

ALPHA starts west at (-2.5,0), BRAVO south at (0,-2.5), with perpendicular onward
intent toward Intersection A. Physical missions stop at (-1.5,0)/(0,-1.5).
Neither robot needs to enter the unsafe shared region to demonstrate prediction.

| Local detector | Own boundary distance | Own ETA | Peer ETA | Buffered overlap |
| --- | ---: | ---: | ---: | ---: |
| ALPHA | 1.3915 m | 5.566 s | 5.580 s | 9.136 s |
| BRAVO | 1.3950 m | 5.580 s | 5.616 s | 9.114 s |

Both independently detected `intersection_A:ALPHA:BRAVO`; both local controller
statuses commanded 0.25 m/s and showed live LiDAR sectors. Each event_count was 1.
The relatively long overlap reflects a full 2.10 m zone traversal at 0.25 m/s,
plus the buffer; it is not negotiation latency or imminent impact time.

## Same-zone non-conflict test

PASS in Gazebo: both advertised intersection_A while moving. At the final
capture ALPHA was (-1.7657,0), BRAVO approximately (0,-4.2634): predicted entry
ETAs 2.863 s and 12.854 s. BRAVO was outside the 6 s horizon. All five detectors
reported zero events. A separate pure test checks disjoint windows within horizon.

## Conflict clearing test

PASS: a separate repeated approach was detected; disabling BRAVO through its
existing local service withdrew intent. Both logged CONFLICT_CLEARED and zero
active conflicts. No detector issued a motion command.

## Offline peer invalidation

PASS: SIGSTOP only on BRAVO's peer process caused ALPHA to log
CONFLICT_INVALIDATED_PEER_OFFLINE when receive age exceeded 1 s. The process was
resumed with SIGCONT; no robot controller or Gazebo process was stopped by this
fault injection. A resumed stale detector also invalidated its outdated table.
Pure tests cover prediction removal and later lifecycle reconstruction.

## False positives

Zero across all five detectors during the recorded 15 s moving safe-mission
capture. CHARLIE/DELTA/ECHO also reported zero during the deliberate crossing.
These are scoped observations, not a claim of zero false positives on all routes.

## Node responsibility audit

PASS: runtime publisher lists for all five peer nodes contain only local
conflict_status, peer_diagnostics, /fleet/peer_state and ROS housekeeping topics.
None publishes cmd_vel. Task priority is transmitted but not used by prediction.

## Regression

Stage 3: unchanged local controller; live LiDAR and autonomous waypoint progress
observed. BRAVO/CHARLIE/DELTA/ECHO reached their safe-mission goals; ALPHA reached
two waypoints before the run was stopped for crossing tests. This turn did not
repeat the entire crate-avoidance mission to completion.

Stage 4: five publishers and five peer subscribers observed in each valid run;
state changes were sensor-derived. Receive-age handling and per-robot tables remain.

## Tests

PASS: 10 deterministic tests (overlap, ID symmetry, stopped ETA, horizon, missed
geometry, moving away, lifecycle/dedup/timestamp/clearing/loss, timing separation,
disjoint within-horizon windows, disabled intent); affected package build; Python
syntax; safe runtime; bilateral crossing; clearing; peer-loss; different-arrival
runtime; five-node responsibility audit.

## RTF

Stage 2 reference ≈0.84; Stage 4 previously reported clean sample ≈1.00.
Stage 5 measured ROS simulation-clock advancement / monotonic wall time:
0.319 during the safe moving capture, 0.405 during different-arrival capture,
0.387 in the final short audit. Gazebo and passive visualization were running.
These are descriptive samples, not performance benchmarks.

## Errors and fixes

- Preserved unrelated legacy winner-selection code; isolated the new helper.
- Added per-waypoint zone intent and withdrew it on disabled/completed/stale state.
- Added a Stage 5 launch lock to prevent duplicate Stage 5 launch trees.
- One timing-test Gazebo startup never supplied a world/sensors. Excluded its
  empty capture; clean restart passed. Probe now requires all five packets first.

## Known issues

- Straight-line, constant-speed occupancy prediction; no full path planner,
  footprint sweep, localization-uncertainty model or arbitrary-route guarantee.
- Crossing holds intentionally advertise onward intersection intent while the
  physical test stops short. Completed/disabled routes withdraw that intent.
- Physical stationary obstacles still belong to local LiDAR handling; this is
  an intent-based traffic predictor, not a replacement for collision avoidance.
- RTF is below real time on this visible run. No optimization claim is made.
- Optional conflict RViz labels were not added. Use local conflict_status/logs.

## Demo

Use Ubuntu Terminal, one launch at a time:

```bash
cd /home/maaz-wasi/amr_ws
./run_stage5_conflict_detection.sh scenario:=crossing enabled:=true
```

Alternatives: `scenario:=safe` or `scenario:=different_time`. Stop the current
launch with Ctrl+C before switching. The crossing has built-in safe hold goals.

## Decision

SAFE TO PROCEED TO STAGE 6 — DECENTRALIZED NEGOTIATION AND RIGHT-OF-WAY
