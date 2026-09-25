# STAGE 6A PHYSICAL MOTION RESULT

## Decision

`DO NOT CONTINUE — GAZEBO/RVIZ MOTION STILL DIVERGES`

Stage 6A fixed the original odometry/traction discrepancy, but the corrected
full physical Stage 6 autonomy replay is not yet stable enough to pass. Stage 7
must not begin.

## Root cause

1. DiffDrive published command / wheel-integrated odometry on `/amr_*/odom`.
   The observer converted it to TF and labels, so RViz could move ahead of the
   physical Gazebo model.
2. Wheel transforms were applied twice: in both the revolute joint and child
   link collision/inertial/visual pose. Direct Gazebo pose showed wheel bottoms
   about 0.018 m above the floor, so traction was nearly absent.

## Fix applied

Modified `/home/maaz-wasi/amr_ws/src/amr_simulation/models/amr_waffle.sdf.xacro`:

- Corrected wheel link-local pose offsets and drive-wheel contact friction.
- Moved diagnostic command-integrated DiffDrive odometry to
  `/amr_*/command_odom` and `/amr_*/command_tf`.
- Added Gazebo `OdometryPublisher` on `/amr_*/odom`, so ROS, TF, peer state,
  RViz labels and controllers now consume the physical model pose.

## RViz source

`five_amr_spawn_observer` consumes `/amr_*/odom`, broadcasts
`amr_*/odom -> amr_*/base_footprint`, and publishes `/fleet/spawn_labels`.
RViz shows those TF frames and labels; it does not show an independent Gazebo
RobotModel. It now follows physical-model odometry.

## Gazebo physics

- World: `warehouse_sih_demo`
- `/clock`, world stats and direct entity poses advanced: not paused.
- Observed RTF: `0.257`; slow, but not paused.

## ALPHA manual test

With the controller stopped only for the isolated manual command:

- Gazebo before: `(-2.277, -0.020)`
- Gazebo after: `(-1.870, 0.080)`
- Direct Gazebo displacement: approximately `0.42 m`

Paired physical source verification showed exact agreement:

| Source | x | y |
|---|---:|---:|
| Gazebo | 4.477343523 | 0.247777773 |
| ROS `/amr_alpha/odom` | 4.477343523 | 0.247777773 |

## Bridge

`local_waypoint_controller -> /amr_*/cmd_vel (TwistStamped) -> ros_gz_bridge
-> /amr_*/cmd_vel (gz.msgs.Twist) -> DiffDrive -> wheel joints -> Gazebo model
pose -> OdometryPublisher -> ros_gz_bridge -> /amr_*/odom`.

`/amr_alpha/cmd_vel` and `/amr_bravo/cmd_vel` each had exactly one ROS
publisher: their local controller. `peer_state` publishes no `cmd_vel`.

## Five physical tests

| Robot | Gazebo displacement | Only intended robot commanded? |
|---|---:|---|
| ALPHA | ~0.42 m | Yes |
| BRAVO | ~0.30 m | Yes |
| CHARLIE | ~0.12 m | Yes |
| DELTA | ~0.16 m | Yes |
| ECHO | ~0.27 m | Yes |

## Stage 3 physical autonomy

`FAIL / not accepted`: corrected full physical LiDAR avoidance remains to be
re-run.

## Stage 6 physical negotiation

`FAIL / not accepted`: corrected ALPHA/BRAVO replay began with direct physical
movement, but the later rigid-body path became unstable and did not complete
the required approach, yield, cross and resume sequence.

## Required next action

Stabilize the Waffle rigid-body/contact model, then re-run physical Stage 3
LiDAR avoidance and full physical Stage 6 negotiation. Do not begin Stage 7.
