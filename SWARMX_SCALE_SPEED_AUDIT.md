# SWARMX 3× size / 4× speed — pre-change audit

Recorded 2026-10-04 BEFORE runtime/model/configuration changes. Clean baseline
commit: `2db4adf`. Existing V2 remains the rollback/demo baseline.

## Current validated robot

- Chassis visual and collision: 0.90 × 0.70 × 0.20 m, centre z=0.20 m.
- Full XY collision bounds: x ±0.45 m, y ±0.38 m (including wheels).
- Overall physical collision dimensions, not just chassis: 0.90 × 0.76 ×
  0.30 m (z=0 to .30). Overall visible geometry: 0.90 × 0.76 × 0.35 m,
  including the LiDAR cylinder. These scale to 2.70 × 2.28 × 0.90 m collision
  and 2.70 × 2.28 × 1.05 m visible geometry.
- Wheels: radius 0.11 m; width 0.06 m; separation 0.70 m.
- Wheel centres: (±0.30, ±0.35, 0.11) m; four driven wheels, no caster.
- LiDAR: (0.12,0,0.33) m; visual radius 0.07 m, height 0.04 m.
  GPU scan 5 Hz, 360 samples, 20 m range. IMU: (0,0,0.25) m, 50 Hz.
- base_footprint and base_link coincide; chassis CoM z=0.19 m.
- Navigation polygon: (-.45,-.35),(-.41,-.38),(.41,-.38),(.45,-.35),
  (.45,.35),(.41,.38),(-.41,.38),(-.45,.35).
- Circular fallback diameter 1.18 m, radius 0.59 m; graph clearance 0.59 m.
- Chassis mass 35 kg; each wheel 1.5 kg. Friction mu=2, lateral mu2=.05.

## Current motion and safety

- Validated launch maximum forward speed 0.60 m/s; clear-straight target up to
  that cap with learned performance factor and heading slowdown. Fleet measured
  mean moving speed 0.337 m/s, not a fixed normal cruise.
- Controller angular cap 0.90 rad/s; physical plugin cap 1.20 rad/s.
- Controller linear acceleration cap 0.65 m/s²; safety requests zero immediately.
  Physical acceleration/deceleration ±0.80 m/s², angular ±1.50 rad/s².
- Actual local-control timer 10 Hz; wheel/physical odometry 20 Hz.
- Lookahead base 0.75 m; adaptive cap 1.20 m; sharp-turn target 0.55 m.
  Charging corners stop lookahead at the corner. Goal/waypoint tolerances 0.45 m;
  charging-corner arrival 0.25 m, docking slot tolerance 0.22 m.
- LiDAR stop .85 m, preparation 1.25 m, release 1.10 m (sensor-origin ranges).
- Peer physical brake 2.00 m centre distance, releases at 2.20 m.
  This includes a 0.82 m comfort buffer beyond the 1.18 m pair diameter;
  the enlarged profile must retain it, not substitute a smaller margin.
- Reservation: v²/(2×.65) + .15 processing + .45 footprint + .55 safety,
  clamped to 1.50–3.00 m. At .60 m/s the unclamped estimate is 1.427 m.
- Prediction horizon 12 s, temporal buffer .75 s, minimum prediction speed
  .35 m/s. Unzoned path guard looks 8 m ahead with 10 s ETA slack; not one
  fixed universal detection radius. Reroute route-to-route clearance 2.35 m.
- Optional Nav2 inflation .70 m; this demo uses the local controller, not Nav2.

## Requested exact targets and initial feasibility

- Uniform linear scale 3.0: chassis 2.70 × 2.10 × .60 m; full collision
  bounds x ±1.35, y ±1.14 m; wheels radius .33, width .18, separation 2.10 m.
  Full turning-envelope radius sqrt(1.35²+1.14²)=1.76695 m.
- Target straight cap .60×4=2.40 m/s; NOT 3.00 m/s (+400%).
- Existing inner shelf aisles 2.70 m: aligned width 2.28 fits physically with
  only .42 m total spare, but a 3.534 m turning envelope fails by .834 m.
  With a separate .15 m navigation margin each side, required width is
  3.834 m: short by 1.134 m. No safety-margin reduction is permitted.
- Existing pickup centres are 1.35 m south of shelf faces: short of the
  proposed 1.92 m centre clearance by .57 m.
- Existing chargers are spaced 3.0 m: aligned bodies do not overlap, but the
  turning/navigation envelope cannot pass neighbouring bays. Bay centres at
  y=36 are only 1.36 m from the backstop: short by .56 m.
- Corner pallets currently obstruct enlarged outer-aisle passages.
- At 2.40 m/s, braking at .80 m/s² alone needs 3.60 m; conservatively using
  .65 m/s² requires 4.431 m, before reaction, negotiation and body margins.
  The existing 3 m reservation clamp is insufficient.

## Planned geometry-only feasibility correction (not yet accepted)

Preserve 50×40 m, twelve unchanged-size shelves in three rows/four columns,
central main corridor, twelve pickups, seven drops and five top chargers.
Widen inner/outer aisles to 4.20 m by moving shelf columns to x=7.225,
17.225,32.775,42.775. Central free gap remains 9.75 m. Move pickup approaches
farther south to satisfy the real footprint; expand top charger pitch and move
bay centres south of the backstop; move corner pallets out of through-aisles.
Every change must be reflected in the physical world and route graph.

No upgraded speed, geometry or launch is accepted until its gates pass.
