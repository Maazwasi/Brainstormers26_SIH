# SWARMX V2 — final validation checkpoint

## Final accepted gate — 2026-10-04

Implementation and tested demo gates: **PASS**. Final frozen launch limits:
**0.60 m/s linear, 0.90 rad/s angular**. Both packages build; **141 tests pass**.

- Corrected charging departures: five deliveries and five CHARGING at baseline,
  minimum sampled gap 2.204 m, static gap 1.004 m.
- 0.50 m/s fleet: five deliveries and five CHARGING, gaps 2.295/1.000 m.
- 0.60 m/s fleet: five deliveries and five CHARGING, gaps 2.140/0.959 m.
- Real straight/30°/45°/90°/successive-turn runs PASS at both speed stages.
- Final-speed priority/yield/resume PASS (2.300 m), close recovery PASS
  (1.967 m), one forward reroute/original destination PASS (3.0 m).
- Equal-priority and fresh-later-encounter gates PASS at baseline; peer protocol
  and arbitration were not changed for the speed stages.
- Five-robot IMU stability sample PASS. All five RViz TF/labels agree with
  physical odometry during a normal MANUAL inspection; that task completed.
- Gazebo GUI/RViz launch and live dashboard verified. Desktop rendering was
  not independently screen-captured; EGL warnings did not terminate the stack.

See `SWARMX_V2_IMPLEMENTATION_REPORT.md` for all 36 report items,
`SWARMX_V2_VALIDATION_EVIDENCE.json` for retained results, and the IMU/visual-sync
JSON gates. Safety checks were not relaxed. Contact sensors were not used;
do not present unmeasured collision counts as measured zero. Battery is simulated.
No commit or remote push was made. The following sections are historical.

## Earlier gate history — 2026-10-04

Implementation estimate: approximately 95%. Final acceptance remains PARTIAL.
The historical results below are retained as history, not the current state.

- The latest complete fleet replay finished all five deliveries but timed out
  during return to charge; only two AMRs docked. Minimum sampled robot gap was
  1.685 m; static-obstacle center gap was 0.746 m. Evidence:
  `/tmp/swarmx-v2-fleet-g08qf9p6.json`.
- Each charging bay now has its own south approach at y=32.5 m. Lookahead is
  constrained at the last docking bends, with reduced approach speed and
  stationary alignment before the perpendicular dock segment.
- Direct Gazebo command inspection showed zero linear / 0.75 rad/s angular
  command and matching wheel odometry, while physical chassis odometry still
  drifted forward. This disproved the initial command-queue hypothesis.
- The V2 four-wheel model now retains rolling friction mu=2.0 and uses lateral
  friction mu2=0.05 with wheel-local rolling direction fdir1=(1,0,0). The
  speculative shared bridge-queue change was removed.
- The exact north-cross-aisle docking-corner replay now PASSes: minimum sampled
  robot separation 2.998 m; static center gap 1.36 m; peak physical speed
  0.412 m/s. Evidence: `/tmp/swarmx-v2-charging_gap-70d4futp.json`.
- Full regression suite: **141 passed**. Both affected packages build.
- A new full five-AMR delivery/charging acceptance is running at 0.40 m/s.
  Speed increase and final visible demonstration remain pending.
- First full run on the corrected model: all five deliveries completed, four
  robots charged, last robot was advancing along its docking approach when
  the 1100-second wall-clock limit expired. This remains a FAIL/timeout,
  not fleet acceptance. Minimum robot gap 1.837 m; static center gap 1.006 m.
  Evidence: `/tmp/swarmx-v2-fleet-dbdyoyhb.json`. A headless replay uses a
  bounded 1800-second fleet timeout and the stricter 0.59 m static-clearance
  gate. The two GUI clients opened for viewing were closed before this replay.
- Focused priority, equal-priority, fresh later encounter, yielding recovery,
  and alternate-aisle rerouting passed before the physical-friction correction.
  Their logic is implemented, but physical acceptance must be checked against
  the corrected model before final demo acceptance.

No commit or remote push was made. Runtime evidence under `/tmp` is temporary.

### Corrected-model baseline acceptance

The headless full fleet replay **PASSed**: all five AUTO-assigned deliveries
completed and all five robots entered CHARGING in separate bays. Minimum
sampled robot separation **1.885 m**; minimum static-obstacle center clearance
**1.006 m**, exceeding the enlarged footprint's 0.59 m graph clearance. Peak
physical speed 0.415 m/s; mean moving speed 0.301 m/s. Evidence:
`/tmp/swarmx-v2-fleet-hfgj_ksi.json`. Safety abort gates stayed enabled.

Focused corrected-model negotiation checks now PASS: priority (2.311 m), equal
priority (2.297 m), recovery (1.964 m), reroute (3.0 m, one detour, original
destination reached), and two fresh encounters (2.303 m). Latest evidence:
`/tmp/swarmx-v2-priority-ebkskyv9.json`,
`/tmp/swarmx-v2-equal-lvsojb46.json`,
`/tmp/swarmx-v2-recovery-_7kgyq0h.json`,
`/tmp/swarmx-v2-reroute-1y7_fs2z.json`,
`/tmp/swarmx-v2-later-6inhrhze.json`.

The first 0.50 m/s full-fleet attempt was interrupted after one delivery and
four prolonged charging-exit holds; retained FAIL at
`/tmp/swarmx-v2-fleet-07pbqfvg.json` (1.858 m minimum robot gap). Docked departure
routes were diagonal toward MAIN_NORTH. The route graph now preserves each
existing straight south entry for departures, and the controller protects
that corner. Baseline physical revalidation PASSed at 0.40 m/s:
`/tmp/swarmx-v2-fleet-fae3bk2e.json`, all five deliveries and all five CHARGING,
minimum separation 2.204 m, static gap 1.004 m. The staged 0.50 m/s motion and
fleet checks are active. Speed increase remains unaccepted; final visible
verification remains.

## Historical checkpoint

Recorded 2026-10-03. Status: PARTIAL, stopped at a safe gate. The percentage is
an engineering estimate of implementation progress, not a fleet acceptance or
collision-free score. The 75% validation milestone has not passed.

## Implemented and checked

- V2 warehouse, five named AMRs, graph routing, live dashboard map/status,
  decentralized task assignment, priority decisions, and charging integration
  are present. V2 starts with `./run_swarmx_v2_demo.sh`; its dashboard is
  `http://localhost:8091/fleet`.
- Dashboard supports rectangular corridor zones as well as circular crossings.
- V2 side aisles have reservation zones and a longer prediction horizon.
- Local path/proximity holds supplement zone decisions. Close-range physical
  occupancy now overrides PROCEED as well as yield commands and covers idle,
  charging, and returning robots. It does not transfer the negotiated winner.
- Delivery pads remain at the approved coordinates. South through-traffic
  edges were removed because they crossed other delivery pads; transit uses
  the y=8 cross aisle. All 84 pickup-to-drop raw graph paths clear the other
  drop pads by at least the configured 1.18 m footprint diameter.
- V2 final approach slows to 0.20 m/s, then 0.12 m/s within 1 m, with a 0.45 m
  goal tolerance. The focused ALPHA 2 DROP_02 replay completed and started
  return-to-charge instead of repeatedly missing the terminal goal.

## Retained live evidence and its limits

The five-task run assigned and ultimately completed all five deliveries:

| Robot | Pickup | Drop | Final observed bay |
|---|---|---|---|
| ALPHA 1 | PICKUP_01 | DROP_01 | CHARGE_01 |
| ALPHA 2 | PICKUP_02 | DROP_02 | CHARGE_03 |
| ALPHA 3 | PICKUP_03 | DROP_07 | CHARGE_02 |
| ALPHA 4 | PICKUP_10 | DROP_03 | CHARGE_04 |
| ALPHA 5 | PICKUP_04 | DROP_06 | CHARGE_05 |

All five were subsequently observed CHARGING in distinct bays, with zero active
tasks and five completed tasks. Nevertheless, **this fleet run FAILED the
physical separation gate**: the retained monitor recorded ALPHA 3 / ALPHA 5 at
1.10 m, below the 1.18 m configured diameter. Earlier 1.62 m observations did
not cover the full run. Physical contact was not independently measured.

The old telemetry monitor exited on threshold failure but did not shut down
Gazebo. Therefore later task completion does not turn that run into a pass.
After inspecting the retained failure, the simulation was stopped.

The focused correction adds an independent V2 physical occupancy brake at
2.0 m, retained until separation exceeds 2.2 m. A winner cannot rely on the
other robot's yield state to assume its physical path is clear. This is a
conservative emergency hold: two blocked robots may remain waiting. A safe
waiting position or route recovery is still required for fleet acceptance.

## Validation after the occupancy correction

- `python3 -m pytest src/edge_ai_nav/test src/edge_ai_nav/tests -q`:
  **118 passed**.
- `colcon build --packages-select edge_ai_nav amr_simulation --symlink-install`:
  **both packages built successfully**.
- `git diff --check`: passed.
- `validate_v2_occupied_bay.py`: **PASS**. This deliberately routes ALPHA 5
  toward ALPHA 4's occupied bay, using actual Gazebo odometry and the local
  controller. Approach: 1.130 m; minimum sampled separation: **1.932 m**;
  stable hold: **5.0 seconds**; measured hold drift: **0.000 m**.
- Live log: `/tmp/swarmx-v2-occupied-g_v9u_ay.log` (temporary local evidence).
  The check automatically stopped its own simulation on completion.

Reproduce that short negative safety test in an Ubuntu Terminal after stopping
any existing Gazebo stack:

```bash
cd ~/amr_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 validate_v2_occupied_bay.py
```

The check monitors physical odometry, stops its own stack on a 1.50 m
preemptive threshold, timeout, stale telemetry, or completion, and does not
kill unrelated processes. PASS confirms approach and braking only.

## Remaining before 75% / fleet acceptance

1. Provide a safe exit or waiting position for the blocked side-aisle pair;
   verify automatic progress after physical occupancy clears.
2. Repeat the five-AMR delivery and charging run with continuous separation
   monitoring and automatic shutdown on a failed gate. The new brake has not
   yet passed that full replay.
3. Validate forward reroute and downstream merge safety before enabling V2
   `allow_reroute`; it remains disabled.
4. Finish the priority/tie-break demonstration, visual Gazebo/RViz verification,
   and reproducible final recording sequence.

Existing working-tree changes are preserved. No commit or remote push was made
for this checkpoint. The legacy launch remains available; V2 is not declared
ready for a collision-free demo recording.
