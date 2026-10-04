# SWARMX final acceptance audit — 2026-10-04

## Decision

**20 / 21 gates accepted (95%).** The 1.80 m/s five-AMR end-to-end fleet gate
remains **UNACCEPTED**. No further run was started after the operator's
15-minute cutoff.

## Latest final-run evidence

Evidence file: `SWARMX_LARGE_20261004_FLEET_180_FINAL_ACCEPTANCE9.json`

- Validator status: **FAIL**
- Stop reason: `Persistent waypoint alignment/dancing: alpha_2`
- Deliveries completed before stop: **4 / 5**
- Maximum concurrent outstanding tasks: **5 / 5**
- Peak physical speed: **1.795 m/s**
- Peak per-AMR speeds: 1.790, 1.733, 1.728, 1.795, 1.768 m/s
- Maximum physical lateral speed: **0.1630 m/s** (gate: 0.200 m/s)
- Minimum sampled robot separation: **3.287 m** (gate: 3.180 m)
- Minimum sampled static gap: **1.959 m**
- Maximum tilt: **1.79e-9 rad**
- Maximum angular speed: **0.461 rad/s** (gate: 0.750 rad/s)
- Yield hold oscillation cycles: **none**
- Every recorded route-version/waypoint alignment entry occurred once; none
  exceeded the per-corner repetition limit.

## Remaining blocker

The latest stop is an observer bookkeeping defect, not a measured safety-gate
failure. `align_episodes` is cumulative across delivery, reroutes, and automatic
charging-return routes, but the observer compares it to only the length of the
currently active route. ALPHA 2 completed its delivery and was on a fresh
two-point charging route when this cumulative comparison fired. The retained
per-route-version/waypoint evidence shows one alignment per recorded corner,
not repeated corner dancing.

This interpretation is evidence-backed, but the final gate is still not marked
PASS because the required clean rerun was not completed before the cutoff.

## Corrections completed during final validation

- Retained 1.80 m/s straight cruise while limiting active steering to 0.55 m/s.
- Enforced exclusive yield-pocket relocation and extra pocket clearance.
- Preserved the certified 3.18 m centre-separation boundary.
- Canonicalized simultaneous adjacent-zone decisions per robot pair.
- Cleared stale loser pocket plans when the canonical decision makes that robot
  the winner.
- Reordered the same five AUTO tasks so the longest transfer clears before the
  charging-return wave.
- Required and measured five simultaneous outstanding tasks.

Focused affected tests: **38 / 38 PASS**.

## Demo-safe recommendation

For today's video, use the already accepted 0.60 m/s fleet profile for the full
five-AMR end-to-end delivery/charging sequence. The 1.80 m/s profile has strong
physical evidence and four completed deliveries in the latest run, but must not
be described as fully fleet-accepted until the observer counter is corrected
and one clean five-AMR rerun passes.

## Exact next action

Replace the cumulative alignment-vs-current-route check in
`validate_v2_large.py` with a per-route-version/per-waypoint check (the observer
already records `corner_align_cycles`), then run exactly one fleet acceptance
command. Do not replay the earlier 20 accepted gates.
