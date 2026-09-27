"""Deterministic Stage 6 negotiation and local coordination helpers."""
from dataclasses import dataclass, asdict
import math
import time


@dataclass(frozen=True)
class Decision:
    winner: str
    loser: str
    reason: str
    details: dict
    loser_action: str = 'YIELD_AND_WAIT'

    def dictionary(self):
        return asdict(self)


def _active(state):
    return state.get('local_nav_state') not in (
        'DISABLED', 'MISSION_COMPLETE', 'PARKED', 'AVAILABLE', 'UNKNOWN')


def resolve_conflict(a, b, zone, route_costs=None, etas=None, waiting_cap=30.0,
                     waiting_quantum=1.0, eta_quantum=0.25):
    """Resolve a peer conflict with one deterministic, shared hierarchy.

    ``route_costs`` is keyed by robot ID and contains measured/graph-derived
    ``wait`` and ``reroute`` seconds plus an optional alternate route.  It is
    used only after safety, reservation, priority, activity and commitment.
    """
    states = sorted((a, b), key=lambda state: state['robot_id'])
    first, second = states
    etas = etas or {s['robot_id']: float(s.get('eta') or 1e6) for s in states}
    route_costs = route_costs or {}
    waits = {s['robot_id']: min(waiting_cap, max(0.0, float(s.get('waiting_time', 0.0))))
             for s in states}
    values = {
        'safety': {s['robot_id']: bool(s.get('safety_emergency')) for s in states},
        'reserved': {s['robot_id']: bool(s.get('zone_reserved')) for s in states},
        'priority': {s['robot_id']: int(s.get('task_priority', 0)) if _active(s) else 0 for s in states},
        'active': {s['robot_id']: _active(s) for s in states},
        'committed': {s['robot_id']: bool(s.get('zone_committed')) for s in states},
        'eta': {s['robot_id']: round(float(etas.get(s['robot_id'], 1e6))/eta_quantum) for s in states},
        'waiting': {rid: math.floor(value/waiting_quantum) for rid, value in waits.items()},
    }
    # A robot already committed owns the zone even when a later urgent task
    # arrives.  A safety-stopped robot cannot be instructed to proceed.
    hierarchy = (
        ('SAFETY', lambda s: not values['safety'][s['robot_id']]),
        ('ACTIVE_RESERVATION', lambda s: values['reserved'][s['robot_id']]),
        ('ZONE_COMMITTED', lambda s: values['committed'][s['robot_id']]),
        ('TASK_PRIORITY', lambda s: values['priority'][s['robot_id']]),
        ('EXECUTING_OVER_IDLE', lambda s: values['active'][s['robot_id']]),
        ('WAITING_TIME', lambda s: values['waiting'][s['robot_id']]),
        ('ETA', lambda s: -values['eta'][s['robot_id']]),
    )
    winner = None; reason = 'ROBOT_ID'
    candidates = states
    for label, key in hierarchy:
        scores = [key(s) for s in candidates]
        if scores[0] != scores[1]:
            winner = candidates[scores.index(max(scores))]; reason = label; break
    if winner is None:
        winner = first
    loser = second if winner is first else first
    costs = route_costs.get(loser['robot_id'], {})
    wait_cost = float(costs.get('wait', 0.0))
    reroute_cost = float(costs.get('reroute', 1e9))
    action = ('MOVE_ASIDE' if _active(winner) and not _active(loser) else
              'REROUTE' if costs.get('route') and reroute_cost < wait_cost else
              'YIELD_AND_WAIT')
    details = dict(values, zone=zone, wait_cost=wait_cost,
                   reroute_cost=reroute_cost,
                   reroute_route=costs.get('route'),
                   extra_detour_m=costs.get('extra_detour_m'),
                   decision_basis=reason)
    return Decision(winner['robot_id'], loser['robot_id'], reason, details, action)


def choose_winner(a, b, etas, waiting_cap=30.0, waiting_quantum=1.0,
                  eta_quantum=0.25):
    """Compatibility wrapper using the canonical hierarchy."""
    return resolve_conflict(a, b, '', etas=etas, waiting_cap=waiting_cap,
                            waiting_quantum=waiting_quantum, eta_quantum=eta_quantum)


class NegotiationBook:
    """Small local latch table.  It deliberately contains no ROS state."""
    def __init__(self):
        self._decisions = {}

    def decide(self, conflict_id, a, b, etas, now, first_detected,
               zone='', route_costs=None, detection_monotonic_ns=None, **policy):
        if conflict_id not in self._decisions:
            decision = resolve_conflict(a, b, zone, route_costs, etas, **policy)
            committed_ns=time.perf_counter_ns()
            detected_ns=int(detection_monotonic_ns or committed_ns)
            self._decisions[conflict_id] = {
                **decision.dictionary(), 'conflict_id': conflict_id,
                'decision_time': now, 'first_detected_time': first_detected,
                'detection_monotonic_ns': detected_ns,
                'decision_monotonic_ns': committed_ns,
                'decision_latency_ms': max(0.0, (committed_ns-detected_ns)/1e6),
                'winner_entered_zone': False,
            }
        return self._decisions[conflict_id]

    def get(self, conflict_id):
        return self._decisions.get(conflict_id)

    def clear(self, conflict_id):
        return self._decisions.pop(conflict_id, None)

    def values(self):
        return self._decisions.values()


def outside_with_margin(state, zone, margin):
    cx, cy = zone['center']
    if 'radius' in zone:
        return math.hypot(state['x']-cx, state['y']-cy) >= zone['radius']+margin
    bounds = zone['bounds']
    return (state['x'] < bounds['x'][0]-margin or state['x'] > bounds['x'][1]+margin or
            state['y'] < bounds['y'][0]-margin or state['y'] > bounds['y'][1]+margin)


def motion_gate(front, stop_distance, coordination_hold):
    """The controller uses this ordering: physical safety before fleet hold."""
    if front < stop_distance:
        return 'LIDAR_STOP'
    if coordination_hold:
        return 'COORDINATION_HOLD'
    return 'NAVIGATE'


def reservation_distance(speed, safe_deceleration=0.65, processing_margin=0.15,
                         footprint_margin=0.45, safety_margin=0.55,
                         minimum_distance=1.5, maximum_distance=2.0):
    """Speed-aware pre-zone hold distance; never reduces the safety minimum."""
    stopping=max(0.0,float(speed))**2/(2.0*max(0.05,float(safe_deceleration)))
    required=stopping+processing_margin+footprint_margin+safety_margin
    return min(maximum_distance,max(minimum_distance,required))


def bid_eligible(controller_state, has_pending_assignment, pose_fresh):
    """Parking/repositioning is interruptible; an assigned task is not."""
    return (controller_state in ('DISABLED','MISSION_COMPLETE',
            'POST_TASK_REPOSITION','RETURNING_TO_CHARGE','GOING_TO_CHARGE',
            'CHARGING','PARKED','AVAILABLE','MOVE_ASIDE')
            and not has_pending_assignment and pose_fresh)
