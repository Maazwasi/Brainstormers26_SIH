"""Deterministic Stage 6 negotiation and local coordination helpers."""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class Decision:
    winner: str
    loser: str
    reason: str
    details: dict

    def dictionary(self):
        return asdict(self)


def choose_winner(a, b, etas, waiting_cap=30.0, waiting_quantum=1.0,
                  eta_quantum=0.25):
    """Return an invocation-order-independent winner for two published states."""
    states = sorted((a, b), key=lambda state: state['robot_id'])
    first, second = states
    p1, p2 = int(first['task_priority']), int(second['task_priority'])
    if p1 != p2:
        winner = first if p1 > p2 else second
        reason = 'TASK_PRIORITY'
        details = {'priorities': {s['robot_id']: int(s['task_priority']) for s in states}}
    else:
        waits = {s['robot_id']: min(waiting_cap, max(0.0, float(s.get('waiting_time', 0.0))))
                 for s in states}
        wait_buckets = {rid: math.floor(value/waiting_quantum) for rid, value in waits.items()}
        if len(set(wait_buckets.values())) > 1:
            # The second tuple element keeps equal buckets lexical and therefore
            # independent of dict insertion order.
            rid = sorted(wait_buckets, key=lambda key: (-wait_buckets[key], key))[0]
            winner = next(s for s in states if s['robot_id'] == rid)
            reason = 'WAITING_TIME'
            details = {'waiting_seconds_capped': waits, 'waiting_buckets': wait_buckets}
        else:
            eta_buckets = {s['robot_id']: round(float(etas[s['robot_id']])/eta_quantum)
                           for s in states}
            if len(set(eta_buckets.values())) > 1:
                rid = min(eta_buckets, key=lambda key: (eta_buckets[key], key))
                winner = next(s for s in states if s['robot_id'] == rid)
                reason = 'ETA'
                details = {'eta_seconds': dict(etas), 'eta_buckets': eta_buckets}
            else:
                winner = first
                reason = 'ROBOT_ID'
                details = {'lexical_rule': 'lower robot ID wins'}
    loser = second if winner is first else first
    return Decision(winner['robot_id'], loser['robot_id'], reason, details)


class NegotiationBook:
    """Small local latch table.  It deliberately contains no ROS state."""
    def __init__(self):
        self._decisions = {}

    def decide(self, conflict_id, a, b, etas, now, first_detected, **policy):
        if conflict_id not in self._decisions:
            decision = choose_winner(a, b, etas, **policy)
            self._decisions[conflict_id] = {
                **decision.dictionary(), 'conflict_id': conflict_id,
                'decision_time': now, 'first_detected_time': first_detected,
                'decision_latency_ms': max(0.0, (now-first_detected)*1000.0),
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
