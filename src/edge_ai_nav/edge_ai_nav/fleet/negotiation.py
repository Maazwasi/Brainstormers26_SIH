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


DECISION_ACTIONS = frozenset(('YIELD_AND_WAIT', 'REROUTE', 'MOVE_ASIDE'))


def conflict_identity(conflict_id):
    """Return the canonical zone, participants and encounter authority."""
    parts = str(conflict_id).split(':')
    if len(parts) != 3 or not all(parts):
        raise ValueError('Malformed conflict ID')
    zone, left, right = parts
    participants = tuple(sorted((left, right)))
    if left == right or (left, right) != participants:
        raise ValueError('Non-canonical conflict ID')
    return zone, participants, participants[0]


def coordination_action(decision, robot_id, already_holding=False):
    """Map one adopted decision to exactly one participant's command."""
    if decision is None:
        return 'SAFE_WAIT'
    if robot_id == decision['winner']:
        return 'PROCEED'
    if robot_id != decision['loser']:
        return 'SAFE_WAIT'
    action = decision.get('loser_action', 'YIELD_AND_WAIT')
    if action == 'REROUTE':
        return 'REROUTE'
    if action == 'MOVE_ASIDE':
        return 'MOVE_ASIDE'
    return 'WAIT_FOR_CLEAR' if already_holding else 'YIELD_AND_WAIT'


def _active(state):
    return state.get('local_nav_state') not in (
        'DISABLED', 'MISSION_COMPLETE', 'PARKED', 'AVAILABLE', 'CHARGING', 'DEMO_STAGED', 'UNKNOWN')


def resolve_conflict(a, b, zone, route_costs=None, etas=None, waiting_cap=30.0,
                     waiting_quantum=1.0, eta_quantum=0.25,
                     priority_first=False, allow_reroute=True):
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
    legacy_hierarchy = (
        ('SAFETY', lambda s: not values['safety'][s['robot_id']]),
        ('ACTIVE_RESERVATION', lambda s: values['reserved'][s['robot_id']]),
        ('ZONE_COMMITTED', lambda s: values['committed'][s['robot_id']]),
        ('TASK_PRIORITY', lambda s: values['priority'][s['robot_id']]),
        ('EXECUTING_OVER_IDLE', lambda s: values['active'][s['robot_id']]),
        ('WAITING_TIME', lambda s: values['waiting'][s['robot_id']]),
        ('ETA', lambda s: -values['eta'][s['robot_id']]),
    )
    if priority_first:
        # A physically committed occupant is a collision-safety exception.
        # Otherwise task priority precedes reservation and every normal tie-break.
        hierarchy = (
            ('SAFETY', lambda s: not values['safety'][s['robot_id']]),
            ('ZONE_COMMITTED', lambda s: values['committed'][s['robot_id']]),
            ('TASK_PRIORITY', lambda s: values['priority'][s['robot_id']]),
            ('CHARGING_URGENCY', lambda s: int(s.get('charging_urgency', 0))),
            ('TASK_DEADLINE', lambda s: -float(s.get('deadline_remaining_s', math.inf))),
            ('LOADED', lambda s: int(bool(s.get('loaded', False)))),
            ('MISSION_PROGRESS', lambda s: round(float(s.get('mission_progress', 0.0)), 3)),
            ('WAITING_TIME', lambda s: values['waiting'][s['robot_id']]),
            ('CLEARANCE_TIME', lambda s: -float(s.get('clearance_time_s', math.inf))),
        )
    else:
        hierarchy = legacy_hierarchy
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
              'REROUTE' if allow_reroute and costs.get('route') and reroute_cost < wait_cost else
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
        # Retained after clear so a delayed DDS sample cannot resurrect an old
        # encounter.  The authority increments this value for the next epoch.
        self._highest_version = {}

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

    @staticmethod
    def _wire_decision(conflict_id, decision, version, authority, now,
                       first_detected, detection_monotonic_ns=None):
        zone, participants, expected_authority = conflict_identity(conflict_id)
        if authority != expected_authority:
            raise ValueError('Wrong decision authority')
        committed_ns=time.perf_counter_ns()
        detected_ns=int(detection_monotonic_ns or committed_ns)
        values=decision.details
        snapshot={
            'participants':list(participants),
            'safety':values.get('safety',{}),
            'reserved':values.get('reserved',{}),
            'committed':values.get('committed',{}),
            'priority':values.get('priority',{}),
            'active':values.get('active',{}),
            'waiting':values.get('waiting',{}),
            'eta':values.get('eta',{}),
            'decision_basis':decision.reason,
        }
        return {
            **decision.dictionary(),
            'status':'ACTIVE', 'conflict_id':conflict_id, 'zone_id':zone,
            'participants':list(participants), 'decision_version':int(version),
            'authority_robot_id':authority,
            'winner_robot_id':decision.winner, 'loser_robot_id':decision.loser,
            'decision_timestamp':float(now), 'decision_time':float(now),
            'first_detected_time':float(first_detected),
            'detection_monotonic_ns':detected_ns,
            'decision_monotonic_ns':committed_ns,
            'decision_latency_ms':max(0.0,(committed_ns-detected_ns)/1e6),
            'arbitration_snapshot':snapshot, 'winner_entered_zone':False,
        }

    def create_authoritative(self, conflict_id, authority, a, b, etas, now,
                             first_detected, zone='', route_costs=None,
                             detection_monotonic_ns=None, **policy):
        """Create one versioned decision; callable only by encounter authority."""
        _, participants, expected = conflict_identity(conflict_id)
        if authority != expected or authority not in participants:
            raise ValueError('Only canonical authority may create a decision')
        if conflict_id in self._decisions:
            return self._decisions[conflict_id]
        decision=resolve_conflict(a,b,zone,route_costs,etas,**policy)
        version=self._highest_version.get(conflict_id,0)+1
        value=self._wire_decision(conflict_id,decision,version,authority,now,
                                  first_detected,detection_monotonic_ns)
        self._highest_version[conflict_id]=version
        self._decisions[conflict_id]=value
        return value

    @staticmethod
    def _validate_wire(value):
        if not isinstance(value,dict) or value.get('status') != 'ACTIVE':
            raise ValueError('Not an active negotiation decision')
        cid=value.get('conflict_id','')
        zone,participants,authority=conflict_identity(cid)
        if value.get('zone_id') != zone:
            raise ValueError('Wrong decision zone')
        if tuple(value.get('participants',())) != participants:
            raise ValueError('Wrong decision participants')
        if value.get('authority_robot_id') != authority:
            raise ValueError('Wrong decision authority')
        winner=value.get('winner_robot_id'); loser=value.get('loser_robot_id')
        if {winner,loser} != set(participants) or winner == loser:
            raise ValueError('Invalid winner/loser pair')
        if value.get('loser_action') not in DECISION_ACTIONS:
            raise ValueError('Invalid loser action')
        version=value.get('decision_version')
        if not isinstance(version,int) or isinstance(version,bool) or version < 1:
            raise ValueError('Invalid decision version')
        return cid,version

    def adopt(self, value):
        """Validate and adopt an authority decision, rejecting stale versions."""
        try:
            cid,version=self._validate_wire(value)
        except ValueError:
            return False
        current=self._decisions.get(cid)
        if current and version == current['decision_version']:
            immutable=('authority_robot_id','winner_robot_id','loser_robot_id',
                       'loser_action','participants','zone_id')
            return all(value.get(key)==current.get(key) for key in immutable)
        if version <= self._highest_version.get(cid,0):
            return False
        adopted=dict(value)
        adopted['winner']=adopted['winner_robot_id']
        adopted['loser']=adopted['loser_robot_id']
        adopted.setdefault('details',{})
        adopted['winner_entered_zone']=False
        self._highest_version[cid]=version
        self._decisions[cid]=adopted
        return True

    def clear_message(self, conflict_id, reason, timestamp):
        """Build the authority's versioned clear record for peer adoption."""
        value=self._decisions[conflict_id]
        return {key:value[key] for key in ('conflict_id','zone_id','participants',
            'decision_version','authority_robot_id','winner_robot_id','loser_robot_id') } | {
            'status':'CLEARED','clear_reason':reason,'decision_timestamp':float(timestamp)}

    def adopt_clear(self, value):
        """Clear only the matching active version from the canonical authority."""
        if not isinstance(value,dict) or value.get('status') != 'CLEARED':
            return None
        try:
            cid=value['conflict_id']; zone,participants,authority=conflict_identity(cid)
            version=value['decision_version']
        except (KeyError,ValueError,TypeError):
            return None
        current=self._decisions.get(cid)
        if (not current or not isinstance(version,int) or
                value.get('zone_id') != zone or
                tuple(value.get('participants',())) != participants or
                value.get('authority_robot_id') != authority or
                version != current.get('decision_version')):
            return None
        if (value.get('winner_robot_id') != current.get('winner') or
                value.get('loser_robot_id') != current.get('loser')):
            return None
        self._highest_version[cid]=max(version,self._highest_version.get(cid,0))
        return self._decisions.pop(cid)

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


def physically_committed(state, zone, reservation_policy):
    """True once this approach can no longer stop before zone entry.

    This is a collision-safety exception to mission-priority ordering, not a
    normal secondary preference.  It uses the route's distance to zone entry.
    """
    if not outside_with_margin(state, zone, 0.0):
        return True
    intent=state.get('zone_intent') or {}
    distance=float(intent.get('distance', math.inf))
    speed=abs(float(state.get('linear_velocity', 0.0)))
    return (state.get('local_nav_state') not in
            ('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','UNKNOWN')
            and distance <= reservation_distance(speed, **reservation_policy))


def bid_eligible(controller_state, has_pending_assignment, pose_fresh):
    """Parking/repositioning is interruptible; an assigned task is not."""
    return (controller_state in ('DISABLED','MISSION_COMPLETE','DEMO_STAGED',
            'POST_TASK_REPOSITION','RETURNING_TO_CHARGE','GOING_TO_CHARGE',
            'CHARGING','PARKED','AVAILABLE','MOVE_ASIDE')
            and not has_pending_assignment and pose_fresh)
