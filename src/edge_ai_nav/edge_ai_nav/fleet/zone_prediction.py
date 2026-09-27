"""Stage 5 pure local occupancy prediction. No ROS or motion decisions."""
import math
import time


def conflict_id(zone, a, b):
    return ':'.join((zone, *sorted((a, b))))


def overlap(a, b, buffer=0.0):
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]) + buffer)


def load_zones(warehouse):
    zones = {}
    for name in ('intersection_A', 'intersection_B', 'narrow_aisle_1', 'obstacle_demo_area'):
        source = warehouse['zones'][name]
        zone = dict(source)
        if name == 'narrow_aisle_1':
            cx, cy = source['center']
            zone['bounds'] = {'x': [cx-source['clear_width_m']/2, cx+source['clear_width_m']/2],
                              'y': [cy-source['length_m']/2, cy+source['length_m']/2]}
        zones['obstacle_area' if name == 'obstacle_demo_area' else name] = zone
    return zones


def window(state, zone, minimum_speed=0.25, horizon=6.0):
    """Ray/zone intersection yields distance to entry and distance inside zone.

    Route goal gives approach direction; moving heading must also approach.
    Stopped but enabled missions use nominal speed. Completed/disabled missions
    outside the zone have no intent; a robot inside still occupies the zone.
    """
    intent = state.get('zone_intent')
    if intent and intent.get('zone') == state.get('next_zone'):
        if intent['eta'] > horizon and intent['zone'] != 'narrow_aisle_1' and not intent['inside']:
            return None
        return intent
    x, y = state['x'], state['y']
    cx, cy = zone['center']
    if 'radius' in zone:
        inside = math.hypot(x-cx, y-cy) <= zone['radius']
    else:
        bounds = zone['bounds']
        inside = bounds['x'][0] <= x <= bounds['x'][1] and bounds['y'][0] <= y <= bounds['y'][1]
    if not inside and state.get('local_nav_state') in ('DISABLED', 'MISSION_COMPLETE', 'UNKNOWN', 'SENSOR_STALE_SAFE_STOP'):
        return None
    dx, dy = state['goal_x']-x, state['goal_y']-y
    length = math.hypot(dx, dy)
    if length < 1e-8:
        if inside: return {'distance': 0.0, 'eta': 0.0, 'window': (0.0, horizon), 'inside': True}
        return None
    ux, uy = dx/length, dy/length
    speed = max(abs(state['linear_velocity']), minimum_speed)
    if not inside and state['linear_velocity'] > 0.02:
        if math.cos(state['yaw'])*(cx-x) + math.sin(state['yaw'])*(cy-y) <= 0:
            return None
    if 'radius' in zone:
        projection = (cx-x)*ux + (cy-y)*uy
        disc = zone['radius']**2 - ((cx-x)**2+(cy-y)**2-projection**2)
        if disc < 0: return None
        entry, leave = projection-math.sqrt(disc), projection+math.sqrt(disc)
    else:
        entry, leave = -math.inf, math.inf
        for pos, direction, bound in ((x, ux, zone['bounds']['x']), (y, uy, zone['bounds']['y'])):
            if abs(direction) < 1e-9:
                if not bound[0] <= pos <= bound[1]: return None
            else:
                a, b = sorted(((bound[0]-pos)/direction, (bound[1]-pos)/direction))
                entry, leave = max(entry, a), min(leave, b)
    if leave < max(entry, 0.0): return None
    distance = max(0.0, entry)
    eta = distance/speed
    if eta > horizon: return None
    return {'distance': distance, 'eta': eta,
            'window': (eta, max(leave, 0.0)/speed), 'inside': inside}


class LocalDetector:
    def __init__(self, zones, horizon=6.0, buffer=0.75, minimum_speed=0.25):
        self.zones, self.horizon, self.buffer, self.minimum_speed = zones, horizon, buffer, minimum_speed
        self.active = {}
        self.event_count = 0

    def update(self, own, peers, now, unavailable=()):
        predicted = {}
        zone_name = own.get('next_zone', '')
        zone = self.zones.get(zone_name)
        mine = window(own, zone, self.minimum_speed, self.horizon) if zone else None
        if mine:
            for peer in peers:
                if peer['robot_id'] == own['robot_id'] or peer.get('next_zone') != zone_name: continue
                theirs = window(peer, zone, self.minimum_speed, self.horizon)
                if not theirs: continue
                seconds = overlap(mine['window'], theirs['window'], self.buffer)
                # The aisle is exclusive for its entire traverse, including
                # opposed entrants whose estimated arrival times differ.
                if seconds <= 0 and zone_name != 'narrow_aisle_1': continue
                cid = conflict_id(zone_name, own['robot_id'], peer['robot_id'])
                predicted[cid] = dict(conflict_id=cid, peer=peer['robot_id'], zone=zone_name,
                    my_eta=mine['eta'], peer_eta=theirs['eta'], my_distance=mine['distance'],
                    peer_distance=theirs['distance'], overlap_seconds=seconds,
                    state='ACTIVE' if mine['inside'] and theirs['inside'] else 'PREDICTED',
                    conflict_first_detected_time=self.active.get(cid, {}).get('conflict_first_detected_time', now),
                    conflict_first_detected_monotonic_ns=self.active.get(cid, {}).get(
                        'conflict_first_detected_monotonic_ns',time.perf_counter_ns()))
        events = []
        for cid, record in predicted.items():
            if cid not in self.active:
                self.event_count += 1
                events.append(('CONFLICT_PREDICTED', record))
            elif record['state'] != self.active[cid]['state']:
                events.append(('CONFLICT_ACTIVE', record))
        for cid, record in self.active.items():
            if cid not in predicted:
                event = 'CONFLICT_INVALIDATED_PEER_OFFLINE' if record['peer'] in unavailable else 'CONFLICT_CLEARED'
                events.append((event, dict(record, state='CLEARED')))
        self.active = predicted
        return events
