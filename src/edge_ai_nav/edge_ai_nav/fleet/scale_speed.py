"""Opt-in enlarged-platform safety calculations, in physical SI units.

These are estimates, not measured braking performance or acceptance evidence.
"""
import math


def braking_distance(speed, deceleration=0.65):
    if deceleration <= 0:
        raise ValueError('deceleration must be positive')
    return abs(speed) ** 2 / (2.0 * deceleration)


def travel_time(distance, current_speed, target_speed, acceleration=0.65):
    """Acceleration-aware forward ETA; a stopped AMR is not already at cruise."""
    distance=max(0.0,distance)
    target=max(.05,target_speed); current=min(target,abs(current_speed))
    accelerating=(target*target-current*current)/(2*acceleration)
    if distance<=accelerating:
        return (math.sqrt(current*current+2*acceleration*distance)-current)/acceleration
    return (target-current)/acceleration+(distance-accelerating)/target


def projected_peer_gap(own, peer, deceleration=.65, reaction_seconds=.35):
    """Conservative constant-velocity relative sweep over both stopping times."""
    r=(peer['x']-own['x'],peer['y']-own['y'])
    def velocity(state):
        v=float(state.get('linear_velocity',0.));yaw=float(state.get('yaw',0.))
        return (v*math.cos(yaw),v*math.sin(yaw))
    a,b=velocity(own),velocity(peer);rv=(b[0]-a[0],b[1]-a[1])
    length2=rv[0]*rv[0]+rv[1]*rv[1]
    duration=max(math.hypot(*a),math.hypot(*b))/deceleration+reaction_seconds
    closest=(max(0.,min(duration,-(r[0]*rv[0]+r[1]*rv[1])/length2))
             if length2>1e-12 else 0.)
    return math.hypot(r[0]+rv[0]*closest,r[1]+rv[1]*closest)


def obstacle_distance(speed, front_overhang=0.66, margin=0.64,
                      reaction_seconds=0.35, deceleration=0.65):
    return (front_overhang + margin + abs(speed) * reaction_seconds +
            braking_distance(speed, deceleration))


def peer_envelope(own, peer, physical_diameter=2.36, margin=0.82,
                  reaction_seconds=0.35, deceleration=0.65):
    """Centre gap plus projected relative closing speed and both braking legs.

    Parallel departures do not get a head-on braking envelope. The footprint
    gap remains mandatory even when both robots are stationary.
    """
    dx, dy = peer['x'] - own['x'], peer['y'] - own['y']
    distance = math.hypot(dx, dy)
    if distance < 1e-9:
        return physical_diameter + margin, 0.0
    nx, ny = dx / distance, dy / distance
    def component(state):
        v = float(state.get('linear_velocity', 0.0))
        yaw = float(state.get('yaw', 0.0))
        return v * (math.cos(yaw) * nx + math.sin(yaw) * ny)
    a, b = component(own), component(peer)
    closing = max(0.0, a - b)
    braking = braking_distance(max(0.0, a), deceleration)
    braking += braking_distance(max(0.0, -b), deceleration)
    return physical_diameter + margin + closing * reaction_seconds + braking, closing


def forward_scan(scan, half_width=0.76, lidar_x=0.24, margin=0.15):
    """Distance from LiDAR to returns in the actual forward swept body strip.

    Unlike a fixed cone, distant side shelves cannot falsely block a straight
    aisle. Invalid scans fail closed; positive infinite beams are free space.
    """
    values = []
    valid = False
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        if abs(angle) > math.pi / 2:
            continue
        if math.isinf(distance) and distance > 0:
            valid = True
            continue
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        valid = True
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if x + lidar_x >= 0 and abs(y) <= half_width + margin:
            values.append(x)
    return min(values) if values else scan.range_max if valid else None
