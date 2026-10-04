"""Pure geometry for conservative, unzoned peer-path crossing protection."""
import math


def segment_crossing(a, b, c, d):
    """Return the intersection point and fractional progress on both segments."""
    rx, ry = b[0]-a[0], b[1]-a[1]
    sx, sy = d[0]-c[0], d[1]-c[1]
    denominator = rx*sy-ry*sx
    if abs(denominator) < 1e-9:
        return None  # Collinear traffic remains governed by corridor/LiDAR safety.
    qx, qy = c[0]-a[0], c[1]-a[1]
    t = (qx*sy-qy*sx)/denominator
    u = (qx*ry-qy*rx)/denominator
    if -1e-8 <= t <= 1.+1e-8 and -1e-8 <= u <= 1.+1e-8:
        return ((a[0]+t*rx,a[1]+t*ry),max(0.,min(1.,t)),max(0.,min(1.,u)))
    return None


def approaching_crossing(own, peer, horizon_m=8.0, eta_slack_s=10.0,
                         nominal_speed=0.35):
    """Find an upcoming route intersection for both AMRs, if timing can overlap."""
    def legs(state):
        route=[(float(state['x']),float(state['y']))]
        route.extend((float(p[0]),float(p[1])) for p in state.get('remaining_route',()))
        traveled=0.0
        for a,b in zip(route,route[1:]):
            length=math.dist(a,b)
            if length<0.03:
                continue
            yield a,b,traveled,length
            traveled+=length
            if traveled>horizon_m:
                break
    candidates=[]
    for a,b,own_base,own_length in legs(own):
        for c,d,peer_base,peer_length in legs(peer):
            crossing=segment_crossing(a,b,c,d)
            if crossing is None:
                continue
            point,t,u=crossing
            own_distance=own_base+t*own_length
            peer_distance=peer_base+u*peer_length
            if own_distance>horizon_m or peer_distance>horizon_m:
                continue
            own_eta=own_distance/max(nominal_speed,abs(float(own.get('linear_velocity',0))))
            peer_eta=peer_distance/max(nominal_speed,abs(float(peer.get('linear_velocity',0))))
            if abs(own_eta-peer_eta)>eta_slack_s:
                continue
            candidates.append((own_distance+peer_distance,point,own_distance,
                               peer_distance,own_eta,peer_eta))
    if not candidates:
        return None
    _,point,own_distance,peer_distance,own_eta,peer_eta=min(candidates)
    return dict(point=point,own_distance=own_distance,
                peer_distance=peer_distance,own_eta=own_eta,peer_eta=peer_eta)


def should_yield_for_path(own, peer):
    """Priority first; a stable robot-ID tie-break prevents reciprocal holds."""
    own_priority=int(own.get('task_priority',0))
    peer_priority=int(peer.get('task_priority',0))
    return (own_priority<peer_priority or
            (own_priority==peer_priority and own['robot_id']>peer['robot_id']))


def yield_pocket(graph, own, winner, peers, clearance=3.0):
    """Find a short, forward/lateral escape with a clear swept footprint.

    The task route is not changed. The caller holds at this pocket until the
    encounter clears. No candidate is permission to pass through another AMR.
    """
    position=(float(own['x']),float(own['y']))
    winner_position=(float(winner['x']),float(winner['y']))
    remaining=own.get('remaining_route') or []
    forward=next(((p[0]-position[0],p[1]-position[1]) for p in remaining
                  if math.dist(position,p)>.2),None)
    if forward is None:
        return None
    heading=math.atan2(forward[1],forward[0])
    # The pocket stays occupied until the winner clears. Check its complete
    # remaining route, including a farther endpoint it can reach while the
    # loser waits; truncating at 8 m can pick a pocket beside that endpoint.
    winner_path=[winner_position]
    for point in winner.get('remaining_route') or []:
        leg=math.dist(winner_path[-1],point)
        if leg<.03:
            continue
        winner_path.append(tuple(point))
    if len(winner_path)<2:
        return None
    candidates=[]
    for length in (1.5,2.,2.5,3.,3.5,4.):
        for angle in (-90,90,-60,60,-30,30,0):
            direction=heading+math.radians(angle)
            target=(position[0]+length*math.cos(direction),
                    position[1]+length*math.sin(direction))
            if not graph.visible(position,target):
                continue
            if min(graph.point_segment_distance(target,a,b)
                   for a,b in zip(winner_path,winner_path[1:]))<clearance:
                continue
            safe=True
            for peer in peers:
                point=(float(peer['x']),float(peer['y']))
                original=math.dist(position,point)
                swept=graph.point_segment_distance(point,position,target)
                if (swept<1.55 or math.dist(target,point)<clearance or
                        (peer['robot_id']==winner['robot_id'] and swept<original-.02)):
                    safe=False
                    break
            if safe:
                candidates.append((length,abs(angle),angle,target))
    return list(min(candidates)[-1]) if candidates else None
