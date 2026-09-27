"""Small warehouse graph, physical-position attachment and route zone intent."""
import heapq
import math
from .zone_prediction import load_zones


def segment_interval(a, b, zone):
    dx, dy = b[0]-a[0], b[1]-a[1]
    if 'radius' in zone:
        ox, oy = a[0]-zone['center'][0], a[1]-zone['center'][1]
        aa = dx*dx+dy*dy
        cc = ox*ox+oy*oy-zone['radius']**2
        if aa < 1e-12:
            return (0., 1.) if cc <= 0 else None
        bb = 2*(ox*dx+oy*dy)
        disc = bb*bb-4*aa*cc
        if disc < 0: return None
        lo, hi = (-bb-math.sqrt(disc))/(2*aa), (-bb+math.sqrt(disc))/(2*aa)
    else:
        lo, hi = 0., 1.
        for p, d, bounds in ((a[0],dx,zone['bounds']['x']), (a[1],dy,zone['bounds']['y'])):
            if abs(d) < 1e-12:
                if not bounds[0] <= p <= bounds[1]: return None
            else:
                s, t = sorted(((bounds[0]-p)/d,(bounds[1]-p)/d))
                lo, hi = max(lo,s), min(hi,t)
    return (max(0.,lo),min(1.,hi)) if max(0.,lo) <= min(1.,hi) else None


class WarehouseGraph:
    def __init__(self, cfg):
        g = cfg['route_graph']
        self.nodes = g['nodes']
        self.parking = tuple(g.get('parking_hotspots', ()))
        self.charging_station = g.get('charging_station', {})
        self.charging = tuple(self.charging_station.get('slots', ()))
        self.zones = {k:v for k,v in load_zones(cfg).items() if k != 'obstacle_area'}
        self.obstacles = g['obstacles']
        self.clearance = g['clearance']
        self.zone_bypasses = g.get('zone_bypasses', {})
        self.edges = {n:[] for n in self.nodes}
        for a,b in g['edges']:
            if not self.visible(self.nodes[a],self.nodes[b]):
                raise ValueError(f'Graph edge crosses rack clearance: {a}, {b}')
            d = math.dist(self.nodes[a], self.nodes[b])
            self.edges[a].append((b,d)); self.edges[b].append((a,d))
        for zone,pairs in self.zone_bypasses.items():
            for a,b in pairs:
                if (not self.visible(self.nodes[a],self.nodes[b]) or
                        segment_interval(self.nodes[a],self.nodes[b],self.zones[zone])):
                    raise ValueError(f'Unsafe bypass for {zone}: {a}, {b}')

    def visible(self,a,b):
        c = self.clearance
        if any(abs(p[0]) > 11.9-c or abs(p[1]) > 8.9-c for p in (a,b)):
            return False
        return not any(segment_interval(a,b,{'bounds':{'x':[x0-c,x1+c], 'y':[y0-c,y1+c]}})
                       for x0,x1,y0,y1 in self.obstacles)

    def astar(self, start, end, zone_penalties=None, include_bypass_for=None, stats=None):
        """Deterministic A* over the collision-checked warehouse graph.

        Edges retain their physical Euclidean length.  Callers can provide a
        transient mapping such as ``{'intersection_A': math.inf}`` or a large
        finite penalty.  The graph itself is never mutated.
        """
        if start not in self.nodes or end not in self.nodes: raise ValueError('Unknown station')
        penalties=zone_penalties or {}
        def heuristic(node):
            return math.dist(self.nodes[node],self.nodes[end])
        todo=[(heuristic(start),0.,start)]
        cost={start:0.}; parent={}
        expanded=0
        while todo:
            _,known,n=heapq.heappop(todo)
            if known != cost.get(n): continue
            expanded += 1
            if n == end:
                path=[]
                while n is not None:
                    path.append(n); n=parent.get(n)
                if stats is not None:
                    stats.update(expanded_nodes=expanded, raw_route_cost=known,
                                 start_node=start, goal_node=end, heuristic_weight=1.0)
                return known,list(reversed(path))
            neighbors=list(self.edges[n])
            if include_bypass_for:
                for a,b in self.zone_bypasses.get(include_bypass_for,()):
                    if n == a: neighbors.append((b,math.dist(self.nodes[a],self.nodes[b])))
                    if n == b: neighbors.append((a,math.dist(self.nodes[a],self.nodes[b])))
            for nxt,d in sorted(neighbors,key=lambda item:item[0]):
                edge_penalty=0.
                for zone,penalty in sorted(penalties.items()):
                    if zone in self.zones and segment_interval(self.nodes[n],self.nodes[nxt],self.zones[zone]):
                        edge_penalty+=float(penalty)
                if math.isinf(edge_penalty):
                    continue
                next_cost=known+d+edge_penalty
                if next_cost < cost.get(nxt,float('inf'))-1e-9:
                    cost[nxt]=next_cost; parent[nxt]=n
                    # Node ID makes equal f/g exploration stable across AMRs.
                    heapq.heappush(todo,(next_cost+heuristic(nxt),next_cost,nxt))
        raise ValueError('No warehouse route')

    def attachment(self,position):
        candidates=[n for n,p in self.nodes.items() if self.visible(position,p)]
        if not candidates: raise ValueError('Current pose has no safe graph connection')
        return min(candidates,key=lambda n:(math.dist(position,self.nodes[n]),n))

    def pickup_distance(self,position,pickup):
        n=self.attachment(position)
        return math.dist(position,self.nodes[n])+self.astar(n,pickup)[0]

    def mission(self,position,pickup,destination):
        return self.mission_plan(position,pickup,destination)['smoothed_waypoints']

    def _protected_node(self, name):
        """Keep controlled-zone/hold/station nodes for coordination semantics."""
        point=self.nodes[name]
        return (name in self.parking or name.startswith(('PICKUP','DROP','NARROW','AISLE_')) or
                any(segment_interval(point,point,zone) for zone in self.zones.values()))

    def smooth_names(self, names):
        """Greedy line-of-sight compression without deleting protected nodes."""
        if len(names)<3: return list(names)
        result=[names[0]]; i=0
        while i<len(names)-1:
            chosen=i+1
            for j in range(i+2,len(names)):
                skipped=names[i+1:j]
                if any(self._protected_node(n) for n in skipped): break
                if not self.visible(self.nodes[names[i]],self.nodes[names[j]]): break
                # Do not smooth through a controlled zone that was not already
                # traversed by the raw graph segment.
                crossed=[z for z,zone in self.zones.items()
                         if segment_interval(self.nodes[names[i]],self.nodes[names[j]],zone)]
                raw_crossed=[z for z,zone in self.zones.items()
                             if any(segment_interval(self.nodes[a],self.nodes[b],zone)
                                    for a,b in zip(names[i:j],names[i+1:j+1]))]
                if set(crossed) - set(raw_crossed): break
                chosen=j
            result.append(names[chosen]); i=chosen
        return result

    def mission_plan(self,position,pickup,destination):
        n=self.attachment(position)
        first_stats={}; second_stats={}
        first_cost,first=self.astar(n,pickup,stats=first_stats)
        second_cost,second=self.astar(pickup,destination,stats=second_stats)
        names=first+second[1:]
        # Opposing aisle entrants wait in lateral pockets. Exiting traffic uses
        # the western exit, so a waiting robot never blocks the far portal.
        for portal,hold,other in [('NARROW_AISLE_SOUTH','AISLE_SOUTH_HOLD','NARROW_AISLE_NORTH'),
                                  ('NARROW_AISLE_NORTH','AISLE_NORTH_HOLD','NARROW_AISLE_SOUTH')]:
            i=0
            while i < len(names)-1:
                if names[i:i+2] == [portal,other] and (i==0 or names[i-1]!=hold):
                    names.insert(i,hold); i+=1
                i+=1
        raw_names=list(names)
        names=self.smooth_names(names)
        raw_points=[list(position)]
        for name in raw_names:
            point=self.nodes[name]
            if math.dist(raw_points[-1],point)>0.03: raw_points.append(list(point))
        points=[list(position)]
        for name in names:
            point=self.nodes[name]
            if math.dist(points[-1],point)>0.03: points.append(list(point))
        if not all(self.visible(a,b) for a,b in zip(points,points[1:])):
            raise ValueError('Route attachment or aisle approach is obstructed')
        return dict(planner='A*',start_node=n,goal_node=destination,
                    raw_astar_nodes=raw_names,raw_waypoints=raw_points,
                    raw_route_cost=first_cost+second_cost,
                    expanded_nodes=first_stats['expanded_nodes']+second_stats['expanded_nodes'],
                    smoothed_node_names=names,smoothed_waypoints=points,
                    smoothed_route_length=self.polyline_length(points))

    @staticmethod
    def polyline_length(points):
        return sum(math.dist(a,b) for a,b in zip(points,points[1:]))

    @staticmethod
    def point_segment_distance(point, a, b):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=dx*dx+dy*dy
        if length < 1e-12: return math.dist(point,a)
        t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length))
        return math.dist(point,(a[0]+t*dx,a[1]+t*dy))

    def route_to_with_cost(self, position, destination, zone_penalties=None):
        """Return graph cost and A* route from the current physical position."""
        penalties=zone_penalties or {}
        candidates=[]
        for node,point in self.nodes.items():
            if not self.visible(position,point): continue
            if any(zone in self.zones and math.isinf(float(penalty)) and
                   segment_interval(position,point,self.zones[zone])
                   for zone,penalty in penalties.items()):
                continue
            try:
                cost,names=self.astar(node,destination,penalties,
                    next(iter(penalties),None) if penalties else None)
            except ValueError:
                continue
            candidates.append((math.dist(position,point)+cost,node,names))
        if not candidates: raise ValueError('No route avoiding controlled zone')
        total_cost,_,names=min(candidates)
        names=self.smooth_names(names)
        points=[list(position)]
        for name in names:
            if math.dist(points[-1],self.nodes[name])>.03: points.append(list(self.nodes[name]))
        return total_cost,points

    def route_to(self, position, destination, zone_penalties=None):
        """A* route from physical position with optional transient zone costs."""
        return self.route_to_with_cost(position,destination,zone_penalties)[1]

    def reroute(self, position, remaining, destination, blocked_zone):
        """A* alternate with a temporary controlled-zone exclusion."""
        direct=self.polyline_length([position]+list(remaining))
        alternate=self.route_to(position,destination,{blocked_zone:math.inf})
        return alternate,max(0.,self.polyline_length(alternate)-direct)

    def parking_candidates(self, position, unavailable=(), active_routes=(), exclude=()):
        """Nearest reachable parking nodes not reserved or on an active route."""
        blocked=set(unavailable)|set(exclude)
        routes=[r for r in active_routes if len(r)>1]
        candidates=[]
        for name in self.parking:
            point=self.nodes[name]
            if name in blocked: continue
            if any(self.point_segment_distance(point,a,b)<0.9
                   for route in routes for a,b in zip(route,route[1:])):
                continue
            try: route=self.route_to(position,name)
            except ValueError: continue
            candidates.append((self.polyline_length(route),name,route))
        return sorted(candidates,key=lambda item:(item[0],item[1]))

    def charging_candidates(self, position, unavailable=(), active_routes=()):
        """A*-cost ordered slots inside one shared charging station."""
        blocked=set(unavailable); routes=[r for r in active_routes if len(r)>1]; choices=[]
        for name in self.charging:
            point=self.nodes[name]
            if name in blocked or any(self.point_segment_distance(point,a,b)<0.8
                                      for route in routes for a,b in zip(route,route[1:])):
                continue
            try: cost,route=self.route_to_with_cost(position,name)
            except ValueError: continue
            choices.append((cost,name,route))
        return sorted(choices,key=lambda item:(item[0],item[1]))

    def intent(self,position,remaining):
        points=[position]+list(remaining)
        candidates=[]
        for name,zone in self.zones.items():
            traveled=0.; entry=None; leave=0.
            for a,b in zip(points,points[1:]):
                length=math.dist(a,b); interval=segment_interval(a,b,zone)
                if interval:
                    if entry is None: entry=traveled+interval[0]*length
                    leave=traveled+interval[1]*length
                elif entry is not None: break
                traveled+=length
            if entry is not None: candidates.append((entry,name,leave))
        if not candidates: return {}
        entry,name,leave=min(candidates)
        # Predict controlled-zone arrival with the conservative controlled-zone
        # speed, while the controller may use 0.50 m/s in open corridors.
        return {'zone':name,'distance':entry,'eta':entry/0.35,
                'window':[entry/0.35,max(leave/0.35,entry/0.35+1.)],
                'inside':segment_interval(position,position,self.zones[name]) is not None}
