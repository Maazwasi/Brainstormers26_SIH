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
        self.large_platform=bool(cfg.get('scale_speed_upgrade',{}).get('experimental',False))
        self.peer_clearance=float(cfg.get('stage6',{}).get('physical_stop_distance_m',2.0))
        bounds = g.get('bounds', cfg.get('bounds', {'x': [-12.0, 12.0], 'y': [-9.0, 9.0]}))
        self.bounds = bounds
        self.side_aisle_cost_multiplier = float(g.get('side_aisle_cost_multiplier', 1.0))
        self.forward_rejoin = bool(g.get('forward_rejoin', False))
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
        if any(not (self.bounds['x'][0]+c <= p[0] <= self.bounds['x'][1]-c and
                    self.bounds['y'][0]+c <= p[1] <= self.bounds['y'][1]-c)
               for p in (a,b)):
            return False
        return not any(segment_interval(a,b,{'bounds':{'x':[x0-c,x1+c], 'y':[y0-c,y1+c]}})
                       for x0,x1,y0,y1 in self.obstacles)

    def astar(self, start, end, zone_penalties=None, include_bypass_for=None, stats=None,
              blocked_edges=()):
        """Deterministic A* over the collision-checked warehouse graph.

        Edges retain their physical Euclidean length.  Callers can provide a
        transient mapping such as ``{'intersection_A': math.inf}`` or a large
        finite penalty.  The graph itself is never mutated.
        """
        if start not in self.nodes or end not in self.nodes: raise ValueError('Unknown station')
        penalties=zone_penalties or {}
        blocked = {frozenset(pair) for pair in blocked_edges}
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
                if frozenset((n,nxt)) in blocked:
                    continue
                edge_penalty=0.
                for zone,penalty in sorted(penalties.items()):
                    if zone in self.zones and segment_interval(self.nodes[n],self.nodes[nxt],self.zones[zone]):
                        edge_penalty+=float(penalty)
                if math.isinf(edge_penalty):
                    continue
                side_factor = (self.side_aisle_cost_multiplier
                               if not (n.startswith('MAIN_') and nxt.startswith('MAIN_'))
                               else 1.0)
                next_cost=known+d*side_factor+edge_penalty
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
        departure=self.charging_departure(position)
        if departure is not None:
            approach=self.nodes[departure]
            return math.dist(position,approach)+self.pickup_distance(approach,pickup)
        n=self.attachment(position)
        return math.dist(position,self.nodes[n])+self.astar(n,pickup)[0]

    def mission(self,position,pickup,destination):
        return self.mission_plan(position,pickup,destination)['smoothed_waypoints']

    def _protected_node(self, name):
        """Keep controlled-zone/hold/station nodes for coordination semantics."""
        point=self.nodes[name]
        return (name in self.parking or name.startswith(('PICKUP','DROP','NARROW','AISLE_')) or
                (self.forward_rejoin and name.endswith('_APPROACH') and
                 name.removesuffix('_APPROACH') in self.charging) or
                any(segment_interval(point,point,zone) for zone in self.zones.values()))

    def charging_departure(self, position):
        """Keep a docked V2 AMR in its own lane until the south entry."""
        if not self.forward_rejoin:
            return None
        for slot in self.charging:
            approach=f'{slot}_APPROACH'
            if approach not in self.nodes:
                continue
            dock=self.nodes[slot]; entry=self.nodes[approach]
            if abs(position[0]-dock[0])<0.75 and position[1]>entry[1]+0.5:
                if self.visible(position,entry):
                    return approach
        return None

    def smooth_names(self, names, forbidden_zones=()):
        """Greedy line-of-sight compression without deleting protected nodes."""
        if len(names)<3: return list(names)
        result=[names[0]]; i=0
        while i<len(names)-1:
            chosen=i+1
            for j in range(i+2,len(names)):
                skipped=names[i+1:j]
                if any(self._protected_node(n) for n in skipped): break
                if not self.visible(self.nodes[names[i]],self.nodes[names[j]]): break
                if any(segment_interval(self.nodes[names[i]],self.nodes[names[j]],
                                        self.zones[zone])
                       for zone in forbidden_zones if zone in self.zones):
                    break
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
        departure=self.charging_departure(position)
        if departure is not None:
            entry=self.nodes[departure]
            plan=self.mission_plan(entry,pickup,destination)
            for field in ('raw_waypoints','smoothed_waypoints'):
                plan[field]=[list(position)]+plan[field]
            plan['start_node']=departure
            plan['raw_route_cost']+=math.dist(position,entry)
            plan['smoothed_route_length']=self.polyline_length(plan['smoothed_waypoints'])
            return plan
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

    @staticmethod
    def segment_distance(a, b, c, d):
        """Minimum planar separation of two closed route segments."""
        def cross(p, q, r):
            return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
        ab_c,ab_d=cross(a,b,c),cross(a,b,d)
        cd_a,cd_b=cross(c,d,a),cross(c,d,b)
        if (ab_c*ab_d<=0 and cd_a*cd_b<=0 and
                max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0])) and
                max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))):
            return 0.0
        return min(WarehouseGraph.point_segment_distance(p,u,v)
                   for p,u,v in ((a,c,d),(b,c,d),(c,a,b),(d,a,b)))

    @classmethod
    def route_clearance(cls, route, peer_route):
        """Conservative spatial clearance; zero when routes intersect."""
        if len(route)<2 or len(peer_route)<2:
            return 0.0
        return min(cls.segment_distance(a,b,c,d)
                   for a,b in zip(route,route[1:])
                   for c,d in zip(peer_route,peer_route[1:]))

    def route_to_with_cost(self, position, destination, zone_penalties=None):
        """Return graph cost and A* route from the current physical position."""
        departure=self.charging_departure(position)
        if departure is not None and destination not in self.charging:
            entry=self.nodes[departure]
            cost,route=self.route_to_with_cost(entry,destination,zone_penalties)
            return math.dist(position,entry)+cost,[list(position)]+route
        if self.forward_rejoin and destination in self.charging:
            # Every V2 charger is approached along its own south entry. A
            # diagonal from the main aisle can cut inside an occupied
            # neighbour's physical brake radius and deadlock at the dock.
            approach=f'{destination}_APPROACH'
            cost,route=self.route_to_with_cost(position,approach,zone_penalties)
            end=list(self.nodes[destination])
            if not self.visible(route[-1],end):
                raise ValueError('Charging approach is blocked')
            return cost+math.dist(route[-1],end),route+[end]
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
        forbidden={zone for zone,penalty in penalties.items()
                   if math.isinf(float(penalty)) and zone in self.zones}
        names=self.smooth_names(names,forbidden)
        points=[list(position)]
        for name in names:
            if math.dist(points[-1],self.nodes[name])>.03: points.append(list(self.nodes[name]))
        if any(segment_interval(a,b,self.zones[zone])
               for zone in forbidden for a,b in zip(points,points[1:])):
            raise ValueError('Smoothed route enters blocked coordination zone')
        return total_cost,points

    def route_to(self, position, destination, zone_penalties=None):
        """A* route from physical position with optional transient zone costs."""
        return self.route_to_with_cost(position,destination,zone_penalties)[1]

    def reroute(self, position, remaining, destination, blocked_zone):
        """A* alternate with a temporary controlled-zone exclusion."""
        direct=self.polyline_length([position]+list(remaining))
        alternate=self.route_to(position,destination,{blocked_zone:math.inf})
        return alternate,max(0.,self.polyline_length(alternate)-direct)

    def reroute_blocked_edge(self, position, destination, heading):
        """A* fallback when a physical obstacle closes the current aisle edge.

        The global graph is not mutated.  The closest forward edge is excluded
        for this one plan and the robot first returns to a visible node behind
        the blockage, preventing a direct attachment through the obstacle.
        """
        if destination not in self.nodes:
            raise ValueError('Unknown destination')
        forward=(math.cos(heading),math.sin(heading))
        unique=[]
        for a,neighbors in self.edges.items():
            for b,_ in neighbors:
                if a < b:
                    midpoint=((self.nodes[a][0]+self.nodes[b][0])/2,
                              (self.nodes[a][1]+self.nodes[b][1])/2)
                    along=((midpoint[0]-position[0])*forward[0]+
                           (midpoint[1]-position[1])*forward[1])
                    distance=self.point_segment_distance(position,self.nodes[a],self.nodes[b])
                    if along >= 0.15:
                        unique.append((distance,max(0.,along),a,b))
        if not unique:
            raise ValueError('No forward graph edge to block')
        _,_,edge_a,edge_b=min(unique)
        behind=[]
        for name,point in self.nodes.items():
            along=((point[0]-position[0])*forward[0]+(point[1]-position[1])*forward[1])
            if along <= 0.15 and self.visible(position,point):
                behind.append((math.dist(position,point),name))
        blocked=((edge_a,edge_b),)
        candidates=[]
        for distance,attach in sorted(behind):
            try:
                cost,names=self.astar(attach,destination,blocked_edges=blocked)
            except ValueError:
                continue
            route=[list(position)]
            for name in names:
                point=list(self.nodes[name])
                if math.dist(route[-1],point)>.03:
                    route.append(point)
            if all(self.visible(a,b) for a,b in zip(route,route[1:])):
                candidates.append((distance+cost,attach,route))
        if not candidates:
            raise ValueError('No A* route around blocked aisle edge')
        _,attach,route=min(candidates,key=lambda item:(item[0],item[1]))
        return route,(edge_a,edge_b),attach

    def reroute_forward(self, position, remaining, destination, blocked_zone):
        """Avoid a zone, then rejoin an original-route node strictly downstream.

        This planner is separate from the legacy reroute path.  It does not
        authorize physical execution; the caller must additionally establish
        peer clearance before choosing REROUTE over YIELD.
        """
        if blocked_zone not in self.zones or destination not in self.nodes:
            raise ValueError('Unknown blocked zone or destination')
        original=[list(position)]+[list(p) for p in remaining]
        if len(original)<2:
            raise ValueError('No original route to rejoin')
        zone=self.zones[blocked_zone]
        first_hit=next((i for i,(a,b) in enumerate(zip(original,original[1:]))
                        if segment_interval(a,b,zone)),None)
        if first_hit is None:
            raise ValueError('Original route does not enter blocked zone')
        initial=next(((b[0]-position[0],b[1]-position[1]) for b in original[1:]
                      if math.dist(position,b)>0.03),None)
        if initial is None:
            raise ValueError('No forward route direction')
        norm=math.hypot(*initial)
        candidates=[]
        for rejoin_index in range(first_hit+1,len(original)):
            rejoin=original[rejoin_index]
            name=next((n for n,p in self.nodes.items()
                       if math.dist(p,rejoin)<0.05),None)
            if name is None or segment_interval(rejoin,rejoin,zone):
                continue
            tail=original[rejoin_index+1:]
            if any(not self.visible(a,b) or segment_interval(a,b,zone)
                   for a,b in zip([rejoin]+tail,tail)):
                continue
            for attach,point in self.nodes.items():
                if not self.visible(position,point) or segment_interval(position,point,zone):
                    continue
                offset=(point[0]-position[0],point[1]-position[1])
                length=math.hypot(*offset)
                # Reject a first leg that turns back toward the completed
                # section. Lateral bypasses remain eligible up to 107 degrees.
                if length>0.03 and (initial[0]*offset[0]+initial[1]*offset[1])/(norm*length)<-0.3:
                    continue
                try:
                    _,names=self.astar(attach,name,{blocked_zone:math.inf},
                                       include_bypass_for=blocked_zone)
                except ValueError:
                    continue
                names=self.smooth_names(names,(blocked_zone,))
                route=[list(position)]
                for node in names:
                    point=self.nodes[node]
                    if math.dist(route[-1],point)>0.03:
                        route.append(list(point))
                for point in tail:
                    if math.dist(route[-1],point)>0.03:
                        route.append(list(point))
                if (route[-1] != list(self.nodes[destination]) or
                        any(not self.visible(a,b) or segment_interval(a,b,zone)
                            for a,b in zip(route,route[1:]))):
                    continue
                candidates.append((self.polyline_length(route),rejoin_index,
                                   attach,route))
        if not candidates:
            raise ValueError('No forward-safe downstream rejoin')
        length,_,_,route=min(candidates,key=lambda v:(v[0],v[1],v[2]))
        original_length=self.polyline_length(original)
        return route,max(0.,length-original_length)

    def parking_candidates(self, position, unavailable=(), active_routes=(), exclude=()):
        """Nearest reachable parking nodes not reserved or on an active route."""
        blocked=set(unavailable)|set(exclude)
        routes=[r for r in active_routes if len(r)>1]
        candidates=[]
        for name in self.parking:
            point=self.nodes[name]
            if name in blocked: continue
            if any(self.point_segment_distance(point,a,b)<(self.peer_clearance if self.large_platform else 0.9)
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
            if name in blocked or any(self.point_segment_distance(point,a,b)<(self.peer_clearance if self.large_platform else 0.8)
                                      for route in routes for a,b in zip(route,route[1:])):
                continue
            try: cost,route=self.route_to_with_cost(position,name)
            except ValueError: continue
            choices.append((cost,name,route))
        return sorted(choices,key=lambda item:(item[0],item[1]))

    def intent(self,position,remaining,zone_name=None):
        points=[position]+list(remaining)
        candidates=[]
        for name,zone in self.zones.items():
            if zone_name is not None and name!=zone_name:continue
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
