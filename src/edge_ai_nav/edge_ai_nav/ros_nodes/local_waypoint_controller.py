"""Local odometry/LaserScan autonomy with a local, non-motion fleet hold input."""
import json
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from edge_ai_nav.fleet.negotiation import motion_gate
from edge_ai_nav.fleet.route_graph import WarehouseGraph


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def sectors(scan):
    buckets = {k: [] for k in ('front', 'left', 'right')}
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        a = wrap(scan.angle_min + i * scan.angle_increment)
        if abs(a) < 0.38:
            buckets['front'].append(distance)
        if 0.38 <= a <= 1.5:
            buckets['left'].append(distance)
        if -1.5 <= a <= -0.38:
            buckets['right'].append(distance)
    # Third-smallest return rejects up to two isolated noise beams.
    return {k: sorted(v)[min(2, len(v)-1)] if v else None for k, v in buckets.items()}


class LocalController(Node):
    def __init__(self):
        super().__init__('local_waypoint_controller')
        defaults = dict(robot_id='amr_alpha', config_file='', enabled=False,
                        odom_coordinates='local',
                        max_linear_speed=0.50, max_angular_speed=0.50,
                        lookahead_distance=0.55, linear_acceleration=0.65,
                        controller_trace=False, enable_obstacle_replan=False,
                        diagnostic_mode=False,
                        goal_tolerance=0.16, obstacle_stop_distance=0.85,
                        obstacle_prepare_distance=1.25,
                        obstacle_clear_distance=1.10, sensor_timeout=2.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.rid = self.p['robot_id']
        with open(self.p['config_file']) as f:
            config = yaml.safe_load(f)['warehouse']
        self.spawn = config['robot_spawns'][self.rid]
        self.graph = WarehouseGraph(config)
        self.task_priority = int(config['stage4_peer_metadata'][config['robot_visuals'][self.rid]['label']]['priority'])
        self.pickup = self.destination = ''
        self.accepted_tasks = set()
        # Missions are authored in the warehouse drawing's coordinates, but the
        # controller deliberately operates only in its own odometry frame.  Do
        # this fixed, start-pose conversion once; no map pose or peer data is
        # consumed at runtime.
        sx, sy, syaw = self.spawn
        self.route = []
        self.lookahead_progress = 0.0
        self.route_cumulative = []
        self.active_route_version = 0
        self.dashboard_tasks = config.get('dashboard_tasks', {})
        self.set_world_route(config['stage3_missions'][self.rid])
        self.active_task = 'NONE'
        self.enabled = self.p['enabled']
        self.pose = None
        self.scan = None
        self.odom_time = self.scan_time = 0.0
        self.index = 0
        self.state = 'DISABLED'
        self.side = 1
        self.obstacle_side = None
        self.turn_start = 0.0
        self.avoid_start = None
        self.coordination_command = {'state': 'NONE'}
        self.coordination_received = 0.0
        self.coordination_holding = False
        self.peer_states = {}
        self.applied_reroutes = set()
        self.mission_kind = 'TASK'
        self.completed_at = None
        self.parking_target = ''
        self.parking_state = 'FREE'
        self.parking_mode = ''
        self.route_version = 0
        self.route_metadata = {}
        self.last_linear_command = 0.0
        self.last_target = None
        self.last_heading_error = 0.0
        self.aligning = False
        self.align_episodes = 0
        self.live_obstacle = {'active':False}
        self.obstacle_seen_at = None
        self.obstacle_event_sent = False
        self.obstacle_reroutes = set()
        self.trace_at = 0.0
        self.sim_battery={'amr_alpha':92.,'amr_bravo':87.,'amr_charlie':95.,
                          'amr_delta':81.,'amr_echo':89.}.get(self.rid,90.)
        self.battery_tick=time.monotonic()
        self.move_aside_cooldown = 0.0
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status = self.create_publisher(String, 'local_status', 10)
        self.mission_pub = self.create_publisher(String, '/fleet/mission_routes', 10)
        self.event_pub = self.create_publisher(String, '/fleet/coordination_events', 20)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.lidar, qos_profile_sensor_data)
        self.create_subscription(String, 'coordination_command', self.coordination, 10)
        self.create_subscription(String, '/fleet/task_assignment', self.task_assignment, 10)
        self.create_subscription(String, '/fleet/peer_state', self.peer_state, 20)
        self.create_subscription(String, '/fleet/live_obstacle', self.live_obstacle_update, 10)
        self.create_service(SetBool, 'autonomy_enabled', self.enable)
        self.create_timer(0.1, self.tick)

    def odom(self, m):
        px, py = m.pose.pose.position.x, m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.pose = (px, py,
                     math.atan2(2*(q.w*q.z+q.x*q.y),
                                1-2*(q.y*q.y+q.z*q.z)))
        self.odom_time = time.monotonic()

    def lidar(self, m):
        values = sectors(m)
        self.scan = values
        if all(v is not None for v in values.values()):
            self.scan_time = time.monotonic()

    def coordination(self, msg):
        try:
            command = json.loads(msg.data)
            if isinstance(command, dict) and command.get('state'):
                self.coordination_command = command
                self.coordination_received = time.monotonic()
                if command.get('state') == 'REROUTE' and command.get('route'):
                    cid=command.get('conflict_id','')
                    if cid not in self.applied_reroutes:
                        self.applied_reroutes.add(cid)
                        self.route_version += 1
                        self.install_new_route(command['route'],self.route_version)
                        self.route_metadata={
                            'planner':'A*', 'raw_astar_nodes':command.get('raw_astar_nodes',[]),
                            'raw_waypoints':command.get('raw_waypoints',command['route']),
                            'expanded_nodes':command.get('expanded_nodes',0),
                            'raw_route_cost':command.get('raw_route_cost'),
                        }
                        self.change('REROUTING')
                        self.emit('REROUTE_STARTED', conflict_id=cid,
                                  zone=command.get('zone'), reason=command.get('reason'),
                                  route_length_m=round(self.graph.polyline_length(self.world_route),2),
                                  extra_detour_m=command.get('extra_detour_m'),
                                  route_version=self.route_version)
                        self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
                            task_id=self.active_task,route=self.world_route,
                            route_version=self.route_version,reroute=True))))
        except (ValueError, TypeError):
            return

    def peer_state(self, msg):
        try:
            data=json.loads(msg.data)
            if data.get('robot_id') != self.graph_robot_id:
                self.peer_states[data['robot_id']]=data
        except (ValueError,TypeError,KeyError):
            return

    def live_obstacle_update(self, msg):
        """Runtime crate metadata; physical reaction remains LaserScan-driven."""
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.live_obstacle=data
                self.obstacle_seen_at=None; self.obstacle_event_sent=False
                if not data.get('active'):
                    self.emit('LIVE_OBSTACLE_CLEARED',preset=data.get('preset','CLEAR'))
        except (ValueError,TypeError):
            return

    def world_pose(self):
        x,y,_=self.pose
        if self.p['odom_coordinates']=='world': return (x,y)
        sx,sy,syaw=self.spawn
        return (sx+math.cos(syaw)*x-math.sin(syaw)*y,
                sy+math.sin(syaw)*x+math.cos(syaw)*y)

    def persistent_obstacle_replan(self):
        """One bounded A* replan from live odometry for a real blocked crate."""
        if (not self.p['enable_obstacle_replan'] or
                not self.live_obstacle.get('persistent') or not self.destination):
            return False
        key=self.live_obstacle.get('id','live_obstacle')
        if key in self.obstacle_reroutes: return False
        blocked_zone=self.live_obstacle.get('blocked_zone','intersection_B')
        try:
            route,extra=self.graph.reroute(self.world_pose(),self.world_route[self.index+1:],
                                            self.destination,blocked_zone)
        except ValueError:
            self.change('BLOCKED_WAIT'); self.command(0,0)
            self.emit('PERSISTENT_BLOCKAGE_NO_ROUTE',preset=self.live_obstacle.get('preset'),
                      blocked_zone=blocked_zone)
            return True
        self.obstacle_reroutes.add(key)
        self.route_version+=1
        self.install_new_route(route,self.route_version)
        self.route_metadata.update(planner='A*',raw_waypoints=route)
        self.emit('PERSISTENT_BLOCKAGE_DETECTED',preset=self.live_obstacle.get('preset'),
                  blocked_zone=blocked_zone)
        self.emit('ASTAR_REPLANNING_FROM_CURRENT_POSITION',destination=self.destination,
                  blocked_zone=blocked_zone)
        self.emit('ALTERNATE_ROUTE_ACTIVE',blocked_zone=blocked_zone,
                  extra_detour_m=round(extra,2),route_version=self.route_version)
        self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
            task_id=self.active_task,route=self.world_route,route_version=self.route_version,
            reroute=True,obstacle=True))))
        return True

    @property
    def graph_robot_id(self):
        return self.rid.replace('amr_','').upper()

    def emit(self,event,**fields):
        self.event_pub.publish(String(data=json.dumps(dict(event=event,
            robot_id=self.rid,timestamp=time.time(),**fields),separators=(',',':'))))

    def set_world_route(self, points):
        self.world_route = [list(p) for p in points]
        if self.p['odom_coordinates'] == 'world':
            self.route = [(float(gx), float(gy)) for gx, gy in points]
        else:
            sx, sy, syaw = self.spawn
            self.route = []
            for gx, gy in points:
                dx, dy = gx-sx, gy-sy
                self.route.append((math.cos(syaw)*dx + math.sin(syaw)*dy,
                                   -math.sin(syaw)*dx + math.cos(syaw)*dy))
        self.route_cumulative=[0.0]
        for a,b in zip(self.route,self.route[1:]):
            self.route_cumulative.append(self.route_cumulative[-1]+math.dist(a,b))
        self.lookahead_progress=0.0

    def install_new_route(self, points, route_version):
        """Install only a genuine new route; state transitions preserve progress."""
        normalized=[list(p) for p in points]
        if (self.active_route_version == route_version and
                normalized == getattr(self, 'world_route', None)):
            return False
        self.set_world_route(normalized)
        self.index=0
        self.active_route_version=route_version
        return True

    def lookahead_target(self):
        """Return a monotonic target projected only onto the active segment.

        Searching all future segments is ambiguous on an out-and-back route:
        overlapping corridor segments can make the nearest projection jump to
        the return leg.  The route index is authoritative, so projection stays
        on the segment currently being traversed.
        """
        x,y,_=self.pose
        first=max(0,min(self.index-1,len(self.route)-2))
        nearest_progress=self.route_cumulative[first]
        a,b=self.route[first],self.route[first+1]
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if length > 1e-9:
            fraction=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
            nearest_progress=self.route_cumulative[first]+fraction*length
        ahead=float(self.p['lookahead_distance'])
        # Preserve a warehouse corner and approach it with a shorter target.
        # Once the route index advances, normal 0.55 m look-ahead resumes.
        if 0 < self.index < len(self.route)-1:
            prev=self.route[self.index-1]; corner=self.route[self.index]
            nxt=self.route[self.index+1]
            incoming=math.atan2(corner[1]-prev[1],corner[0]-prev[0])
            outgoing=math.atan2(nxt[1]-corner[1],nxt[0]-corner[0])
            turn=wrap(outgoing-incoming)
            if abs(turn)>math.pi/4:
                ahead=0.35
        # A target only advances for a fixed route version; it cannot jump
        # behind the AMR when odometry jitters or it turns in place.
        target_progress=max(self.lookahead_progress,
                            nearest_progress+ahead)
        self.lookahead_progress=min(target_progress,self.route_cumulative[-1])
        for i in range(len(self.route)-1):
            if self.route_cumulative[i+1] >= self.lookahead_progress:
                length=self.route_cumulative[i+1]-self.route_cumulative[i]
                fraction=0. if length<1e-9 else (self.lookahead_progress-self.route_cumulative[i])/length
                return (self.route[i][0]+fraction*(self.route[i+1][0]-self.route[i][0]),
                        self.route[i][1]+fraction*(self.route[i+1][1]-self.route[i][1]))
        return self.route[-1]

    def desired_speed(self, error):
        """Adaptive profile for clear corridors, controlled zones and bends."""
        zone=self.coordination_command.get('zone','')
        if zone=='narrow_aisle_1': base=0.28
        elif zone.startswith('intersection_') and self.coordination_command.get('state')=='PROCEED': base=0.35
        elif self.mission_kind=='CHARGING':
            # Only the final dock is slowed; normal warehouse cruise is untouched.
            target=self.graph.nodes.get(self.parking_target)
            base=0.18 if target and math.dist(self.world_pose(),target)<0.80 else 0.28
        elif self.mission_kind=='PARKING': base=0.42
        else: base=0.50
        magnitude=abs(error)
        if magnitude < .10: return min(base,float(self.p['max_linear_speed']))
        if magnitude < .30: return min(base*.84,float(self.p['max_linear_speed']))
        if magnitude < .55: return min(base*.67,float(self.p['max_linear_speed']))
        return 0.0

    def task_assignment(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('robot_id') != self.rid or data.get('task_id') in self.accepted_tasks:
                return
            if self.pose is None or time.monotonic()-self.odom_time > 2.:
                return
            points = data.get('route')
            if points is None:
                if data.get('task') not in self.dashboard_tasks: return
                points = self.dashboard_tasks[data['task']]
            if not points or any(len(p)!=2 or not all(math.isfinite(float(v)) for v in p) for p in points):
                return
            self.route_version += 1
            self.install_new_route(points,self.route_version)
            self.route_metadata={k:data.get(k) for k in ('planner','raw_astar_nodes',
                'raw_waypoints','expanded_nodes','raw_route_cost')}
            self.accepted_tasks.add(data.get('task_id'))
            self.task_priority = int(data.get('priority',self.task_priority))
            self.pickup, self.destination = data.get('pickup',''), data.get('destination','')
            self.active_task = data.get('task_id', data['task'])
            self.index = 0
            self.enabled = True
            self.mission_kind = 'TASK'; self.completed_at=None
            self.parking_target=''; self.parking_state='FREE'
            self.coordination_holding = False
            self.obstacle_side = None
            self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
                task_id=self.active_task,route=self.world_route,pickup=self.pickup,
                destination=self.destination,priority=self.task_priority,
                route_version=self.route_version))))
            self.emit('ASTAR_ROUTE_PLANNED',task_id=self.active_task,pickup=self.pickup,
                      destination=self.destination,route_length_m=round(
                      self.graph.polyline_length(self.world_route),2),
                      expanded_nodes=self.route_metadata.get('expanded_nodes',0),
                      raw_astar_nodes=len(self.route_metadata.get('raw_astar_nodes') or []),
                      smoothed_waypoints=len(self.world_route),
                      raw_route_cost=self.route_metadata.get('raw_route_cost'),
                      route_version=self.route_version)
            self.change('WAYPOINT_TRACK')
            self.get_logger().info(f'[{self.rid}] DASHBOARD_TASK_ASSIGNED {self.active_task}')
        except (ValueError, TypeError, KeyError):
            return

    def enable(self, req, res):
        self.enabled = req.data
        self.command(0, 0)
        self.change('WAYPOINT_TRACK' if req.data else 'DISABLED')
        res.success = True
        res.message = f'{self.rid} enabled={self.enabled}'
        return res

    def change(self, state):
        if state != self.state:
            self.get_logger().info(f'[{self.rid}] {self.state} -> {state} waypoint={self.index+1}')
            self.state = state

    def command(self, v, w):
        # Safety/yield stops remain immediate.  Moving commands use a modest
        # acceleration cap so the verified physical platform is not jolted.
        if v > 0.0:
            max_step=float(self.p['linear_acceleration'])*0.1
            v=min(v,self.last_linear_command+max_step)
        self.last_linear_command=float(v)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.rid+'/base_footprint'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.pub.publish(m)
        self.status.publish(String(data=json.dumps(dict(state=self.state, waypoint=self.index,
                            pose=self.pose, scan=self.scan, linear=v, angular=w,
                            active_task=self.active_task,
                            route=self.world_route, remaining_route=self.world_route[self.index:],
                            task_priority=self.task_priority, pickup=self.pickup, destination=self.destination,
                            task_progress=(100.0 if self.index >= len(self.route) else
                                100.0*self.index/max(1, len(self.route))),
                            coordination_state=self.coordination_command.get('state', 'NONE'),
                            coordination_hold=self.coordination_holding,
                            route_version=self.route_version,
                            planner=self.route_metadata.get('planner','A*'),
                            raw_astar_nodes=self.route_metadata.get('raw_astar_nodes',[]),
                            raw_waypoints=self.route_metadata.get('raw_waypoints',[]),
                            expanded_nodes=self.route_metadata.get('expanded_nodes',0),
                            raw_route_cost=self.route_metadata.get('raw_route_cost'),
                            lookahead_target=self.last_target,
                            heading_error=self.last_heading_error,
                            align_mode=self.aligning,
                            align_episodes=self.align_episodes,
                            mission_kind=self.mission_kind,
                            parking_target=self.parking_target,
                            parking_state=self.parking_state,
                            battery_pct=round(self.sim_battery,1),
                            battery_simulated=True))))

    def should_hold_for_coordination(self):
        """Yield only at a pre-zone boundary; PROCEED never bypasses LiDAR."""
        if self.p['diagnostic_mode']:
            return False
        state = self.coordination_command.get('state', 'NONE')
        if state == 'SAFE_WAIT':
            return True
        if state not in ('YIELD', 'YIELD_AND_WAIT', 'WAIT_FOR_CLEAR'):
            return False
        zone = self.coordination_command.get('zone_geometry', {})
        if not zone or self.pose is None:
            return True  # malformed local command fails safe
        sx, sy, syaw = self.spawn
        px, py, _ = self.pose
        if self.p['odom_coordinates'] == 'world':
            wx, wy = px, py
        else:
            wx = sx + math.cos(syaw)*px - math.sin(syaw)*py
            wy = sy + math.sin(syaw)*px + math.cos(syaw)*py
        if self.coordination_command.get('zone') == 'narrow_aisle_1':
            # Reach the lateral waiting pocket before applying the aisle gate.
            target=self.world_route[min(self.index,len(self.world_route)-1)]
            holds=[self.graph.nodes[n] for n in ('AISLE_SOUTH_HOLD','AISLE_NORTH_HOLD')]
            if any(math.dist(point,p)<0.05 for point in self.world_route[self.index:] for p in holds):
                return False
            return True
        margin = float(self.coordination_command.get('hold_margin', 0.40))
        if 'radius' in zone:
            return math.hypot(wx-zone['center'][0], wy-zone['center'][1]) <= float(zone['radius']) + margin
        bounds = zone.get('bounds', {})
        return (bounds.get('x', [float('inf'), -float('inf')])[0]-margin <= wx <= bounds.get('x', [-float('inf'), float('inf')])[1]+margin and
                bounds.get('y', [float('inf'), -float('inf')])[0]-margin <= wy <= bounds.get('y', [-float('inf'), float('inf')])[1]+margin)

    def active_peer_routes(self):
        inactive=('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','UNKNOWN')
        return [p.get('remaining_route',[]) for p in self.peer_states.values()
                if p.get('local_nav_state') not in inactive and p.get('remaining_route')]

    def start_parking(self, mode='POST_TASK_REPOSITION', obstruction=None):
        if self.pose is None: return False
        reserved={p.get('parking_target') for p in self.peer_states.values()
                  if p.get('parking_state') in ('RESERVED','OCCUPIED')}
        exclude={self.parking_target} if mode=='MOVE_ASIDE' else set()
        candidates=(self.graph.charging_candidates(self.world_pose(),reserved,self.active_peer_routes())
                    if mode=='POST_TASK_REPOSITION' else
                    self.graph.parking_candidates(self.world_pose(),reserved,self.active_peer_routes(),exclude))
        if not candidates: return False
        _,target,route=candidates[0]
        self.route_version += 1
        self.install_new_route(route,self.route_version); self.enabled=True
        self.mission_kind='CHARGING' if mode=='POST_TASK_REPOSITION' else 'PARKING'; self.active_task='NONE'; self.task_priority=0
        self.pickup=''; self.destination=target
        self.parking_target=target; self.parking_state='RESERVED'
        self.parking_mode=mode
        self.coordination_holding=False; self.completed_at=None
        self.change('RETURNING_TO_CHARGE' if mode=='POST_TASK_REPOSITION' else mode)
        self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
            task_id='PARKING',route=self.world_route,destination=target,priority=0,
            route_version=self.route_version))))
        self.emit('MOVE_ASIDE_STARTED' if mode=='MOVE_ASIDE' else 'CHARGING_BAY_RESERVED',
                  parking_target=target, obstruction=obstruction)
        if mode=='POST_TASK_REPOSITION':
            self.emit('ASTAR_POST_TASK_ROUTE',parking_target=target,
                      route_version=self.route_version)
        return True

    def idle_obstruction(self):
        if self.pose is None or time.monotonic()<self.move_aside_cooldown: return None
        if self.state not in ('PARKED','DISABLED','MISSION_COMPLETE','AVAILABLE'): return None
        for peer in self.peer_states.values():
            # A disabled controller still publishes its configured demo route.
            # It is not traffic and must never displace a parked AMR.
            if peer.get('active_task') in (None,'NONE','PARKING'):
                continue
            if peer.get('local_nav_state') in ('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','UNKNOWN'):
                continue
            route=peer.get('remaining_route') or []
            if len(route)>1 and any(self.graph.point_segment_distance(self.pose[:2],a,b)<0.82
                                    for a,b in zip(route,route[1:])):
                return peer.get('robot_id')
        return None

    def parking_conflict(self):
        if self.mission_kind!='PARKING' or not self.parking_target: return False
        mine=self.graph_robot_id
        return any(p.get('parking_target')==self.parking_target and
                   p.get('parking_state') in ('RESERVED','OCCUPIED') and
                   (p.get('parking_state')=='OCCUPIED' or p.get('robot_id','')<mine)
                   for p in self.peer_states.values())

    def tick(self):
        now = time.monotonic()
        elapsed=now-self.battery_tick
        if elapsed >= 1.0:
            self.battery_tick=now
            # Presentation-only battery simulation: charge rises visibly while
            # parked in a real charging bay; no physical telemetry is claimed.
            if self.state=='CHARGING': self.sim_battery=min(100.,self.sim_battery+elapsed*1.0)
            elif self.enabled: self.sim_battery=max(20.,self.sim_battery-elapsed*0.004)
        if not self.p['diagnostic_mode']:
            obstruction=self.idle_obstruction()
            if obstruction and self.start_parking('MOVE_ASIDE',obstruction):
                self.move_aside_cooldown=now+5.0
            if self.parking_conflict():
                old=self.parking_target
                self.parking_target=''
                if self.start_parking(self.parking_mode or 'POST_TASK_REPOSITION'):
                    self.emit('PARKING_RESELECTED',released=old,
                              parking_target=self.parking_target)
        if not self.enabled:
            if self.state == 'PARKED': self.command(0,0); return
            self.change('DISABLED'); self.command(0, 0); return
        if self.pose is None or now-min(self.odom_time, self.scan_time) > self.p['sensor_timeout']:
            self.change('SENSOR_STALE_SAFE_STOP'); self.command(0, 0); return
        if self.index >= len(self.route):
            if self.mission_kind in ('PARKING','CHARGING'):
                completed_mode=self.parking_mode
                self.enabled=False; self.parking_state='OCCUPIED'
                self.change('CHARGING' if self.mission_kind=='CHARGING' else 'PARKED'); self.command(0,0)
                self.emit('MOVE_ASIDE_COMPLETE' if completed_mode=='MOVE_ASIDE' else 'CHARGING_STARTED',
                          parking_target=self.parking_target)
                return
            if self.completed_at is None:
                self.completed_at=now; self.change('MISSION_COMPLETE')
                self.emit('TASK_COMPLETED',task_id=self.active_task,destination=self.destination)
            self.command(0,0)
            if not self.p['diagnostic_mode'] and now-self.completed_at>=1.25:
                self.change('CLEARING_DROP_ZONE')
                self.emit('DROP_ZONE_CLEARING',destination=self.destination)
                self.start_parking('POST_TASK_REPOSITION')
            return
        x, y, yaw = self.pose
        front, left, right = (self.scan[k] for k in ('front', 'left', 'right'))
        requested_hold = self.should_hold_for_coordination()
        # All moving branches (including obstacle avoidance) obey a peer hold.
        if requested_hold:
            self.coordination_holding = True
            self.change('COORDINATION_HOLD'); self.command(0,0); return
        if self.coordination_holding:
            self.coordination_holding=False
            self.change('WAYPOINT_TRACK')
        if self.state == 'AVOID_TURN':
            # Commit to a clear side before trying to re-acquire the original
            # waypoint.  A shallow turn / short advance can otherwise make a
            # circular re-detection around a wide static obstacle.
            if front > self.p['obstacle_clear_distance'] and abs(wrap(yaw-self.turn_start)) > 1.15:
                self.avoid_start = (x, y)
                self.change('AVOID_FORWARD')
                self.emit('LOCAL_EDGE_AVOIDANCE_ACTIVE',side='LEFT' if self.side==1 else 'RIGHT')
            else:
                self.command(0, self.side*0.60); return
        if self.state == 'AVOID_FORWARD':
            if front < 0.40:
                self.change('OBSTACLE_STOP'); self.command(0, 0); return
            if math.hypot(x-self.avoid_start[0], y-self.avoid_start[1]) < 1.15:
                self.command(0.12, 0); return
            self.change('REACQUIRE_WAYPOINT')
            self.emit('ORIGINAL_ASTAR_ROUTE_REACQUIRED',route_version=self.route_version)
        if self.state == 'OBSTACLE_STOP':
            # Pick the safer side from the first scan and retain it while
            # clearing this waypoint's obstacle.  Re-picking on each scan can
            # make a robot alternate around the two faces of one crate.
            if self.obstacle_side is None:
                self.obstacle_side = 1 if left >= right else -1
            self.side = self.obstacle_side
            self.turn_start = yaw
            self.get_logger().info(f'AVOID_DIRECTION={"LEFT" if self.side==1 else "RIGHT"} front={front:.3f} left={left:.3f} right={right:.3f}')
            self.change('AVOID_TURN'); self.command(0, self.side*0.60); return
        gx, gy = self.route[self.index]
        distance = math.hypot(gx-x, gy-y)
        while self.index < len(self.route)-1 and distance < self.p['goal_tolerance']:
            self.get_logger().info(f'WAYPOINT_REACHED {self.index+1}')
            self.index += 1
            self.obstacle_side = None
            gx,gy=self.route[self.index]; distance=math.hypot(gx-x,gy-y)
        # Only the real terminal station completes a mission.  Intermediate
        # A* nodes are deliberately pass-through points.
        if self.index == len(self.route)-1 and distance < self.p['goal_tolerance']:
            self.get_logger().info(f'WAYPOINT_REACHED {self.index+1} (terminal)')
            self.index += 1
            self.command(0, 0)
            return
        if self.mission_kind=='CHARGING' and self.index == len(self.route)-1:
            self.change('DOCKING')
        tx,ty=self.lookahead_target()
        error = wrap(math.atan2(ty-y, tx-x)-yaw)
        self.last_target=(round(tx,3),round(ty,3))
        self.last_heading_error=round(error,4)
        if self.p['controller_trace'] and now >= self.trace_at:
            self.trace_at=now+0.5
            self.get_logger().info(
                'FOLLOW_TRACE route_version=%s index=%s pose=(%.3f,%.3f,%.3f) '
                'lookahead=(%.3f,%.3f) error=%.3f state=%s cmd_linear=%.3f '
                'coord=%s obstacle=%s replan_requested=%s' %
                (self.route_version,self.index,x,y,yaw,tx,ty,error,self.state,
                 self.last_linear_command,self.coordination_command.get('state','NONE'),
                 self.live_obstacle.get('active',False),self.p['enable_obstacle_replan']))
        requested_hold = self.should_hold_for_coordination()
        gate = motion_gate(front, self.p['obstacle_stop_distance'], requested_hold)
        # At 0.50 m/s, start reacting long before the hard-stop threshold.
        # The crate's *position* is never trusted: LaserScan is the evidence.
        if front < self.p['obstacle_prepare_distance']:
            self.obstacle_seen_at=self.obstacle_seen_at or now
            if not self.obstacle_event_sent:
                self.obstacle_event_sent=True
                self.emit('LIDAR_OBSTACLE_DETECTED',distance_m=round(front,3),
                          preset=self.live_obstacle.get('preset','UNKNOWN'))
            if now-self.obstacle_seen_at >= 1.5 and self.persistent_obstacle_replan():
                return
        else:
            self.obstacle_seen_at=None; self.obstacle_event_sent=False
        # Physical sensor safety is intentionally evaluated before the fleet
        # decision, including when the local command says PROCEED.
        if gate == 'LIDAR_STOP' and abs(error) < 0.55:
            self.get_logger().info(f'LIDAR_OBSTACLE front={front:.3f} left={left:.3f} right={right:.3f}')
            self.change('OBSTACLE_STOP'); self.command(0, 0); return
        if gate == 'COORDINATION_HOLD':
            if not self.coordination_holding:
                self.get_logger().info(f'[{self.rid}] HOLDING before {self.coordination_command.get("zone", "controlled zone")}')
            self.coordination_holding = True
            self.change('COORDINATION_HOLD'); self.command(0, 0); return
        if self.coordination_holding:
            self.coordination_holding = False
            if self.coordination_command.get('state') == 'RESUME':
                self.get_logger().info(f'[{self.rid}] COORDINATION RESUME')
        # Hysteretic alignment prevents the rotate/forward/rotate thrash that
        # appeared when every minor waypoint correction was treated as a turn.
        if not self.aligning and abs(error) > 0.55:
            self.aligning=True; self.align_episodes += 1
            self.change('ALIGN')
        elif self.aligning and abs(error) < 0.20:
            self.aligning=False
            self.change('WAYPOINT_TRACK')
        if self.aligning:
            self.change('ALIGN')
            v=0.0 if abs(error)>0.85 else 0.12
        else:
            self.change('WAYPOINT_TRACK')
            v=self.desired_speed(error)
        w=0.0 if abs(error)<0.05 else max(-self.p['max_angular_speed'], min(self.p['max_angular_speed'],1.25*error))
        self.command(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = LocalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.command(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
