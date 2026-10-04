"""Local odometry/LaserScan autonomy with a local, non-motion fleet hold input."""
import json
import math
import os
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
from edge_ai_nav.fleet.performance_profile import PerformanceProfile
from edge_ai_nav.fleet.route_graph import WarehouseGraph


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_rate(error, gain, limit):
    """Proportional angular correction for any signed target angle."""
    return 0.0 if abs(error)<0.05 else max(-limit,min(limit,gain*error))


def passed_waypoint(pose, previous, target, lateral_tolerance=0.45):
    """Advance a nonterminal waypoint once its inbound segment is traversed.

    A waypoint need not be hit to centimetre precision during a smooth arc;
    lateral error is still bounded, and the caller checks next-edge visibility.
    """
    dx, dy = target[0]-previous[0], target[1]-previous[1]
    length2 = dx*dx+dy*dy
    if length2 < 1e-12:
        return False
    along = ((pose[0]-previous[0])*dx+(pose[1]-previous[1])*dy)/length2
    lateral = abs((pose[0]-previous[0])*dy-(pose[1]-previous[1])*dx)/math.sqrt(length2)
    return along >= 0.88 and lateral <= lateral_tolerance


def sectors(scan):
    buckets = {k: [] for k in ('front', 'left', 'right')}
    clear_beams = {k: False for k in buckets}
    for i, distance in enumerate(scan.ranges):
        a = wrap(scan.angle_min + i * scan.angle_increment)
        names=[]
        if abs(a) < 0.38:
            names.append('front')
        if 0.38 <= a <= 1.5:
            names.append('left')
        if -1.5 <= a <= -0.38:
            names.append('right')
        for name in names:
            if math.isinf(distance) and distance > 0:
                clear_beams[name] = True
            elif math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                buckets[name].append(distance)
    # Third-smallest return rejects up to two isolated noise beams.
    return {k: (sorted(v)[min(2, len(v)-1)] if v else
                scan.range_max if clear_beams[k] else None)
            for k, v in buckets.items()}


class LocalController(Node):
    def __init__(self):
        super().__init__('local_waypoint_controller')
        defaults = dict(robot_id='amr_alpha', config_file='', enabled=False,
                        odom_coordinates='local',
                        max_linear_speed=0.50, max_angular_speed=0.50,
                        lookahead_distance=0.55, linear_acceleration=0.65,
                        smooth_steering=False,
                        controller_trace=False, enable_obstacle_replan=False,
                        diagnostic_mode=False,
                        battery_low_threshold=30.0,
                        battery_critical_threshold=15.0,
                        battery_drain_per_meter=0.08,
                        battery_charge_per_second=0.25,
                        charging_slot_tolerance=0.22,
                        performance_profile_dir='~/.ros/swarmx_profiles',
                        goal_tolerance=0.16, obstacle_stop_distance=0.85,
                        waypoint_tolerance=0.16,
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
        self.logical_id = config['robot_visuals'][self.rid]['label']
        self.task_priority = int(config['stage4_peer_metadata'][self.logical_id]['priority'])
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
        self.demo_hold = False
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
        self.sim_battery = float(config.get('battery_initial_percent', {}).get(self.rid, 90.))
        self.battery_tick=time.monotonic()
        self.battery_last_pose=None
        self.battery_full_emitted=False
        self.docking_event_sent=False
        self.idle_charge_retry_at=0.0
        self.move_aside_cooldown = 0.0
        profile_dir=os.path.expanduser(str(self.p['performance_profile_dir']))
        self.profile_path=os.path.join(profile_dir,self.rid+'.json')
        self.performance=PerformanceProfile.load(self.rid,self.profile_path)
        self.mission_metrics=None
        self.metric_last_pose=None
        self.metric_tick=time.monotonic()
        self.metric_last_error=0.0
        self.metric_last_moving=False
        self.action_events=set()
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
        if self.battery_last_pose is not None and self.state != 'CHARGING':
            travelled=math.dist(self.battery_last_pose,(px,py))
            # Ignore spawn/reset discontinuities; drain only for physical travel.
            if travelled < 1.0:
                self.sim_battery=max(0.,self.sim_battery-
                    travelled*float(self.p['battery_drain_per_meter']))
        self.battery_last_pose=(px,py)
        if self.mission_metrics is not None and self.metric_last_pose is not None:
            step=math.dist(self.metric_last_pose,(px,py))
            if step < 1.0:
                self.mission_metrics['actual_distance_m'] += step
        self.metric_last_pose=(px,py)
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
                action=command.get('state')
                cid=command.get('conflict_id','')
                if (action=='RESUME' and command.get('resume_route') and
                        command.get('resume_token')!=getattr(self,'last_yield_resume',None)):
                    self.last_yield_resume=command['resume_token']
                    self.route_version+=1
                    self.install_new_route(command['resume_route'],self.route_version)
                    self.coordination_holding=False
                    self.change('WAYPOINT_TRACK')
                    self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
                        task_id=self.active_task,route=self.world_route,
                        route_version=self.route_version))))
                    self.emit('YIELD_MISSION_REJOIN',conflict_id=cid,
                              route_version=self.route_version,destination=self.destination)
                if (cid and action in ('YIELD','YIELD_AND_WAIT','WAIT_FOR_CLEAR','REROUTE','MOVE_ASIDE')
                        and cid not in self.action_events):
                    detected_ns=int(command.get('detection_monotonic_ns',0) or 0)
                    action_latency=(time.perf_counter_ns()-detected_ns)/1e6 if detected_ns else None
                    self.action_events.add(cid)
                    self.emit('NEGOTIATION_ACTION_APPLIED',conflict_id=cid,
                              action=action,action_latency_ms=round(action_latency,3)
                              if action_latency is not None else None,
                              decision_latency_ms=command.get('decision_latency_ms'),
                              winner=command.get('winner'),loser=command.get('loser'),
                              decision_basis=command.get('decision_basis'))
                    if self.mission_metrics is not None and action_latency is not None:
                        self.mission_metrics['conflict_resolution_delay_ms'] += action_latency
                if (action=='MOVE_ASIDE' and cid and cid not in self.applied_reroutes and
                        self.active_task in ('', 'NONE', None)):
                    self.applied_reroutes.add(cid)
                    self.start_parking('MOVE_ASIDE',command.get('winner'))
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
                        if self.mission_metrics is not None:
                            self.mission_metrics['reroute_count'] += 1
                            self.mission_metrics['distance_rerouted_m'] += float(
                                command.get('extra_detour_m') or 0.0)
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
        return self.logical_id

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
        if not self.route:
            return None
        if len(self.route) == 1 or self.index >= len(self.route):
            return self.route[-1]
        x,y,_=self.pose
        first=max(0,min(self.index-1,len(self.route)-2))
        nearest_progress=self.route_cumulative[first]
        a,b=self.route[first],self.route[first+1]
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if length > 1e-9:
            fraction=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
            nearest_progress=self.route_cumulative[first]+fraction*length
        ahead=float(self.p['lookahead_distance'])*self.performance.lookahead_factor
        if self.p.get('smooth_steering', False):
            speed=max(0.0,float(getattr(self,'last_linear_command',0.0)))
            ahead=min(1.20,ahead+0.35*speed/max(0.1,float(self.p['max_linear_speed'])))
        # Preserve a warehouse corner and approach it with a shorter target.
        # Once the route index advances, normal 0.55 m look-ahead resumes.
        if 0 < self.index < len(self.route)-1:
            prev=self.route[self.index-1]; corner=self.route[self.index]
            nxt=self.route[self.index+1]
            incoming=math.atan2(corner[1]-prev[1],corner[0]-prev[0])
            outgoing=math.atan2(nxt[1]-corner[1],nxt[0]-corner[0])
            turn=wrap(outgoing-incoming)
            if abs(turn)>math.pi/4:
                ahead=min(ahead,0.55 if self.p.get('smooth_steering',False) else 0.35)
        # A target only advances for a fixed route version; it cannot jump
        # behind the AMR when odometry jitters or it turns in place.
        target_progress=max(self.lookahead_progress,
                            nearest_progress+ahead)
        # A charger is reached through its own perpendicular entry.  Do not
        # cut a lookahead arc past either docking-area bend: that can carry a
        # robot north of the cross aisle and into an occupied bay's brake zone.
        for corner in (self.index, self.index+1):
            if LocalController.charging_bend(self,corner):
                target_progress=min(target_progress,self.route_cumulative[corner])
                break
        self.lookahead_progress=min(target_progress,self.route_cumulative[-1])
        for i in range(len(self.route)-1):
            if self.route_cumulative[i+1] >= self.lookahead_progress:
                length=self.route_cumulative[i+1]-self.route_cumulative[i]
                fraction=0. if length<1e-9 else (self.lookahead_progress-self.route_cumulative[i])/length
                return (self.route[i][0]+fraction*(self.route[i+1][0]-self.route[i][0]),
                        self.route[i][1]+fraction*(self.route[i+1][1]-self.route[i][1]))
        return self.route[-1]

    def charging_bend(self, index):
        graph=getattr(self,'graph',None)
        if (graph is None or not graph.forward_rejoin or
                not 1 <= index < len(self.route)-1):
            return False
        docking=(getattr(self,'destination','') in graph.charging and
                 index >= max(1,len(self.route)-3))
        departure=(index==1 and any(
            math.dist(self.route[index],point)<.03
            for name,point in getattr(graph,'nodes',{}).items()
            if name.endswith('_APPROACH') and
            name.removesuffix('_APPROACH') in graph.charging))
        if not (docking or departure):
            return False
        previous,corner,next_point=self.route[index-1:index+2]
        inbound=math.atan2(corner[1]-previous[1],corner[0]-previous[0])
        outbound=math.atan2(next_point[1]-corner[1],next_point[0]-corner[0])
        return abs(wrap(outbound-inbound)) > math.pi/4

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
        else: base=(max(0.50,float(self.p['max_linear_speed']))
                    if self.p.get('smooth_steering',False) else 0.50)
        base*=self.performance.speed_factor
        magnitude=abs(error)
        if self.p.get('smooth_steering', False):
            # Continuous curvature/heading slowdown; only near-reversal
            # errors require in-place alignment.
            if magnitude > 1.7:
                return 0.0
            return min(base*max(0.18,1.0-magnitude/1.6),
                       float(self.p['max_linear_speed']))
        if magnitude < .10: return min(base,float(self.p['max_linear_speed']))
        if magnitude < .30: return min(base*.84,float(self.p['max_linear_speed']))
        if magnitude < .55: return min(base*.67,float(self.p['max_linear_speed']))
        return 0.0

    def start_mission_metrics(self):
        self.mission_metrics=dict(
            task_id=self.active_task,
            planned_route_distance_m=self.graph.polyline_length(self.world_route),
            actual_distance_m=0.0,
            started_monotonic=time.monotonic(),
            unnecessary_stop_count=0,
            align_episodes=0,
            align_time_s=0.0,
            turn_overshoot_rad=0.0,
            heading_correction_count=0,
            heading_error_sum=0.0,
            heading_error_samples=0,
            obstacle_stop_time_s=0.0,
            negotiation_wait_time_s=0.0,
            reroute_count=0,
            distance_rerouted_m=0.0,
            moving_speed_sum=0.0,
            moving_speed_samples=0,
            conflict_resolution_delay_ms=0.0)
        self.metric_tick=time.monotonic()
        self.metric_last_pose=self.pose[:2] if self.pose else None
        self.metric_last_error=0.0
        self.metric_last_moving=False

    def update_mission_metrics(self, now):
        if self.mission_metrics is None:
            self.metric_tick=now
            return
        elapsed=max(0.0,min(0.5,now-self.metric_tick)); self.metric_tick=now
        metric=self.mission_metrics
        error=abs(float(self.last_heading_error))
        metric['heading_error_sum']+=error
        metric['heading_error_samples']+=1
        if self.aligning:
            metric['align_time_s']+=elapsed
        if self.state in ('OBSTACLE_STOP','AVOID_TURN','AVOID_FORWARD'):
            metric['obstacle_stop_time_s']+=elapsed
        if self.coordination_holding or self.state=='COORDINATION_HOLD':
            metric['negotiation_wait_time_s']+=elapsed
        moving=self.last_linear_command>0.02
        if moving:
            metric['moving_speed_sum']+=self.last_linear_command
            metric['moving_speed_samples']+=1
        elif self.metric_last_moving and self.state not in (
                'ALIGN','OBSTACLE_STOP','COORDINATION_HOLD','MISSION_COMPLETE'):
            metric['unnecessary_stop_count']+=1
        signed=float(self.last_heading_error)
        if (abs(signed)>0.08 and abs(self.metric_last_error)>0.08 and
                signed*self.metric_last_error<0):
            metric['heading_correction_count']+=1
            metric['turn_overshoot_rad']=max(metric['turn_overshoot_rad'],abs(signed))
        self.metric_last_error=signed
        self.metric_last_moving=moving

    def finish_mission_metrics(self):
        if self.mission_metrics is None:
            return None
        raw=self.mission_metrics; actual=max(raw['actual_distance_m'],1e-6)
        samples=max(1,raw['heading_error_samples'])
        speed_samples=max(1,raw['moving_speed_samples'])
        metrics={
            'task_id':raw['task_id'],
            'planned_route_distance_m':round(raw['planned_route_distance_m'],3),
            'actual_distance_m':round(raw['actual_distance_m'],3),
            'route_efficiency':round(min(1.0,raw['planned_route_distance_m']/actual),4),
            'completion_time_s':round(time.monotonic()-raw['started_monotonic'],3),
            'unnecessary_stop_count':raw['unnecessary_stop_count'],
            'align_episodes':self.align_episodes,
            'total_align_time_s':round(raw['align_time_s'],3),
            'turn_overshoot_rad':round(raw['turn_overshoot_rad'],4),
            'heading_correction_count':raw['heading_correction_count'],
            'average_heading_error_rad':round(raw['heading_error_sum']/samples,4),
            'obstacle_stop_time_s':round(raw['obstacle_stop_time_s'],3),
            'negotiation_wait_time_s':round(raw['negotiation_wait_time_s'],3),
            'reroute_count':raw['reroute_count'],
            'distance_rerouted_m':round(raw['distance_rerouted_m'],3),
            'average_moving_speed_mps':round(raw['moving_speed_sum']/speed_samples,3),
            'conflict_resolution_delay_ms':round(raw['conflict_resolution_delay_ms'],3),
            'task_success':True,
        }
        self.performance.record(metrics)
        try:
            self.performance.save(self.profile_path)
        except OSError as error:
            self.get_logger().warning(f'Performance profile not persisted: {error}')
        self.mission_metrics=None
        self.emit('MISSION_PERFORMANCE',metrics=metrics,
                  profile=self.performance.dictionary())
        return metrics

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
            released_slot=self.parking_target if self.parking_state in ('RESERVED','OCCUPIED') else ''
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
            # Used only by the deterministic video helper to hold at a
            # graph-defined staging node.  Normal assignments omit it.
            self.demo_hold=bool(data.get('demo_hold',False))
            self.parking_target=''; self.parking_state='FREE'
            self.parking_mode=''; self.docking_event_sent=False
            self.coordination_holding = False
            self.obstacle_side = None
            self.align_episodes=0
            self.start_mission_metrics()
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
            if released_slot:
                self.emit('CHARGE_SLOT_RELEASED',parking_target=released_slot,
                          task_id=self.active_task)
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
        battery_state=self.battery_state()
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
                            battery_percent=round(self.sim_battery,1),
                            battery_state=battery_state,
                            is_charging=self.state=='CHARGING',
                            battery_simulated=True,
                            performance_profile=self.performance.dictionary(),
                            route_efficiency=(round(self.performance.average_route_efficiency*100,1)
                                if self.performance.missions_completed else None)))))

    def battery_state(self):
        if self.state == 'CHARGING': return 'CHARGING'
        if self.sim_battery < float(self.p['battery_critical_threshold']): return 'CRITICAL'
        if self.sim_battery <= float(self.p['battery_low_threshold']): return 'LOW'
        return 'NORMAL'

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
        inactive=('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','DEMO_STAGED','UNKNOWN')
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
        if not candidates:
            return self.start_safe_wait() if mode=='POST_TASK_REPOSITION' else False
        _,target,route=candidates[0]
        self.route_version += 1
        self.install_new_route(route,self.route_version); self.enabled=True
        self.mission_kind='CHARGING' if mode=='POST_TASK_REPOSITION' else 'PARKING'; self.active_task='NONE'; self.task_priority=0
        self.pickup=''; self.destination=target
        self.parking_target=target; self.parking_state='RESERVED'
        self.parking_mode=mode
        self.docking_event_sent=False
        self.coordination_holding=False; self.completed_at=None
        self.change('RETURNING_TO_CHARGE' if mode=='POST_TASK_REPOSITION' else mode)
        self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
            task_id='PARKING',route=self.world_route,destination=target,priority=0,
            route_version=self.route_version))))
        self.emit('MOVE_ASIDE_STARTED' if mode=='MOVE_ASIDE' else 'CHARGE_SLOT_RESERVED',
                  parking_target=target, obstruction=obstruction)
        if mode=='POST_TASK_REPOSITION':
            self.emit('ASTAR_RETURN_ROUTE',parking_target=target,
                      route_version=self.route_version)
        return True

    def start_safe_wait(self):
        """Leave an operational endpoint when every charging slot is busy."""
        reserved={p.get('parking_target') for p in self.peer_states.values()
                  if p.get('parking_state') in ('RESERVED','OCCUPIED')}
        candidates=self.graph.parking_candidates(
            self.world_pose(),reserved,self.active_peer_routes(),{self.parking_target})
        if not candidates: return False
        _,target,route=candidates[0]
        self.route_version+=1
        self.install_new_route(route,self.route_version); self.enabled=True
        self.mission_kind='PARKING'; self.active_task='NONE'; self.task_priority=0
        self.pickup=''; self.destination=target
        self.parking_target=target; self.parking_state='RESERVED'
        self.parking_mode='SAFE_WAIT_FOR_CHARGE'; self.completed_at=None
        self.coordination_holding=False; self.docking_event_sent=False
        self.change('SAFE_WAIT_FOR_CHARGE')
        self.mission_pub.publish(String(data=json.dumps(dict(robot_id=self.rid,
            task_id='SAFE_WAIT',route=self.world_route,destination=target,priority=0,
            route_version=self.route_version))))
        self.emit('SAFE_WAIT_ROUTE',parking_target=target,
                  route_version=self.route_version)
        return True

    def at_charging_slot(self):
        if self.pose is None: return ''
        position=self.world_pose()
        return next((name for name in self.graph.charging
            if math.dist(position,self.graph.nodes[name]) <=
               float(self.p['charging_slot_tolerance'])), '')

    def idle_obstruction(self):
        if self.pose is None or time.monotonic()<self.move_aside_cooldown: return None
        if self.state not in ('PARKED','DISABLED','MISSION_COMPLETE','AVAILABLE'): return None
        for peer in self.peer_states.values():
            # A disabled controller still publishes its configured demo route.
            # It is not traffic and must never displace a parked AMR.
            if peer.get('active_task') in (None,'NONE','PARKING'):
                continue
            if peer.get('local_nav_state') in ('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','DEMO_STAGED','UNKNOWN'):
                continue
            route=peer.get('remaining_route') or []
            if len(route)>1 and any(self.graph.point_segment_distance(self.pose[:2],a,b)<0.82
                                    for a,b in zip(route,route[1:])):
                return peer.get('robot_id')
        return None

    def parking_conflict(self):
        if (self.mission_kind not in ('PARKING','CHARGING') or
                self.parking_state!='RESERVED' or not self.parking_target): return False
        mine=self.graph_robot_id
        return any(p.get('parking_target')==self.parking_target and
                   p.get('parking_state') in ('RESERVED','OCCUPIED') and
                   (p.get('parking_state')=='OCCUPIED' or p.get('robot_id','')<mine)
                   for p in self.peer_states.values())

    def tick(self):
        now = time.monotonic()
        self.update_mission_metrics(now)
        elapsed=now-self.battery_tick
        if elapsed >= 1.0:
            self.battery_tick=now
            # Presentation-only battery simulation: charge rises visibly while
            # parked in a real charging bay; no physical telemetry is claimed.
            if self.state=='CHARGING':
                before=self.sim_battery
                self.sim_battery=min(100.,self.sim_battery+
                    elapsed*float(self.p['battery_charge_per_second']))
                if before < 100. <= self.sim_battery and not self.battery_full_emitted:
                    self.battery_full_emitted=True
                    self.emit('BATTERY_FULL',battery_percent=100.0)
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
        if (not self.p['diagnostic_mode'] and not self.enabled and
                self.active_task in ('', 'NONE', None) and self.state!='CHARGING' and
                self.pose is not None and self.scan is not None and
                now-min(self.odom_time,self.scan_time) <= self.p['sensor_timeout'] and
                now >= self.idle_charge_retry_at):
            slot=self.at_charging_slot()
            if slot:
                self.mission_kind='CHARGING'; self.parking_target=slot
                self.parking_state='OCCUPIED'; self.change('CHARGING')
                self.battery_full_emitted=self.sim_battery>=100.
                self.emit('CHARGING_STARTED',parking_target=slot,
                          battery_percent=round(self.sim_battery,1))
            elif not self.start_parking('POST_TASK_REPOSITION'):
                self.idle_charge_retry_at=now+1.0
        if not self.enabled:
            if self.state in ('PARKED','CHARGING','AVAILABLE','DEMO_STAGED'):
                self.command(0,0); return
            self.change('DISABLED'); self.command(0, 0); return
        if self.pose is None or now-min(self.odom_time, self.scan_time) > self.p['sensor_timeout']:
            self.change('SENSOR_STALE_SAFE_STOP'); self.command(0, 0); return
        if self.index >= len(self.route):
            if self.demo_hold:
                self.enabled=False
                self.change('DEMO_STAGED'); self.command(0,0)
                return
            if self.mission_kind in ('PARKING','CHARGING'):
                completed_mode=self.parking_mode
                self.enabled=False; self.parking_state='OCCUPIED'
                self.change('CHARGING' if self.mission_kind=='CHARGING' else 'PARKED'); self.command(0,0)
                self.emit('MOVE_ASIDE_COMPLETE' if completed_mode=='MOVE_ASIDE' else 'CHARGING_STARTED',
                          parking_target=self.parking_target,
                          battery_percent=round(self.sim_battery,1))
                return
            if self.completed_at is None:
                self.completed_at=now; self.change('MISSION_COMPLETE')
                self.finish_mission_metrics()
                self.emit('TASK_COMPLETED',task_id=self.active_task,destination=self.destination)
            self.command(0,0)
            if not self.p['diagnostic_mode'] and now-self.completed_at>=0.10:
                self.change('CLEARING_DROP_ZONE')
                self.emit('DROP_ZONE_CLEARING',destination=self.destination)
                self.start_parking('POST_TASK_REPOSITION')
            return
        x, y, yaw = self.pose
        front, left, right = (self.scan[k] for k in ('front', 'left', 'right'))
        if any(value is None for value in (front,left,right)):
            self.change('SENSOR_STALE_SAFE_STOP'); self.command(0,0); return
        if self.coordination_command.get('state')=='YIELD_RELOCATE':
            target=self.coordination_command.get('target')
            if (now-self.coordination_received>1.0 or not target or
                    not self.graph.visible(self.world_pose(),target)):
                self.change('COORDINATION_HOLD'); self.command(0,0); return
            wx,wy=self.world_pose()
            distance=math.hypot(target[0]-wx,target[1]-wy)
            if distance<.18:
                self.change('YIELD_POCKET'); self.command(0,0); return
            # Pocket recovery is V2/world-odom only. Keep the original route,
            # index and destination untouched so normal following can resume.
            error=wrap(math.atan2(target[1]-wy,target[0]-wx)-yaw)
            speed=0.22 if abs(error)<.25 and front>self.p['obstacle_stop_distance'] else 0.
            self.change('YIELD_RELOCATING')
            self.command(speed,heading_rate(error,1.25,float(self.p['max_angular_speed'])))
            return
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
        while self.index < len(self.route)-1 and (
                distance < (0.25 if self.charging_bend(self.index)
                            else self.p['waypoint_tolerance']) or
                (self.p['smooth_steering'] and self.index > 0 and
                 not self.charging_bend(self.index) and
                 passed_waypoint((x,y),self.route[self.index-1],self.route[self.index],
                                 float(self.p['waypoint_tolerance'])) and
                 self.graph.visible(self.world_pose(),self.world_route[self.index+1]))):
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
            if not self.docking_event_sent:
                self.docking_event_sent=True
                self.emit('DOCKING',parking_target=self.parking_target)
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
        docking_turn=self.charging_bend(self.index-1)
        enter_align = (0.9 if docking_turn else 1.7) if self.p['smooth_steering'] else 0.55
        exit_align = (0.25 if docking_turn else 0.45) if self.p['smooth_steering'] else 0.20
        if not self.aligning and abs(error) > enter_align:
            self.aligning=True; self.align_episodes += 1
            self.change('ALIGN')
        elif self.aligning and abs(error) < exit_align:
            self.aligning=False
            self.change('WAYPOINT_TRACK')
        if self.aligning:
            self.change('ALIGN')
            v=(0.0 if docking_turn or abs(error)>1.8 else 0.10) if self.p['smooth_steering'] else (
                0.0 if abs(error)>0.85 else 0.12)
        else:
            self.change('WAYPOINT_TRACK')
            v=self.desired_speed(error)
        if self.charging_bend(self.index) and distance < 2.0:
            v=min(v,0.15)
        if self.graph.forward_rejoin and self.index == len(self.route)-1:
            # V2's larger robot must settle on the terminal pad instead of
            # crossing it at cruise speed and repeatedly turning near a wall.
            v=min(v,0.20 if distance>=1.0 else 0.12)
            if distance<1.2 and abs(error)>0.8:
                v=0.0
        steering_gain=1.25*self.performance.steering_factor
        w=heading_rate(error,steering_gain,float(self.p['max_angular_speed']))
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
