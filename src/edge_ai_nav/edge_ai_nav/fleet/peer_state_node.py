"""DDS peer exchange plus local Stage 6 negotiation; never publishes cmd_vel."""
import json
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .zone_prediction import LocalDetector, load_zones
from .negotiation import (NegotiationBook, conflict_identity, coordination_action,
                          outside_with_margin, reservation_distance,
                          physically_committed)
from .route_graph import WarehouseGraph
from .path_safety import approaching_crossing, should_yield_for_path, yield_pocket


class PeerState(Node):
    def __init__(self):
        super().__init__('peer_state')
        for name, value in {'robot_id':'ALPHA', 'config_file':'', 'publish_rate_hz':5.0,
                            'peer_stale_warning':1.0, 'peer_remove_timeout':2.0,
                            'conflict_detection':False, 'negotiation':False,
                            'apply_spawn_transform':True}.items():
            self.declare_parameter(name, value)
        p = {n:self.get_parameter(n).value for n in ('robot_id','config_file','publish_rate_hz','peer_stale_warning','peer_remove_timeout')}
        self.robot_id, self.p = p['robot_id'], p
        with open(p['config_file']) as stream:
            self.cfg = yaml.safe_load(stream)['warehouse']
        self.large_platform=bool(self.cfg.get('scale_speed_upgrade',{}).get('experimental',False))
        self.physical_diameter=float(self.cfg.get('amr_footprint_diameter_m',1.18))
        self.peer_clearance=float(self.cfg.get('stage6',{}).get('physical_stop_distance_m',2.0))
        self.meta = self.cfg['stage4_peer_metadata'][self.robot_id]
        self.graph = WarehouseGraph(self.cfg)
        self.zone_request = None
        self.spawn = self.cfg['robot_spawns'][self.meta['namespace'].lstrip('/')]
        self.negotiation_enabled = self.get_parameter('negotiation').value
        self.path_guard_enabled=bool(self.cfg.get('stage6',{}).get('path_crossing_guard',False))
        self.path_guard_active=''
        self.physical_guard_peer=''
        self.yield_relocation=None
        self.detector = None
        if self.get_parameter('conflict_detection').value or self.negotiation_enabled:
            prediction = self.cfg['stage5']['prediction']
            self.detector = LocalDetector(load_zones(self.cfg), **prediction)
        self.odom_received = self.status_received = 0.0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pose = self.twist = None
        self.status = {}
        self.wait_started = None
        self.wait_credit = 0.0
        self.peers, self.peer_states, self.offline_peers, self.stale_peers = {}, {}, set(), set()
        self.last_active_count = -1
        stage6 = self.cfg.get('stage6', {})
        self.declare_parameter('allow_reroute',bool(stage6.get('allow_reroute',True)))
        self.policy = dict(waiting_cap=float(stage6.get('waiting_cap', 30.0)),
                           waiting_quantum=float(stage6.get('waiting_quantum', 1.0)),
                           eta_quantum=float(stage6.get('eta_quantum', 0.25)),
                           priority_first=bool(stage6.get('priority_first', False)),
                           allow_reroute=bool(self.get_parameter('allow_reroute').value))
        self.hold_margin = max(0.55,float(stage6.get('hold_margin', 0.40)))
        self.exit_margin = max(0.55,float(stage6.get('exit_margin', 0.40)))
        self.maximum_reroute_detour = float(stage6.get('maximum_reroute_detour_m', 2.5))
        self.reservation_policy=dict(
            safe_deceleration=float(stage6.get('safe_deceleration',0.65)),
            processing_margin=float(stage6.get('processing_margin',0.15)),
            footprint_margin=float(stage6.get('footprint_margin',0.45)),
            safety_margin=float(stage6.get('safety_margin',0.55)),
            minimum_distance=float(stage6.get('minimum_negotiation_distance',1.5)),
            maximum_distance=float(stage6.get('maximum_negotiation_distance',2.0)))
        self.book = NegotiationBook()
        self.pending_resumes = []
        self.cleared_conflicts = set()
        self.pub = self.create_publisher(String, '/fleet/peer_state', qos)
        self.diag = self.create_publisher(String, 'peer_diagnostics', 10)
        self.conflicts = self.create_publisher(String, 'conflict_status', 10) if self.detector else None
        self.coordination = self.create_publisher(String, 'coordination_command', 10) if self.negotiation_enabled else None
        self.event_pub = self.create_publisher(String, '/fleet/coordination_events', 20)
        decision_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.decision_pub = self.create_publisher(
            String, '/fleet/negotiation_decision', decision_qos)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, 'local_status', self.local_status, 10)
        self.create_subscription(String, '/fleet/peer_state', self.peer_message, qos)
        self.create_subscription(String, '/fleet/negotiation_decision',
                                 self.decision_message, decision_qos)
        self.create_timer(1.0 / float(p['publish_rate_hz']), self.publish)
        self.create_timer(0.25, self.audit_peers)

    def odom(self, msg):
        self.pose, self.twist = msg.pose.pose, msg.twist.twist
        self.odom_received = time.monotonic()

    def local_status(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.status = data
                self.status_received = time.monotonic()
        except (ValueError, TypeError): pass

    def publish(self):
        if self.pose is None: return
        q, pos = self.pose.orientation, self.pose.position
        math = __import__('math')
        local_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        sx, sy, spawn_yaw = self.spawn
        # Stage 4 used local isolated odometry. Stage 6A's Gazebo physical
        # OdometryPublisher already emits warehouse/world coordinates, so
        # applying this transform there would move every peer twice.
        if self.get_parameter('apply_spawn_transform').value:
            x = sx + math.cos(spawn_yaw)*pos.x - math.sin(spawn_yaw)*pos.y
            y = sy + math.sin(spawn_yaw)*pos.x + math.cos(spawn_yaw)*pos.y
            yaw = math.atan2(math.sin(spawn_yaw + local_yaw), math.cos(spawn_yaw + local_yaw))
        else:
            x, y, yaw = pos.x, pos.y, local_yaw
        index = int(self.status.get('waypoint', 0))
        route = self.status.get('route') or self.cfg['stage3_missions'][self.meta['namespace'].lstrip('/')]
        remaining = self.status.get('remaining_route') or route[index:]
        goal = route[min(index, len(route)-1)]
        nav_state = self.status.get('state', 'UNKNOWN')
        fresh = time.monotonic() - min(self.odom_received, self.status_received) <= 2.0
        if not fresh: nav_state = 'SENSOR_STALE_SAFE_STOP'
        intent = self.graph.intent((x,y),remaining)
        next_zone = intent.get('zone','')
        if nav_state in ('DISABLED', 'MISSION_COMPLETE', 'DEMO_STAGED', 'UNKNOWN',
                         'PARKED', 'AVAILABLE', 'CHARGING', 'SENSOR_STALE_SAFE_STOP'):
            # A stationary robot physically occupying a controlled zone still
            # blocks entry even when its mission is disabled or complete.
            occupied = next((name for name,z in self.graph.zones.items()
                if not outside_with_margin({'x':x,'y':y},z,0.)), '')
            next_zone = occupied
            intent = ({'zone':occupied,'inside':True,'distance':0.,'eta':0.,
                       'window':[0.,1e6]} if occupied else {})
        if nav_state in ('OBSTACLE_STOP', 'SENSOR_STALE_SAFE_STOP') or self.status.get('coordination_hold', False):
            self.wait_started = self.wait_started or time.monotonic()
        else:
            if self.wait_started is not None:
                self.wait_credit=min(self.policy['waiting_cap'],self.wait_credit+time.monotonic()-self.wait_started)
            self.wait_started = None
        waiting_time = min(self.policy['waiting_cap'],self.wait_credit+
            (0.0 if self.wait_started is None else time.monotonic()-self.wait_started))
        idle = nav_state in ('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','CHARGING','DEMO_STAGED','UNKNOWN')
        priority = 0 if idle else max(1,min(4,int(self.status.get('task_priority',2))))
        battery_state=self.status.get('battery_state','NORMAL')
        charging_urgency=(2 if battery_state=='CRITICAL' else
                          1 if battery_state=='LOW' else 0)
        progress=float(self.status.get('task_progress',0.0))
        window=intent.get('window') or [0.,float('inf')]
        clearance_time=max(0.,float(window[1])-float(window[0]))
        request_key=(self.status.get('active_task','NONE'),next_zone)
        if not self.zone_request or tuple(self.zone_request['key']) != request_key:
            self.zone_request=dict(key=request_key,task_priority=priority,
                                   waiting_time=waiting_time,eta=intent.get('eta',0.),
                                   charging_urgency=charging_urgency,
                                   mission_progress=progress,
                                   clearance_time_s=clearance_time)
        adopted_owner=next((item['winner'] for item in self.book.values()
            if item['conflict_id'].split(':',1)[0]==next_zone), '')
        data = dict(robot_id=self.robot_id, timestamp=self.get_clock().now().nanoseconds/1e9,
                    x=x, y=y, yaw=yaw,
                    linear_velocity=self.twist.linear.x if self.twist else 0.0,
                    angular_velocity=self.twist.angular.z if self.twist else 0.0,
                    goal_x=goal[0], goal_y=goal[1], next_zone=next_zone,
                    task_priority=priority, waiting_time=waiting_time,
                    charging_urgency=charging_urgency, mission_progress=progress,
                    clearance_time_s=clearance_time,
                    zone_intent=intent, zone_request=self.zone_request, eta=intent.get('eta'),
                    local_nav_state=nav_state, localization_mode='ODOM',
                    localization_confidence=1.0, coordination_state='NONE',
                    zone_committed=bool(intent.get('inside')),
                    # Reservation is an output of an adopted shared decision,
                    # never a speculative input generated by local arbitration.
                    zone_reserved=adopted_owner==self.robot_id,
                    reservation_owner=adopted_owner,
                    safety_emergency=nav_state in ('SENSOR_STALE_SAFE_STOP','EMERGENCY_STOP'),
                    active_task=self.status.get('active_task','NONE'), route=route,
                    remaining_route=remaining, destination=self.status.get('destination',''),
                    parking_target=self.status.get('parking_target',''),
                    parking_state=self.status.get('parking_state','FREE'))
        resume=getattr(self,'yield_resume',None)
        if resume:
            peer=self.peer_states.get(resume['peer'])
            if (peer and data['active_task']==resume['task'] and
                    math.hypot(data['x']-peer['x'],data['y']-peer['y'])<=4.0):
                data['yielding_to']=resume['peer']
            else:
                self.yield_resume=None
        if self.detector:
            now = time.monotonic()
            unavailable = self.offline_peers | {rid for rid, seen in self.peers.items()
                if now-seen > float(self.p['peer_stale_warning'])}
            current = [v for rid, v in self.peer_states.items() if rid not in unavailable]
            for event, record in self.detector.update(data, current, data['timestamp'], unavailable):
                self.get_logger().info(f'[{self.robot_id}] {event} ' + json.dumps(record, separators=(',', ':')))
                if event == 'CONFLICT_PREDICTED' and self.robot_id < record['peer']:
                    self.emit('CONFLICT_DETECTED',event_id=record['conflict_id'],
                              conflict_id=record['conflict_id'],zone=record['zone'],
                              robots=sorted((self.robot_id,record['peer'])),
                              reason='ETA overlap detected',my_eta=record['my_eta'],
                              peer_eta=record['peer_eta'])
            if self.negotiation_enabled:
                command = self.negotiate(data, unavailable)
                if self.path_guard_enabled:
                    guard=self.path_guard(data,unavailable)
                    if guard:
                        command=guard
                    guard=self.proximity_guard(data,unavailable)
                    if guard:
                        command=guard
                    guard=self.physical_guard(data,unavailable)
                    if guard:
                        command=guard
                    command=self.relocation_command(data,unavailable,command)
                data['coordination_state'] = command['state']
                if command.get('conflict_id'):
                    data['active_conflict_id'] = command['conflict_id']
                    data['negotiation_winner'] = command['winner']
                self.coordination.publish(String(data=json.dumps(command, separators=(',', ':'))))
            elif self.detector.active:
                data['coordination_state'] = 'CONFLICT_PREDICTED'
            self.conflicts.publish(String(data=json.dumps(dict(robot_id=self.robot_id,
                active_conflicts=len(self.detector.active), conflicts=list(self.detector.active.values()),
                negotiations=list(self.book.values()) if self.negotiation_enabled else [],
                event_count=self.detector.event_count))))
        self.pub.publish(String(data=json.dumps(data, separators=(',', ':'))))

    def emit(self,event,**fields):
        self.event_pub.publish(String(data=json.dumps(dict(event=event,
            robot_id=self.robot_id,timestamp=time.time(),**fields),separators=(',',':'))))

    def conflict_costs(self, states, zone):
        """Graph-derived wait/reroute costs shared by both peer computations."""
        costs={}
        for state in states:
            rid=state['robot_id']; other=next(s for s in states if s is not state)
            other_window=other.get('zone_intent',{}).get('window',[0.,0.])
            info={'wait':max(0.,float(other_window[1])),
                  'reroute':float('inf'),'route':None}
            destination=state.get('destination'); remaining=state.get('remaining_route') or []
            if destination in self.graph.nodes and remaining:
                try:
                    planner=(self.graph.reroute_forward if self.graph.forward_rejoin
                             else self.graph.reroute)
                    route,extra=planner((state['x'],state['y']),remaining,destination,zone)
                    if self.graph.forward_rejoin:
                        winner_route=[[other['x'],other['y']]]+(other.get('remaining_route') or [])
                        clearance=self.graph.route_clearance(route,winner_route)
                        if clearance < max(float(self.cfg.get('amr_footprint_diameter_m',1.18)),
                                           float(self.cfg.get('stage6',{}).get('reroute_clearance_m',2.35))):
                            info.update(reroute=float('inf'),route=None,
                                        rejection='UNSAFE_DOWNSTREAM_MERGE',
                                        route_clearance_m=round(clearance,3))
                            costs[rid]=info
                            continue
                    # This is a decision estimate, matching the controller's
                    # clear-corridor cruise profile rather than a physics knob.
                    reroute=extra/0.50
                    if extra > self.maximum_reroute_detour:
                        info.update(reroute=float('inf'),route=None,
                                    extra_detour_m=extra)
                        costs[rid]=info
                        continue
                    new_intent=self.graph.intent((state['x'],state['y']),route[1:])
                    others=[p for p in self.peer_states.values()
                            if p.get('robot_id') not in (states[0]['robot_id'],states[1]['robot_id'])]
                    if any(p.get('next_zone')==new_intent.get('zone') for p in others):
                        reroute+=8.0
                    info.update(reroute=reroute,route=route,extra_detour_m=extra)
                except ValueError:
                    pass
            costs[rid]=info
        return costs

    def path_guard(self, own, unavailable):
        """V2-only local safety overlay for route crossings outside named zones."""
        candidates=[]
        for rid,peer in self.peer_states.items():
            # A completed lateral yield occupies space, but is no longer
            # approaching a crossing along its saved mission route. Physical
            # occupancy still protects the stationary body independently.
            if (peer.get('local_nav_state')=='YIELD_POCKET' or
                    peer.get('yielding_to')==self.robot_id):
                continue
            if rid in unavailable or any(rid in (item['winner'],item['loser'])
                                         for item in self.book.values()):
                continue
            if not should_yield_for_path(own,peer):
                continue
            crossing=approaching_crossing(own,peer)
            if crossing:
                candidates.append((crossing['own_distance'],rid,crossing))
        if not candidates:
            if self.path_guard_active:
                self.emit('PATH_CROSSING_GUARD_CLEARED',
                          event_id=self.path_guard_active+':cleared',
                          conflict_id=self.path_guard_active)
                self.path_guard_active=''
            return None
        _,peer_id,crossing=min(candidates)
        cid='PATH_CROSS:'+':'.join(sorted((self.robot_id,peer_id)))
        if cid!=self.path_guard_active:
            self.path_guard_active=cid
            self.emit('PATH_CROSSING_SAFE_WAIT',event_id=cid+':hold',
                      conflict_id=cid,winner=peer_id,loser=self.robot_id,
                      crossing=list(crossing['point']),
                      distance_m=round(crossing['own_distance'],2))
        return dict(state='SAFE_WAIT',zone='PATH_CROSSING',conflict_id=cid,
                    winner=peer_id,loser=self.robot_id,
                    reason='UNZONED_PATH_CROSSING',
                    crossing=list(crossing['point']),
                    distance_m=round(crossing['own_distance'],2))

    def physical_guard(self, own, unavailable):
        """Brake even a negotiated winner when a peer occupies its safety gap.

        Yielding does not make the loser's physical body disappear. This V2
        emergency hold includes charging/idle robots and return trips. It can
        leave a blocked pair waiting until its path is cleared; it must never
        grant the winner permission to drive through the stopped peer.
        """
        stop=float(self.cfg['stage6'].get('physical_stop_distance_m',2.0))
        candidates=[]
        for rid,peer in self.peer_states.items():
            if rid in unavailable:
                continue
            distance=math.hypot(float(own['x'])-float(peer['x']),
                                float(own['y'])-float(peer['y']))
            # Hysteresis retains the brake while both robots are stopped.
            threshold=stop+(0.20 if self.physical_guard_peer==rid else 0.0)
            if distance<threshold:
                candidates.append((distance,rid))
        if not candidates:
            self.physical_guard_peer=''
            return None
        distance,rid=min(candidates)
        self.physical_guard_peer=rid
        decision=next((item for item in self.book.values()
            if {item['winner'],item['loser']}=={self.robot_id,rid}),None)
        return dict(state='SAFE_WAIT',zone='PHYSICAL_OCCUPANCY',
                    conflict_id='PHYSICAL:'+':'.join(sorted((self.robot_id,rid))),
                    winner=decision['winner'] if decision else '',
                    loser=decision['loser'] if decision else '',
                    reason='PEER_FOOTPRINT_BLOCKED',distance_m=round(distance,2))

    def relocation_command(self, own, unavailable, command):
        """Let only the loser move to a checked pocket during an emergency hold."""
        peers=[p for rid,p in self.peer_states.items() if rid not in unavailable]
        plan=self.yield_relocation
        if plan:
            peer=self.peer_states.get(plan['peer'])
            if not peer or plan['peer'] in unavailable:
                return dict(state='SAFE_WAIT',reason='YIELD_POCKET_PEER_STALE')
            distance=math.hypot(own['x']-peer['x'],own['y']-peer['y'])
            active=any(item['loser']==self.robot_id and item['winner']==plan['peer']
                       for item in self.book.values())
            position=(own['x'],own['y'])
            next_target=next((tuple(point) for point in own.get('remaining_route') or []
                              if math.dist(position,point)>.2),None)
            resume_path=[position]
            if next_target is not None and not self.graph.visible(position,next_target):
                # A lateral pocket can sit across a shelf corner from the
                # next waypoint. Return along its checked escape leg first.
                resume_path.append(tuple(plan.get('origin',position)))
            if next_target is not None:
                resume_path.append(next_target)
            peer_path=[(peer['x'],peer['y'])]
            if peer.get('local_nav_state') not in (
                    'CHARGING','PARKED','AVAILABLE','DEMO_STAGED','DISABLED','MISSION_COMPLETE'):
                peer_path.extend(peer.get('remaining_route') or [])
            resume_clearance=(self.graph.route_clearance(resume_path,peer_path)
                              if next_target is not None and len(peer_path)>1 else
                              min(self.graph.point_segment_distance(peer_path[0],a,b)
                                  for a,b in zip(resume_path,resume_path[1:]))
                              if next_target is not None else 0.)
            safe_resume=(next_target is not None and
                         all(self.graph.visible(a,b) for a,b in zip(resume_path,resume_path[1:])) and
                         resume_clearance>=2.2)
            if (self.status.get('state')=='YIELD_POCKET' and not active and
                    distance>2.2 and safe_resume):
                if len(resume_path)>2 and math.dist(position,resume_path[1])>.18:
                    plan['target']=list(resume_path[1])
                else:
                    self.emit('YIELD_POCKET_CLEARED',event_id=plan['id']+':cleared',
                              conflict_id=plan['id'])
                    self.yield_relocation=None
                    self.yield_resume=dict(peer=plan['peer'],task=own.get('active_task','NONE'))
                    remaining=own.get('remaining_route') or []
                    next_index=next(i for i,p in enumerate(remaining)
                                    if math.dist(p,next_target)<.03)
                    return dict(command,state='RESUME',conflict_id=plan['id'],
                        winner=plan['peer'],loser=self.robot_id,
                        resume_token=f"{plan['id']}:{self.status.get('route_version',0)}",
                        resume_route=[list(p) for p in resume_path]+remaining[next_index+1:])
            target=plan['target']
            # Recheck the complete escape leg against every fresh peer. A new
            # entrant may invalidate a pocket selected in an earlier callback.
            if (not self.graph.visible(position,target) or any(
                    self.graph.point_segment_distance((p['x'],p['y']),position,target)<1.55
                    for p in peers)):
                return dict(state='SAFE_WAIT',reason='YIELD_POCKET_PATH_BLOCKED')
            return dict(state='YIELD_RELOCATE',zone='YIELD_POCKET',target=target,
                        conflict_id=plan['id'],winner=plan['peer'],loser=self.robot_id)
        early_yield=command.get('state') in ('YIELD','YIELD_AND_WAIT','WAIT_FOR_CLEAR')
        if command.get('reason')!='PEER_FOOTPRINT_BLOCKED' and not early_yield:
            return command
        rid=command.get('winner') if early_yield else self.physical_guard_peer
        peer=self.peer_states.get(rid)
        if not peer or own.get('local_nav_state') in (
                'CHARGING','DISABLED','PARKED','AVAILABLE','DEMO_STAGED','MISSION_COMPLETE'):
            return command
        decision=next((item for item in self.book.values()
            if {item['winner'],item['loser']}=={self.robot_id,rid}),None)
        loser=(decision['loser'] if decision else
               self.robot_id if should_yield_for_path(own,peer) else rid)
        if loser!=self.robot_id:
            return command
        if early_yield:
            winner_path=[[peer['x'],peer['y']]]+(peer.get('remaining_route') or [])
            if len(winner_path)<2 or min(self.graph.point_segment_distance(
                    (own['x'],own['y']),a,b)
                    for a,b in zip(winner_path,winner_path[1:]))>=2.35:
                return command
        target=yield_pocket(self.graph,own,peer,peers)
        if target is None:
            return command
        cid='POCKET:'+':'.join(sorted((self.robot_id,rid)))
        self.yield_relocation=dict(peer=rid,target=target,id=cid,
                                  origin=[own['x'],own['y']])
        self.emit('YIELD_POCKET_SELECTED',event_id=cid,conflict_id=cid,
                  winner=rid,loser=self.robot_id,target=target)
        return dict(state='YIELD_RELOCATE',zone='YIELD_POCKET',target=target,
                    conflict_id=cid,winner=rid,loser=self.robot_id)

    def proximity_guard(self, own, unavailable):
        """V2 last-resort brake before two active physical footprints converge.

        This never replaces the shared negotiation winner.  It holds only the
        loser while the winner passes, including when a zone prediction drops
        for a cycle as the robots enter the same crossing.
        """
        if own.get('active_task') in ('', 'NONE', None):
            return None
        candidates=[]
        for rid,peer in self.peer_states.items():
            if (peer.get('local_nav_state')=='YIELD_POCKET' or
                    peer.get('yielding_to')==self.robot_id):
                continue
            if rid in unavailable or peer.get('active_task') in ('', 'NONE', None):
                continue
            distance=math.hypot(float(own['x'])-float(peer['x']),
                                float(own['y'])-float(peer['y']))
            if distance >= 3.8:
                continue
            decision=next((item for item in self.book.values()
                if {item['winner'],item['loser']}=={self.robot_id,rid}),None)
            winner=(decision['winner'] if decision else
                    rid if should_yield_for_path(own,peer) else self.robot_id)
            if winner==rid:
                candidates.append((distance,rid,decision))
        if not candidates:
            return None
        distance,rid,decision=min(candidates)
        cid=(decision['conflict_id'] if decision else
             'PROXIMITY:'+':'.join(sorted((self.robot_id,rid))))
        return dict(state='SAFE_WAIT',zone='PROXIMITY',conflict_id=cid,
                    winner=rid,loser=self.robot_id,
                    reason='V2_PHYSICAL_SEPARATION_GUARD',
                    distance_m=round(distance,2))

    def negotiate(self, own, unavailable):
        """Calculate only this AMR's local command from DDS state and geometry."""
        now = own['timestamp']  # Same ROS system clock as detector first-detect timestamp.
        # A clear may arrive one DDS cycle before prediction windows disappear.
        # Do not immediately recreate the same encounter during that tail.
        for cid in list(self.cleared_conflicts):
            if cid not in self.detector.active:
                self.cleared_conflicts.discard(cid)
        # Only the canonical authority can release a shared encounter.  Peers
        # adopt its versioned clear record instead of clearing on a local timer.
        cleared=[]
        for old in list(self.book.values()):
            if old.get('authority_robot_id') != self.robot_id:
                continue
            winner_state=own if old['winner']==self.robot_id else self.peer_states.get(old['winner'])
            loser_state=own if old['loser']==self.robot_id else self.peer_states.get(old['loser'])
            zone=self.detector.zones[old['conflict_id'].split(':',1)[0]]
            if winner_state and old['winner'] not in unavailable:
                if not outside_with_margin(winner_state,zone,0.): old['winner_entered_zone']=True
                # V2's larger physical footprint can still be converging when
                # ETA windows briefly stop overlapping.  Keep the same owner
                # latched until the pair separates, or the owner really exits.
                separation=math.hypot(float(winner_state['x'])-float(loser_state['x']),
                                      float(winner_state['y'])-float(loser_state['y'])) \
                    if loser_state else float('inf')
                route_invalidated=(old['conflict_id'] not in self.detector.active and
                                   now-old['decision_time']>1.0 and
                                   (not self.path_guard_enabled or separation>5.0))
                if ((old['winner_entered_zone'] and outside_with_margin(winner_state,zone,self.exit_margin))
                        or route_invalidated):
                    reason=('WINNER_CLEARED' if old['winner_entered_zone'] else
                            'ROUTE_INVALIDATED')
                    clear=self.book.clear_message(old['conflict_id'],reason,now)
                    removed=self.book.adopt_clear(clear)
                    if removed:
                        cleared.append(removed)
                        self.cleared_conflicts.add(old['conflict_id'])
                        self.decision_pub.publish(String(data=json.dumps(
                            clear,separators=(',',':'))))
        for cid, record in self.detector.active.items():
            if any(c['conflict_id']==cid for c in cleared): continue
            if cid in self.cleared_conflicts: continue
            peer = self.peer_states.get(record['peer'])
            if peer is None:
                continue
            try:
                _,participants,authority=conflict_identity(cid)
            except ValueError:
                continue
            if self.robot_id not in participants:
                continue
            # A non-authority never creates a speculative local latch.  Until
            # the authority's reliable decision arrives, its command is SAFE_WAIT.
            if self.book.get(cid) is not None:
                continue
            if self.robot_id != authority:
                continue
            # Immutable request snapshots make both independent computations
            # explainable.  Only the authority evaluates them, so DDS timing can
            # no longer create contradictory permanent winner/loser latches.
            request_fields=('task_priority','waiting_time','charging_urgency',
                            'mission_progress','clearance_time_s')
            a=dict(own,**{k:own.get('zone_request',own).get(k,own.get(k))
                          for k in request_fields})
            b=dict(peer,**{k:peer.get('zone_request',peer).get(k,peer.get(k))
                           for k in request_fields})
            zone=self.detector.zones[record['zone']]
            if self.policy['priority_first']:
                for state in (a,b):
                    state['zone_committed']=physically_committed(
                        state,zone,self.reservation_policy)
                # If both are past the stopping boundary, the nearer entrant
                # is the only physically safe owner.  A tie remains governed
                # by the deterministic priority hierarchy.
                if a['zone_committed'] and b['zone_committed']:
                    ad=float(a.get('zone_intent',{}).get('distance',math.inf))
                    bd=float(b.get('zone_intent',{}).get('distance',math.inf))
                    if abs(ad-bd)>0.05:
                        (b if ad<bd else a)['zone_committed']=False
            else:
                for state in (a,b):
                    state['zone_committed']=not outside_with_margin(state,zone,0.)
            costs=self.conflict_costs((a,b),record['zone'])
            decision = self.book.create_authoritative(cid, authority, a, b,
                {self.robot_id: own.get('zone_request',{}).get('eta',record['my_eta']),
                 record['peer']: peer.get('zone_request',{}).get('eta',record['peer_eta'])},
                now, record['conflict_first_detected_time'], zone=record['zone'],
                route_costs=costs,
                detection_monotonic_ns=record.get('conflict_first_detected_monotonic_ns'),
                **self.policy)
            winner_state=a if decision['winner']==a['robot_id'] else b
            loser_state=b if winner_state is a else a
            decision['baseline_delay_s']=round(
                float(winner_state.get('zone_intent',{}).get('window',[0.,0.])[1])+
                float(loser_state.get('zone_intent',{}).get('window',[0.,0.])[1]),3)
            self.decision_pub.publish(String(data=json.dumps(decision,separators=(',',':'))))
            self.get_logger().info(
                f"[{self.robot_id}] SHARED_NEGOTIATION conflict={cid} "
                f"version={decision['decision_version']} authority={authority} "
                f"winner={decision['winner']} loser={decision['loser']} "
                f"reason={decision['reason']} latency_ms={decision['decision_latency_ms']:.3f}")
            self.emit('NEGOTIATION_COMPLETE',event_id=cid+':decision',
                      conflict_id=cid,zone=record['zone'],authority=authority,
                      decision_version=decision['decision_version'],winner=decision['winner'],
                      loser=decision['loser'],loser_action=decision['loser_action'],
                      decision_basis=decision['reason'],
                      winner_priority=int(winner_state.get('task_priority', 0)),
                      loser_priority=int(loser_state.get('task_priority', 0)),
                      decision_latency_ms=round(decision['decision_latency_ms'],3),
                      wait_cost_s=decision['details']['wait_cost'],
                      reroute_cost_s=decision['details']['reroute_cost'],
                      extra_detour_m=decision['details'].get('extra_detour_m'),
                      winner_route_preserved=True,
                      deadlock_prevented=record['zone']=='narrow_aisle_1')

        selected = next((item for item in self.book.values() if item['loser'] == self.robot_id),
                        next(iter(self.book.values()), None))
        if selected is None:
            if cleared:
                self.zone_request = None  # Next arbitration includes accrued waiting credit.
                item=cleared[0]
                pair=sorted((item['winner'],item['loser']))
                actual=max(0.,now-item['decision_time'])
                saved=max(0.,float(item.get('baseline_delay_s',0.))-actual)
                if self.robot_id==pair[0]:
                    self.emit('CONFLICT_RESOLVED',event_id=item['conflict_id']+':resolved',
                              conflict_id=item['conflict_id'],zone=item['conflict_id'].split(':',1)[0],
                              winner=item['winner'],loser=item['loser'],
                              actual_delay_s=round(actual,3),
                              baseline_delay_s=round(float(item.get('baseline_delay_s',0.)),3),
                              time_saved_s=round(saved,3))
                return dict(state='RESUME',zone=item['conflict_id'].split(':',1)[0],
                            conflict_id=item['conflict_id'],winner=item['winner'],loser=item['loser'])
            if self.pending_resumes:
                item=self.pending_resumes.pop(0)
                return dict(state='RESUME',zone=item['conflict_id'].split(':',1)[0],
                            conflict_id=item['conflict_id'],winner=item['winner'],loser=item['loser'],
                            authority_robot_id=item.get('authority_robot_id'),
                            decision_version=item.get('decision_version'))
            waiting=next(((cid,record) for cid,record in self.detector.active.items()
                          if self.book.get(cid) is None and
                          cid not in self.cleared_conflicts),None)
            if waiting:
                cid,record=waiting
                try: authority=conflict_identity(cid)[2]
                except ValueError: authority=''
                return dict(state='SAFE_WAIT',conflict_id=cid,zone=record['zone'],
                            winner='',loser='',authority_robot_id=authority,
                            reason='WAITING_FOR_AUTHORITY_DECISION',
                            zone_geometry=self.detector.zones[record['zone']])
            return {'state': 'NONE'}
        peer_id = selected['loser'] if selected['winner'] == self.robot_id else selected['winner']
        peer = self.peer_states.get(peer_id)
        zone_name = selected['conflict_id'].split(':', 1)[0]
        zone = self.detector.zones[zone_name]
        winner_state = own if selected['winner'] == self.robot_id else peer
        if winner_state is not None and not outside_with_margin(winner_state, zone, 0.0):
            selected['winner_entered_zone'] = True
        # No absence means clearance.  The yielding robot must keep waiting until
        # the known winner has entered and subsequently left with margin.
        peer_unavailable = peer_id in unavailable or peer is None
        if peer_unavailable:
            state = 'SAFE_WAIT'
        else:
            state=coordination_action(selected,self.robot_id,
                                      self.status.get('coordination_hold',False))
        if state == 'PROCEED':
            self.wait_credit=0.0
        approach_speed=max(abs(float(own.get('linear_velocity',0.))),
                           abs(float(peer.get('linear_velocity',0.))) if peer else 0.)
        dynamic_hold=reservation_distance(approach_speed,**self.reservation_policy)
        command=dict(state=state, conflict_id=selected['conflict_id'], zone=zone_name,
                    winner=selected['winner'], loser=selected['loser'],
                    hold_margin=max(self.hold_margin,dynamic_hold), exit_margin=self.exit_margin,
                    zone_geometry=zone, reason=selected['reason'],
                    decision_latency_ms=selected['decision_latency_ms'],
                    decision_basis=selected['reason'],
                    authority_robot_id=selected.get('authority_robot_id'),
                    decision_version=selected.get('decision_version'),
                    detection_monotonic_ns=selected.get('detection_monotonic_ns'),
                    reservation_distance_m=round(dynamic_hold,3))
        if state=='REROUTE':
            command['route']=selected.get('details',{}).get('reroute_route') or \
                selected.get('details',{}).get('route')
            command['extra_detour_m']=selected.get('details',{}).get('extra_detour_m')
            command['planner']='A*'
            # Canonical resolver stores the per-loser route in the cost map.
            if not command['route']:
                command['route']=self.conflict_costs((own,peer),zone_name).get(self.robot_id,{}).get('route')
        return command

    def decision_message(self, msg):
        """Adopt only canonical, versioned authority decisions and clears."""
        try:
            value=json.loads(msg.data)
            participants=tuple(value.get('participants',()))
        except (ValueError,TypeError,AttributeError):
            return
        if self.robot_id not in participants:
            return
        if value.get('status') == 'CLEARED':
            removed=self.book.adopt_clear(value)
            if removed:
                self.cleared_conflicts.add(removed['conflict_id'])
            if removed and removed.get('loser') == self.robot_id:
                self.pending_resumes.append(removed)
                self.get_logger().info(
                    f"[{self.robot_id}] SHARED_CLEAR conflict={removed['conflict_id']} "
                    f"version={removed.get('decision_version')} -> RESUME")
            return
        if self.book.adopt(value):
            decision=self.book.get(value.get('conflict_id'))
            if decision:
                self.get_logger().info(
                    f"[{self.robot_id}] DECISION_ADOPTED conflict={decision['conflict_id']} "
                    f"version={decision['decision_version']} "
                    f"authority={decision['authority_robot_id']} "
                    f"winner={decision['winner']} loser={decision['loser']}")

    def peer_message(self, msg):
        try: data = json.loads(msg.data); peer = data['robot_id']
        except (ValueError, TypeError, KeyError): return
        if peer == self.robot_id or peer not in self.cfg['stage4_peer_metadata']: return
        import math
        try:
            if not all(math.isfinite(float(data[key])) for key in ('x','y','yaw','linear_velocity','goal_x','goal_y')): return
        except (KeyError, ValueError, TypeError): return
        was_new = peer not in self.peers
        self.peers[peer] = time.monotonic(); self.peer_states[peer] = data
        self.stale_peers.discard(peer)
        if was_new:
            event = 'PEER_RECONNECTED' if peer in self.offline_peers else 'PEER_CONNECTED'
            self.offline_peers.discard(peer)
            self.get_logger().info(f'[{self.robot_id}] {event} {peer}')

    def audit_peers(self):
        now = time.monotonic(); active = []
        for peer, seen in list(self.peers.items()):
            age = now-seen
            if age > float(self.p['peer_remove_timeout']):
                self.get_logger().info(f'[{self.robot_id}] PEER_OFFLINE {peer}')
                self.offline_peers.add(peer)
                self.stale_peers.discard(peer)
                del self.peers[peer]; self.peer_states.pop(peer, None)
            elif age <= float(self.p['peer_stale_warning']): active.append(peer)
            elif peer not in self.stale_peers:
                self.stale_peers.add(peer)
                self.get_logger().info(f'[{self.robot_id}] PEER_STALE {peer}')
        self.diag.publish(String(data=json.dumps({'robot_id':self.robot_id, 'active_peers':sorted(active), 'count':len(active)})))
        if len(active) != self.last_active_count:
            self.last_active_count = len(active)
            self.get_logger().info(f'[{self.robot_id}] ACTIVE_PEERS={self.last_active_count}')


def main(args=None):
    rclpy.init(args=args); node = PeerState()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
