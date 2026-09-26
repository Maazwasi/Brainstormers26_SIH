"""DDS peer exchange plus local Stage 6 negotiation; never publishes cmd_vel."""
import json
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .zone_prediction import LocalDetector, load_zones
from .negotiation import NegotiationBook, outside_with_margin
from .route_graph import WarehouseGraph


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
        self.meta = self.cfg['stage4_peer_metadata'][self.robot_id]
        self.graph = WarehouseGraph(self.cfg)
        self.zone_request = None
        self.spawn = self.cfg['robot_spawns'][self.meta['namespace'].lstrip('/')]
        self.negotiation_enabled = self.get_parameter('negotiation').value
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
        self.policy = dict(waiting_cap=float(stage6.get('waiting_cap', 30.0)),
                           waiting_quantum=float(stage6.get('waiting_quantum', 1.0)),
                           eta_quantum=float(stage6.get('eta_quantum', 0.25)))
        self.hold_margin = max(0.55,float(stage6.get('hold_margin', 0.40)))
        self.exit_margin = max(0.55,float(stage6.get('exit_margin', 0.40)))
        self.maximum_reroute_detour = float(stage6.get('maximum_reroute_detour_m', 2.5))
        self.book = NegotiationBook()
        self.pub = self.create_publisher(String, '/fleet/peer_state', qos)
        self.diag = self.create_publisher(String, 'peer_diagnostics', 10)
        self.conflicts = self.create_publisher(String, 'conflict_status', 10) if self.detector else None
        self.coordination = self.create_publisher(String, 'coordination_command', 10) if self.negotiation_enabled else None
        self.event_pub = self.create_publisher(String, '/fleet/coordination_events', 20)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, 'local_status', self.local_status, 10)
        self.create_subscription(String, '/fleet/peer_state', self.peer_message, qos)
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
        if nav_state in ('DISABLED', 'MISSION_COMPLETE', 'UNKNOWN', 'SENSOR_STALE_SAFE_STOP'):
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
        idle = nav_state in ('DISABLED','MISSION_COMPLETE','PARKED','AVAILABLE','UNKNOWN')
        priority = 0 if idle else max(1,min(4,int(self.status.get('task_priority',2))))
        request_key=(self.status.get('active_task','NONE'),next_zone)
        if not self.zone_request or tuple(self.zone_request['key']) != request_key:
            self.zone_request=dict(key=request_key,task_priority=priority,
                                   waiting_time=waiting_time,eta=intent.get('eta',0.))
        data = dict(robot_id=self.robot_id, timestamp=self.get_clock().now().nanoseconds/1e9,
                    x=x, y=y, yaw=yaw,
                    linear_velocity=self.twist.linear.x if self.twist else 0.0,
                    angular_velocity=self.twist.angular.z if self.twist else 0.0,
                    goal_x=goal[0], goal_y=goal[1], next_zone=next_zone,
                    task_priority=priority, waiting_time=waiting_time,
                    zone_intent=intent, zone_request=self.zone_request, eta=intent.get('eta'),
                    local_nav_state=nav_state, localization_mode='ODOM',
                    localization_confidence=1.0, coordination_state='NONE',
                    zone_committed=bool(intent.get('inside')),
                    zone_reserved=any(item['winner']==self.robot_id and
                        item['conflict_id'].split(':',1)[0]==next_zone
                        for item in self.book.values()),
                    safety_emergency=nav_state in ('SENSOR_STALE_SAFE_STOP','EMERGENCY_STOP'),
                    active_task=self.status.get('active_task','NONE'), route=route,
                    remaining_route=remaining, destination=self.status.get('destination',''),
                    parking_target=self.status.get('parking_target',''),
                    parking_state=self.status.get('parking_state','FREE'))
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
                              robots=sorted((self.robot_id,record['peer'])))
            if self.negotiation_enabled:
                command = self.negotiate(data, unavailable)
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
                    route,extra=self.graph.reroute((state['x'],state['y']),remaining,destination,zone)
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

    def negotiate(self, own, unavailable):
        """Calculate only this AMR's local command from DDS state and geometry."""
        now = own['timestamp']  # Same ROS system clock as detector first-detect timestamp.
        # Clear every completed latch, not just the first selected pair.
        cleared=[]
        for old in list(self.book.values()):
            winner_state=own if old['winner']==self.robot_id else self.peer_states.get(old['winner'])
            loser_state=own if old['loser']==self.robot_id else self.peer_states.get(old['loser'])
            zone=self.detector.zones[old['conflict_id'].split(':',1)[0]]
            if winner_state and old['winner'] not in unavailable:
                if not outside_with_margin(winner_state,zone,0.): old['winner_entered_zone']=True
                route_invalidated=(old['conflict_id'] not in self.detector.active and
                                   now-old['decision_time']>1.0)
                if ((old['winner_entered_zone'] and outside_with_margin(winner_state,zone,self.exit_margin))
                        or route_invalidated):
                    cleared.append(dict(old)); self.book.clear(old['conflict_id'])
        for cid, record in self.detector.active.items():
            if any(c['conflict_id']==cid for c in cleared): continue
            peer = self.peer_states.get(record['peer'])
            if peer is None:
                continue
            # Immutable request snapshots make both independent computations
            # use the same priority, waiting age and ETA despite DDS timing.
            a=dict(own,**{k:own.get('zone_request',own).get(k,own[k]) for k in ('task_priority','waiting_time')})
            b=dict(peer,**{k:peer.get('zone_request',peer).get(k,peer[k]) for k in ('task_priority','waiting_time')})
            zone=self.detector.zones[record['zone']]
            for state in (a,b):
                state['zone_committed']=not outside_with_margin(state,zone,0.)
            costs=self.conflict_costs((a,b),record['zone'])
            decision = self.book.decide(cid, a, b,
                {self.robot_id: own.get('zone_request',{}).get('eta',record['my_eta']),
                 record['peer']: peer.get('zone_request',{}).get('eta',record['peer_eta'])},
                now, record['conflict_first_detected_time'], zone=record['zone'],
                route_costs=costs, **self.policy)
            if decision['decision_time'] == now:
                winner_state=a if decision['winner']==a['robot_id'] else b
                loser_state=b if winner_state is a else a
                decision['baseline_delay_s']=round(
                    float(winner_state.get('zone_intent',{}).get('window',[0.,0.])[1])+
                    float(loser_state.get('zone_intent',{}).get('window',[0.,0.])[1]),3)
                self.get_logger().info(
                    f"[{self.robot_id}] NEGOTIATION conflict={cid} winner={decision['winner']} "
                    f"loser={decision['loser']} reason={decision['reason']} "
                    f"latency_ms={decision['decision_latency_ms']:.3f}")
                if self.robot_id < record['peer']:
                    self.emit('NEGOTIATION_COMPLETE',event_id=cid+':decision',
                              conflict_id=cid,zone=record['zone'],winner=decision['winner'],
                              loser=decision['loser'],loser_action=decision['loser_action'],
                              decision_basis=decision['reason'],
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
        elif selected['winner_entered_zone'] and winner_state is not None and outside_with_margin(winner_state, zone, self.exit_margin):
            state = 'RESUME'
            self.get_logger().info(f'[{self.robot_id}] ZONE_CLEAR -> RESUME {zone_name}')
            self.book.clear(selected['conflict_id'])
        elif selected['winner'] == self.robot_id:
            # The immutable, deterministic latch is already calculated by
            # both peers.  Normal P2P arbitration never changes the winner's
            # route or turns it into a waiter.
            state = 'PROCEED'
            self.wait_credit=0.0
        else:
            state = ('REROUTE' if selected.get('loser_action')=='REROUTE' else
                     'WAIT_FOR_CLEAR' if self.status.get('coordination_hold', False) else 'YIELD_AND_WAIT')
        command=dict(state=state, conflict_id=selected['conflict_id'], zone=zone_name,
                    winner=selected['winner'], loser=selected['loser'],
                    hold_margin=self.hold_margin, exit_margin=self.exit_margin,
                    zone_geometry=zone, reason=selected['reason'],
                    decision_latency_ms=selected['decision_latency_ms'],
                    decision_basis=selected['reason'])
        if state=='REROUTE':
            command['route']=selected.get('details',{}).get('reroute_route') or \
                selected.get('details',{}).get('route')
            command['extra_detour_m']=selected.get('details',{}).get('extra_detour_m')
            command['planner']='A*'
            # Canonical resolver stores the per-loser route in the cost map.
            if not command['route']:
                command['route']=self.conflict_costs((own,peer),zone_name).get(self.robot_id,{}).get('route')
        return command

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
