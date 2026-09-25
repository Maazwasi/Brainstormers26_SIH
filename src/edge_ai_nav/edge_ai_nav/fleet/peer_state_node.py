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


class PeerState(Node):
    def __init__(self):
        super().__init__('peer_state')
        for name, value in {'robot_id':'ALPHA', 'config_file':'', 'publish_rate_hz':5.0,
                            'peer_stale_warning':1.0, 'peer_remove_timeout':2.0,
                            'conflict_detection':False, 'negotiation':False}.items():
            self.declare_parameter(name, value)
        p = {n:self.get_parameter(n).value for n in ('robot_id','config_file','publish_rate_hz','peer_stale_warning','peer_remove_timeout')}
        self.robot_id, self.p = p['robot_id'], p
        with open(p['config_file']) as stream:
            self.cfg = yaml.safe_load(stream)['warehouse']
        self.meta = self.cfg['stage4_peer_metadata'][self.robot_id]
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
        self.peers, self.peer_states, self.offline_peers, self.stale_peers = {}, {}, set(), set()
        self.last_active_count = -1
        stage6 = self.cfg.get('stage6', {})
        self.policy = dict(waiting_cap=float(stage6.get('waiting_cap', 30.0)),
                           waiting_quantum=float(stage6.get('waiting_quantum', 1.0)),
                           eta_quantum=float(stage6.get('eta_quantum', 0.25)))
        self.hold_margin = float(stage6.get('hold_margin', 0.40))
        self.exit_margin = float(stage6.get('exit_margin', 0.40))
        self.book = NegotiationBook()
        self.pub = self.create_publisher(String, '/fleet/peer_state', qos)
        self.diag = self.create_publisher(String, 'peer_diagnostics', 10)
        self.conflicts = self.create_publisher(String, 'conflict_status', 10) if self.detector else None
        self.coordination = self.create_publisher(String, 'coordination_command', 10) if self.negotiation_enabled else None
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
        # Only this robot's odometry plus its immutable configured spawn is
        # used.  No other robot's odometry is ever consumed.
        x = sx + math.cos(spawn_yaw)*pos.x - math.sin(spawn_yaw)*pos.y
        y = sy + math.sin(spawn_yaw)*pos.x + math.cos(spawn_yaw)*pos.y
        yaw = math.atan2(math.sin(spawn_yaw + local_yaw), math.cos(spawn_yaw + local_yaw))
        index = int(self.status.get('waypoint', 0))
        route = self.cfg['stage3_missions'][self.meta['namespace'].lstrip('/')]
        goal = route[min(index, len(route)-1)]
        nav_state = self.status.get('state', 'UNKNOWN')
        fresh = time.monotonic() - min(self.odom_received, self.status_received) <= 2.0
        if not fresh: nav_state = 'SENSOR_STALE_SAFE_STOP'
        zones = self.cfg.get('mission_zones', {}).get(self.meta['namespace'].lstrip('/'), [])
        next_zone = zones[min(index, len(zones)-1)] if zones else self.meta['next_zone']
        if nav_state in ('DISABLED', 'MISSION_COMPLETE', 'UNKNOWN', 'SENSOR_STALE_SAFE_STOP'):
            next_zone = ''
        if nav_state in ('OBSTACLE_STOP', 'SENSOR_STALE_SAFE_STOP') or self.status.get('coordination_hold', False):
            self.wait_started = self.wait_started or time.monotonic()
        else:
            self.wait_started = None
        waiting_time = 0.0 if self.wait_started is None else time.monotonic() - self.wait_started
        data = dict(robot_id=self.robot_id, timestamp=self.get_clock().now().nanoseconds/1e9,
                    x=x, y=y, yaw=yaw,
                    linear_velocity=self.twist.linear.x if self.twist else 0.0,
                    angular_velocity=self.twist.angular.z if self.twist else 0.0,
                    goal_x=goal[0], goal_y=goal[1], next_zone=next_zone,
                    task_priority=self.meta['priority'], waiting_time=waiting_time,
                    local_nav_state=nav_state, localization_mode='ODOM',
                    localization_confidence=1.0, coordination_state='NONE')
        if self.detector:
            now = time.monotonic()
            unavailable = self.offline_peers | {rid for rid, seen in self.peers.items()
                if now-seen > float(self.p['peer_stale_warning'])}
            current = [v for rid, v in self.peer_states.items() if rid not in unavailable]
            for event, record in self.detector.update(data, current, data['timestamp'], unavailable):
                self.get_logger().info(f'[{self.robot_id}] {event} ' + json.dumps(record, separators=(',', ':')))
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

    def negotiate(self, own, unavailable):
        """Calculate only this AMR's local command from DDS state and geometry."""
        now = own['timestamp']  # Same ROS system clock as detector first-detect timestamp.
        for cid, record in self.detector.active.items():
            peer = self.peer_states.get(record['peer'])
            if peer is None:
                continue
            decision = self.book.decide(cid, own, peer,
                {self.robot_id: record['my_eta'], record['peer']: record['peer_eta']},
                now, record['conflict_first_detected_time'], **self.policy)
            if decision['decision_time'] == now:
                self.get_logger().info(
                    f"[{self.robot_id}] NEGOTIATION conflict={cid} winner={decision['winner']} "
                    f"loser={decision['loser']} reason={decision['reason']} "
                    f"latency_ms={decision['decision_latency_ms']:.3f}")

        selected = next((item for item in self.book.values() if item['loser'] == self.robot_id),
                        next(iter(self.book.values()), None))
        if selected is None:
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
        if selected['loser'] == self.robot_id and peer_unavailable:
            state = 'SAFE_WAIT'
        elif selected['winner_entered_zone'] and winner_state is not None and outside_with_margin(winner_state, zone, self.exit_margin):
            state = 'RESUME'
            self.get_logger().info(f'[{self.robot_id}] ZONE_CLEAR -> RESUME {zone_name}')
            self.book.clear(selected['conflict_id'])
        elif selected['winner'] == self.robot_id:
            state = 'PROCEED'
        else:
            state = 'WAIT_FOR_CLEAR' if self.status.get('coordination_hold', False) else 'YIELD'
        return dict(state=state, conflict_id=selected['conflict_id'], zone=zone_name,
                    winner=selected['winner'], loser=selected['loser'],
                    hold_margin=self.hold_margin, exit_margin=self.exit_margin,
                    zone_geometry=zone, reason=selected['reason'],
                    decision_latency_ms=selected['decision_latency_ms'])

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
