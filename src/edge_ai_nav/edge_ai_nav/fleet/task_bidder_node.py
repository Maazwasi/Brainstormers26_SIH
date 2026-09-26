"""Per-AMR deterministic task bidding and decentralized claim execution."""
import json
import math
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from .route_graph import WarehouseGraph
from .negotiation import bid_eligible


DISPLAY = {
    'amr_alpha': 'ALPHA 1', 'amr_bravo': 'ALPHA 2',
    'amr_charlie': 'ALPHA 3', 'amr_delta': 'ALPHA 4',
    'amr_echo': 'ALPHA 5',
}
BATTERY = {'amr_alpha': 92, 'amr_bravo': 87, 'amr_charlie': 95,
           'amr_delta': 81, 'amr_echo': 89}


class TaskBidder(Node):
    def __init__(self):
        super().__init__('task_bidder')
        self.declare_parameter('robot_id', 'amr_alpha')
        self.declare_parameter('config_file', '')
        self.rid = self.get_parameter('robot_id').value
        with open(self.get_parameter('config_file').value) as stream:
            self.config = yaml.safe_load(stream)['warehouse']
        self.pose = None
        self.pose_received = 0.0
        self.graph = WarehouseGraph(self.config)
        self.controller_state = 'UNKNOWN'
        self.pending = {}
        self.seen_tasks = set()
        self.claimed_tasks = set()
        self.task_started = None
        self.assignments = []
        self.bid_pub = self.create_publisher(String, '/fleet/task_bids', 20)
        self.claim_pub = self.create_publisher(String, '/fleet/task_claims', 10)
        self.assignment_pub = self.create_publisher(String, '/fleet/task_assignment', 10)
        self.create_subscription(String, '/fleet/tasks', self.on_task, 20)
        self.create_subscription(String, '/fleet/task_bids', self.on_bid, 20)
        self.create_subscription(String, 'local_status', self.on_status, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.resolve)

    @staticmethod
    def decode(msg):
        try:
            value = json.loads(msg.data)
            return value if isinstance(value, dict) else {}
        except (ValueError, TypeError):
            return {}

    def battery(self):
        drain = 0 if self.task_started is None else int((time.monotonic()-self.task_started)/180)
        return max(20, BATTERY[self.rid]-drain)

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.pose_received = time.monotonic()

    def on_status(self, msg):
        state = self.decode(msg).get('state', 'UNKNOWN')
        self.controller_state = state
        if state == 'MISSION_COMPLETE':
            self.task_started = None

    def on_task(self, msg):
        task = self.decode(msg)
        task_id = task.get('task_id')
        if not task_id or task_id in self.seen_tasks:
            return
        self.seen_tasks.add(task_id)
        eligible = bid_eligible(self.controller_state,bool(self.assignments),
                                time.monotonic()-self.pose_received < 2.)
        try:
            if self.pose is None: raise ValueError('No physical pose')
            self.graph.mission_plan(self.pose,task['pickup'],task['destination'])
            distance = self.graph.pickup_distance(self.pose,task['pickup'])
        except (KeyError, ValueError):
            eligible, distance = False, 999.0
        battery = self.battery()
        # Lower is better. Priority is common to every bidder, while distance,
        # battery and workload make the suitability decision explainable.
        priority = int(task.get('priority', 3))
        score = round(distance + (100-battery)*0.08 - priority*0.01
                      + (0 if eligible else 1000), 3)
        self.pending[task_id] = {'task': task, 'bids': {},
                                 'deadline': time.monotonic()+0.65,
                                 'hard_deadline': time.monotonic()+1.20}
        bid = {'task_id': task_id, 'robot_id': self.rid,
               'pickup': task.get('pickup'), 'destination': task.get('destination'),
               'priority': priority, 'type': task.get('type'),
               'display_name': DISPLAY[self.rid], 'eligible': eligible,
               'distance_to_pickup': round(distance, 2), 'battery': battery,
               'score': score, 'timestamp': time.time()}
        self.pending[task_id]['bids'][self.rid] = bid
        self.bid_pub.publish(String(data=json.dumps(bid)))

    def on_bid(self, msg):
        bid = self.decode(msg)
        pending = self.pending.get(bid.get('task_id'))
        if pending and bid.get('robot_id') in DISPLAY:
            pending['bids'][bid['robot_id']] = bid

    def resolve(self):
        now = time.monotonic()
        for assignment in list(self.assignments):
            if now >= assignment['execute_at']:
                self.assignment_pub.publish(String(data=json.dumps(assignment['payload'])))
                self.task_started = now
                self.assignments.remove(assignment)
        for task_id, pending in list(self.pending.items()):
            if now < pending['deadline']:
                continue
            if len(pending['bids']) < len(DISPLAY) and now < pending['hard_deadline']:
                continue
            eligible = [b for b in pending['bids'].values() if b.get('eligible')]
            if not eligible:
                del self.pending[task_id]
                continue
            winner = min(eligible, key=lambda b: (float(b['score']),
                         -int(b['battery']), b['robot_id']))
            if winner['robot_id'] == self.rid and task_id not in self.claimed_tasks:
                self.claimed_tasks.add(task_id)
                task = pending['task']
                if time.monotonic()-self.pose_received > 2.:
                    del self.pending[task_id]
                    continue
                try:
                    plan = self.graph.mission_plan(self.pose,task['pickup'],task['destination'])
                    route = plan['smoothed_waypoints']
                except ValueError:
                    del self.pending[task_id]
                    continue
                claim = dict(task_id=task_id, winner=self.rid,
                             pickup=task['pickup'], destination=task['destination'],
                             priority=task['priority'], type=task['type'], route=route,
                             planner='A*', raw_astar_nodes=plan['raw_astar_nodes'],
                             raw_waypoints=plan['raw_waypoints'],
                             expanded_nodes=plan['expanded_nodes'],
                             raw_route_cost=round(plan['raw_route_cost'],3),
                             smoothed_route_length=round(plan['smoothed_route_length'],3),
                             display_name=DISPLAY[self.rid], state='CLAIMED',
                             battery=winner['battery'],
                             distance=winner['distance_to_pickup'],
                             score=winner['score'], bid_count=len(pending['bids']),
                             reason='Best distance, battery and availability score',
                             timestamp=time.time())
                self.claim_pub.publish(String(data=json.dumps(claim)))
                # Leave a short, video-visible ASSIGNED state between the real
                # claim and automatic local execution.
                self.assignments.append({'execute_at': now+0.45, 'payload': {
                    'robot_id': self.rid, 'task': task['type'], 'task_id': task_id,
                    'pickup': task['pickup'], 'destination': task['destination'],
                    'priority': task['priority'], 'route': route, 'planner':'A*',
                    'raw_astar_nodes':plan['raw_astar_nodes'],
                    'raw_waypoints':plan['raw_waypoints'],
                    'expanded_nodes':plan['expanded_nodes'],
                    'raw_route_cost':plan['raw_route_cost']}})
            del self.pending[task_id]


def main(args=None):
    rclpy.init(args=args)
    node = TaskBidder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
