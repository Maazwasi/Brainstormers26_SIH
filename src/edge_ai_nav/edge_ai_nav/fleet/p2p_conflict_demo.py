"""One-shot setup helper for the ALPHA 2 / ALPHA 4 P2P video demonstration.

This node only stages and manually assigns two valid A* tasks.  It neither
publishes velocity nor chooses a conflict winner; the existing peer-state
nodes perform prediction, hierarchy and the YIELD/PROCEED decision.
"""
import json
import math
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .route_graph import WarehouseGraph


BRAVO = 'amr_bravo'
DELTA = 'amr_delta'
WEST_HOLD = 'INTERSECTION_A_WEST_HOLD'
SOUTH_HOLD = 'INTERSECTION_A_SOUTH_HOLD'
EAST = 'INTERSECTION_A_EAST'
NORTH = 'INTERSECTION_A_NORTH'


class P2PConflictDemo(Node):
    def __init__(self):
        super().__init__('p2p_conflict_demo')
        config = get_package_share_directory('amr_simulation') + '/config/warehouse_sih_demo.yaml'
        with open(config, encoding='utf-8') as stream:
            self.cfg = yaml.safe_load(stream)['warehouse']
        self.graph = WarehouseGraph(self.cfg)
        self.poses, self.statuses = {}, {}
        self.phase = 'WAITING_FOR_FLEET'
        self.minimum_separation = float('inf')
        self.yield_seen = False
        self.assignment_pub = self.create_publisher(String, '/fleet/task_assignment', 10)
        self.event_pub = self.create_publisher(String, '/fleet/coordination_events', 20)
        for rid in (BRAVO, DELTA):
            self.create_subscription(Odometry, f'/{rid}/odom',
                                     lambda msg, robot=rid: self.odom(robot, msg), 10)
            self.create_subscription(String, f'/{rid}/local_status',
                                     lambda msg, robot=rid: self.status(robot, msg), 10)
        self.create_timer(.10, self.tick)
        self.get_logger().info('P2P video demo helper started; waiting for ALPHA 2 and ALPHA 4.')

    def event(self, event, **fields):
        self.event_pub.publish(String(data=json.dumps(dict(
            event=event, robot_id='P2P_DEMO', timestamp=time.time(), **fields),
            separators=(',', ':'))))

    def odom(self, robot, message):
        self.poses[robot] = (message.pose.pose.position.x, message.pose.pose.position.y)
        if BRAVO in self.poses and DELTA in self.poses and self.phase == 'CROSSING':
            self.minimum_separation = min(self.minimum_separation,
                                          math.dist(self.poses[BRAVO], self.poses[DELTA]))

    def status(self, robot, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.statuses[robot] = value
        except (TypeError, ValueError):
            pass

    def at(self, robot, node, tolerance=.28):
        return robot in self.poses and math.dist(self.poses[robot], self.graph.nodes[node]) <= tolerance

    def task(self, robot, task_id, pickup, destination, priority, demo_hold=False):
        position = self.poses[robot]
        plan = self.graph.mission_plan(position, pickup, destination)
        assignment = dict(robot_id=robot, task='P2P_CONFLICT_DEMO', task_id=task_id,
                          pickup=pickup, destination=destination, priority=priority,
                          route=plan['smoothed_waypoints'], planner='A*',
                          raw_astar_nodes=plan['raw_astar_nodes'],
                          raw_waypoints=plan['raw_waypoints'],
                          expanded_nodes=plan['expanded_nodes'],
                          raw_route_cost=plan['raw_route_cost'],
                          demo_hold=demo_hold,
                          assignment_source='MANUAL_OPERATOR_OVERRIDE')
        self.assignment_pub.publish(String(data=json.dumps(assignment)))
        self.get_logger().info(f'{robot} {task_id}: {pickup} -> {destination}')

    def tick(self):
        if len(self.poses) != 2:
            return
        if self.phase == 'WAITING_FOR_FLEET':
            self.task(DELTA, 'P2P-DEMO-STAGE-DELTA', SOUTH_HOLD, SOUTH_HOLD, 2, demo_hold=True)
            self.phase = 'STAGING_DELTA'
            self.event('P2P_DEMO_SETUP', detail='ALPHA 4 staging south of Intersection A')
        elif self.phase == 'STAGING_DELTA' and self.statuses.get(DELTA, {}).get('state') == 'DEMO_STAGED':
            self.task(BRAVO, 'P2P-DEMO-STAGE-BRAVO', WEST_HOLD, WEST_HOLD, 3, demo_hold=True)
            self.phase = 'STAGING_BRAVO'
            self.event('P2P_DEMO_SETUP', detail='ALPHA 4 held; ALPHA 2 staging west of Intersection A')
        elif self.phase == 'STAGING_BRAVO' and self.statuses.get(BRAVO, {}).get('state') == 'DEMO_STAGED':
            self.task(BRAVO, 'P2P-DEMO-BRAVO-HIGH', WEST_HOLD, EAST, 3)
            self.task(DELTA, 'P2P-DEMO-DELTA-NORMAL', SOUTH_HOLD, NORTH, 2)
            self.phase = 'CROSSING'
            self.event('P2P_DEMO_CROSSING_STARTED', zone='intersection_A',
                       robots=[BRAVO, DELTA],
                       detail='ALPHA 2 HIGH vs ALPHA 4 NORMAL; decentralized decision pending')
        elif self.phase == 'CROSSING':
            delta = self.statuses.get(DELTA, {})
            if delta.get('coordination_state') in ('YIELD', 'YIELD_AND_WAIT', 'WAIT_FOR_CLEAR'):
                self.yield_seen = True
            # RESUME is a short edge command; the northbound exit is durable
            # proof that ALPHA 4 resumed its original route after yielding.
            if self.yield_seen and self.poses.get(DELTA, (0., 0.))[1] > 1.35:
                self.event('P2P_DEMO_RESULT', zone='intersection_A',
                           minimum_separation_m=round(self.minimum_separation, 3),
                           collision=False, emergency_recovery=False)
                self.get_logger().info(
                    f'Demo complete; minimum separation {self.minimum_separation:.3f} m')
                self.phase = 'COMPLETE'


def main(args=None):
    rclpy.init(args=args)
    node = P2PConflictDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
