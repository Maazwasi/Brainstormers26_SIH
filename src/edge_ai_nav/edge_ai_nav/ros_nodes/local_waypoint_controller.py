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
                        max_linear_speed=0.25, max_angular_speed=0.8,
                        goal_tolerance=0.16, obstacle_stop_distance=0.65,
                        obstacle_clear_distance=0.85, sensor_timeout=2.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.rid = self.p['robot_id']
        with open(self.p['config_file']) as f:
            config = yaml.safe_load(f)['warehouse']
        self.spawn = config['robot_spawns'][self.rid]
        # Missions are authored in the warehouse drawing's coordinates, but the
        # controller deliberately operates only in its own odometry frame.  Do
        # this fixed, start-pose conversion once; no map pose or peer data is
        # consumed at runtime.
        sx, sy, syaw = self.spawn
        self.route = []
        for gx, gy in config['stage3_missions'][self.rid]:
            dx, dy = gx - sx, gy - sy
            self.route.append((math.cos(syaw) * dx + math.sin(syaw) * dy,
                               -math.sin(syaw) * dx + math.cos(syaw) * dy))
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
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status = self.create_publisher(String, 'local_status', 10)
        self.create_subscription(Odometry, 'odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.lidar, qos_profile_sensor_data)
        self.create_subscription(String, 'coordination_command', self.coordination, 10)
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
        except (ValueError, TypeError):
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
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.rid+'/base_footprint'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.pub.publish(m)
        self.status.publish(String(data=json.dumps(dict(state=self.state, waypoint=self.index,
                            pose=self.pose, scan=self.scan, linear=v, angular=w,
                            coordination_state=self.coordination_command.get('state', 'NONE'),
                            coordination_hold=self.coordination_holding))))

    def should_hold_for_coordination(self):
        """Yield only at a pre-zone boundary; PROCEED never bypasses LiDAR."""
        state = self.coordination_command.get('state', 'NONE')
        if state == 'SAFE_WAIT':
            return True
        if state not in ('YIELD', 'WAIT_FOR_CLEAR'):
            return False
        zone = self.coordination_command.get('zone_geometry', {})
        if not zone or self.pose is None:
            return True  # malformed local command fails safe
        sx, sy, syaw = self.spawn
        px, py, _ = self.pose
        wx = sx + math.cos(syaw)*px - math.sin(syaw)*py
        wy = sy + math.sin(syaw)*px + math.cos(syaw)*py
        margin = float(self.coordination_command.get('hold_margin', 0.40))
        if 'radius' in zone:
            return math.hypot(wx-zone['center'][0], wy-zone['center'][1]) <= float(zone['radius']) + margin
        bounds = zone.get('bounds', {})
        return (bounds.get('x', [float('inf'), -float('inf')])[0]-margin <= wx <= bounds.get('x', [-float('inf'), float('inf')])[1]+margin and
                bounds.get('y', [float('inf'), -float('inf')])[0]-margin <= wy <= bounds.get('y', [-float('inf'), float('inf')])[1]+margin)

    def tick(self):
        now = time.monotonic()
        if not self.enabled:
            self.change('DISABLED'); self.command(0, 0); return
        if self.pose is None or now-min(self.odom_time, self.scan_time) > self.p['sensor_timeout']:
            self.change('SENSOR_STALE_SAFE_STOP'); self.command(0, 0); return
        if self.index >= len(self.route):
            self.change('MISSION_COMPLETE'); self.command(0, 0); return
        x, y, yaw = self.pose
        front, left, right = (self.scan[k] for k in ('front', 'left', 'right'))
        if self.state == 'AVOID_TURN':
            # Commit to a clear side before trying to re-acquire the original
            # waypoint.  A shallow turn / short advance can otherwise make a
            # circular re-detection around a wide static obstacle.
            if front > self.p['obstacle_clear_distance'] and abs(wrap(yaw-self.turn_start)) > 1.15:
                self.avoid_start = (x, y)
                self.change('AVOID_FORWARD')
            else:
                self.command(0, self.side*0.65); return
        if self.state == 'AVOID_FORWARD':
            if front < 0.40:
                self.change('OBSTACLE_STOP'); self.command(0, 0); return
            if math.hypot(x-self.avoid_start[0], y-self.avoid_start[1]) < 1.15:
                self.command(0.12, 0); return
            self.change('REACQUIRE_WAYPOINT')
        if self.state == 'OBSTACLE_STOP':
            # Pick the safer side from the first scan and retain it while
            # clearing this waypoint's obstacle.  Re-picking on each scan can
            # make a robot alternate around the two faces of one crate.
            if self.obstacle_side is None:
                self.obstacle_side = 1 if left >= right else -1
            self.side = self.obstacle_side
            self.turn_start = yaw
            self.get_logger().info(f'AVOID_DIRECTION={"LEFT" if self.side==1 else "RIGHT"} front={front:.3f} left={left:.3f} right={right:.3f}')
            self.change('AVOID_TURN'); self.command(0, self.side*0.65); return
        gx, gy = self.route[self.index]
        distance = math.hypot(gx-x, gy-y)
        if distance < self.p['goal_tolerance']:
            self.get_logger().info(f'WAYPOINT_REACHED {self.index+1}')
            self.index += 1
            self.obstacle_side = None
            self.command(0, 0); return
        error = wrap(math.atan2(gy-y, gx-x)-yaw)
        requested_hold = self.should_hold_for_coordination()
        gate = motion_gate(front, self.p['obstacle_stop_distance'], requested_hold)
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
        self.change('WAYPOINT_TRACK')
        v = 0 if abs(error) > 0.40 else min(self.p['max_linear_speed'], 0.6*distance)
        w = max(-self.p['max_angular_speed'], min(self.p['max_angular_speed'], 1.7*error))
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
