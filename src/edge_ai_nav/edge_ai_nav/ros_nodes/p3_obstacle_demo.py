"""Isolated, deterministic physical-obstacle P3 recording controller."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from edge_ai_nav.ros_nodes.local_waypoint_controller import sectors, wrap


class P3ObstacleDemo(Node):
    def __init__(self):
        super().__init__('p3_obstacle_demo')
        self.pose = None
        self.scan = None
        self.state = 'STARTING'
        self.wait_until = 0.0
        self.index = 0
        self.goal = (25.0, 18.0)
        self.route = [self.goal]
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'demo_path', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w*q.z + q.x*q.y),
                                1 - 2 * (q.y*q.y + q.z*q.z)))

    def on_scan(self, msg):
        self.scan = sectors(msg)

    def command(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'alpha_1/base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_path(self):
        if self.pose is None:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'alpha_1/odom'
        for x, y in [(self.pose[0], self.pose[1])] + self.route[self.index:]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def tick(self):
        self.publish_path()
        if self.pose is None or self.scan is None:
            self.command()
            return
        now = time.monotonic()
        if self.state == 'STARTING':
            self.state = 'NORMAL'
            self.get_logger().info('[ALPHA 1] Navigating to GOAL')
        if self.state == 'WAITING':
            self.command()
            if now >= self.wait_until:
                x, y, _ = self.pose
                self.route = [(x-1.45, y-0.10), (23.55, 22.65),
                              (25.0, 22.25), self.goal]
                self.index = 0
                self.state = 'P3'
                self.get_logger().info('[ALPHA 1] P3 alternate route selected')
                self.get_logger().info('[ALPHA 1] Rerouting around obstacle')
                self.get_logger().info('[ALPHA 1] Original goal retained: GOAL')
                self.get_logger().info('[ALPHA 1] Navigation resumed')
            return
        if self.state == 'DONE':
            self.command()
            return
        front = self.scan.get('front')
        if self.state == 'NORMAL' and front is not None and front < 1.45:
            self.state = 'WAITING'
            self.wait_until = now + 1.3
            self.get_logger().info('[ALPHA 1] OBSTACLE DETECTED')
            self.get_logger().info('[ALPHA 1] Yielding')
            self.command()
            return
        tx, ty = self.route[self.index]
        dx, dy = tx-self.pose[0], ty-self.pose[1]
        distance = math.hypot(dx, dy)
        if distance < (0.28 if self.index == len(self.route)-1 else 0.38):
            if self.index == len(self.route)-1:
                self.state = 'DONE'
                self.command()
                self.get_logger().info('[ALPHA 1] Goal reached')
                return
            self.index += 1
            return
        error = wrap(math.atan2(dy, dx)-self.pose[2])
        angular = max(-0.65, min(0.65, 1.35*error))
        linear = min(0.28, 0.16 + 0.08*distance)
        if abs(error) > 0.8:
            linear = 0.0
        elif abs(error) > 0.35:
            linear *= 0.45
        self.command(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = P3ObstacleDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
