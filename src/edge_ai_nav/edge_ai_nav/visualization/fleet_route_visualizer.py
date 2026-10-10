"""Convert SWARMX mission-route JSON into RViz-friendly Path messages."""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ROBOT_IDS = tuple(f'alpha_{index}' for index in range(1, 6))
INACTIVE_STATES = {
    'AVAILABLE', 'CHARGING', 'DEMO_STAGED', 'DISABLED', 'MISSION_COMPLETE',
    'PARKED', 'UNKNOWN',
}


class FleetRouteVisualizer(Node):
    """Maintain one latched map-frame Path per AMR without affecting control."""

    def __init__(self):
        super().__init__('fleet_route_visualizer')
        route_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publishers = {
            rid: self.create_publisher(Path, f'/{rid}/route_path', route_qos)
            for rid in ROBOT_IDS
        }
        self.routes = {rid: () for rid in ROBOT_IDS}
        self.last_warning = None
        self.create_subscription(String, '/fleet/mission_routes', self.on_route, 10)
        for rid in ROBOT_IDS:
            self.create_subscription(
                String, f'/{rid}/local_status',
                lambda message, robot_id=rid: self.on_status(robot_id, message), 10)
        for rid in ROBOT_IDS:
            self.publish_path(rid, ())
        self.get_logger().info(
            'Fleet route visualization ready: /fleet/mission_routes -> five route_path topics')

    def warn_once(self, reason):
        if reason != self.last_warning:
            self.last_warning = reason
            self.get_logger().warning(f'Ignored malformed mission route: {reason}')

    @staticmethod
    def parse_points(data):
        route = data.get('route')
        if not isinstance(route, list):
            raise ValueError('route must be a list')
        points = []
        for index, point in enumerate(route):
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                x, y = point[0], point[1]
            elif isinstance(point, dict) and 'x' in point and 'y' in point:
                x, y = point['x'], point['y']
            else:
                raise ValueError(f'route[{index}] is not an x/y coordinate')
            try:
                x, y = float(x), float(y)
            except (TypeError, ValueError) as error:
                raise ValueError(f'route[{index}] has non-numeric coordinates') from error
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(f'route[{index}] has non-finite coordinates')
            points.append((x, y))
        return tuple(points)

    def on_route(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('payload must be a JSON object')
            rid = data.get('robot_id')
            if rid not in self.path_publishers:
                raise ValueError(f'unknown robot_id {rid!r}')
            points = self.parse_points(data)
        except (json.JSONDecodeError, ValueError) as error:
            self.warn_once(str(error))
            return
        self.last_warning = None
        if points != self.routes[rid]:
            self.routes[rid] = points
            self.publish_path(rid, points)

    def on_status(self, rid, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict) and data.get('state') in INACTIVE_STATES and self.routes[rid]:
            self.routes[rid] = ()
            self.publish_path(rid, ())

    def publish_path(self, rid, points):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            if len(points) > 1:
                target = points[index + 1] if index + 1 < len(points) else points[index - 1]
                dx = target[0] - x if index + 1 < len(points) else x - target[0]
                dy = target[1] - y if index + 1 < len(points) else y - target[1]
                yaw = math.atan2(dy, dx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.path_publishers[rid].publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = FleetRouteVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
