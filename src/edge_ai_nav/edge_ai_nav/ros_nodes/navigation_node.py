"""ROS 2 Navigation Node publishing fused NavSatFix, Odometry, TF and Diagnostics."""

import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from ..pipeline import NavigationPipeline
from ..visualization.web_server import WebDashboardServer


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to ROS Quaternion message."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class EdgeAINavigationNode(Node):
    """ROS 2 node wrapping the Edge AI + GNSS/IDR Pipeline."""

    def __init__(self):
        super().__init__('edge_ai_navigation_node')
        self.get_logger().info('Initializing Edge AI + GNSS/IDR Navigation Node...')

        # Declare parameters
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('ref_lat', 28.6139)
        self.declare_parameter('ref_lon', 77.2090)
        self.declare_parameter('ref_alt', 216.0)

        rate_hz = self.get_parameter('rate_hz').value
        web_port = self.get_parameter('web_port').value
        ref_lat = self.get_parameter('ref_lat').value
        ref_lon = self.get_parameter('ref_lon').value
        ref_alt = self.get_parameter('ref_alt').value

        # Initialize core pipeline
        self.pipeline = NavigationPipeline(
            ref_lat=ref_lat,
            ref_lon=ref_lon,
            ref_alt=ref_alt,
            enable_logging=True
        )

        # Start Web Dashboard
        self.web_server = WebDashboardServer(self.pipeline, port=web_port)
        try:
            self.web_server.start()
            self.get_logger().info(f'Web Dashboard active at http://localhost:{web_port}')
        except OSError as e:
            self.get_logger().warning(f'Web Dashboard port {web_port} busy ({e}), continuing with ROS 2 publishers.')

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/nav/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/nav/odometry', 10)
        self.idr_pub = self.create_publisher(Odometry, '/nav/idr_odom', 10)
        self.status_pub = self.create_publisher(String, '/nav/status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # High-rate timer
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'Navigation pipeline running at {rate_hz} Hz')

    def timer_callback(self):
        out = self.pipeline.step()
        now_msg = self.get_clock().now().to_msg()

        # 1. Publish /nav/fix (NavSatFix)
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now_msg
        fix_msg.header.frame_id = 'base_link'
        fix_msg.latitude = out.est_lat
        fix_msg.longitude = out.est_lon
        fix_msg.altitude = out.est_alt
        if out.mode == "GNSS_FIX":
            fix_msg.status.status = NavSatStatus.STATUS_FIX
        elif out.mode == "IDR_ACTIVE":
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        self.fix_pub.publish(fix_msg)

        # Heading to ROS ENU yaw (0 = East, pi/2 = North)
        # our heading_deg: 0 = North, 90 = East -> yaw_enu = pi/2 - heading_rad
        heading_rad = math.radians(out.est_heading)
        yaw_enu = (math.pi / 2.0) - heading_rad
        q = euler_to_quaternion(0.0, 0.0, yaw_enu)

        # 2. Publish /nav/odometry (Odometry)
        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = out.est_east
        odom_msg.pose.pose.position.y = out.est_north
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = out.est_speed
        self.odom_pub.publish(odom_msg)

        # 3. Publish /nav/idr_odom (Dead Reckoning Only)
        idr_msg = Odometry()
        idr_msg.header.stamp = now_msg
        idr_msg.header.frame_id = 'odom'
        idr_msg.child_frame_id = 'base_link'
        idr_msg.pose.pose.position.x = out.idr_east
        idr_msg.pose.pose.position.y = out.idr_north
        idr_msg.pose.pose.position.z = 0.0
        idr_msg.pose.pose.orientation = q
        self.idr_pub.publish(idr_msg)

        # 4. Publish /nav/status (JSON string)
        status_dict = {
            "mode": out.mode,
            "ai_condition": out.ai_condition,
            "ai_confidence": round(out.ai_confidence, 3),
            "outage_duration_s": round(out.outage_duration, 2),
            "accumulated_drift_m": round(out.accumulated_drift, 2),
            "speed_mps": round(out.est_speed, 2),
            "sats": out.gnss_sats,
            "hdop": round(out.gnss_hdop, 2),
        }
        str_msg = String()
        str_msg.data = json.dumps(status_dict)
        self.status_pub.publish(str_msg)

        # 5. Broadcast TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = out.est_east
        t.transform.translation.y = out.est_north
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.web_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EdgeAINavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
