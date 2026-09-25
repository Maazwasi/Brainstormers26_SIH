"""Warehouse GNSS Simulator Node for ROS 2 / Gazebo AMR Simulation.

Models realistic GNSS availability across physical warehouse zones:
- Outdoor Loading Bay / Apron (X < -3.5m): Nominal GNSS lock (11 sats, HDOP 0.8)
- Covered Entrance Canopy (-3.5 <= X < -1.5m): Degraded GNSS (5 sats, HDOP 2.8, multipath)
- Indoor Warehouse Aisles / Pallet Racks (X >= -1.5m): Total GNSS Outage (0 sats, HDOP 99.0)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool

from .scenario_generator import enu_to_wgs84


class WarehouseGNSSSimulator(Node):
    """Simulates a GNSS receiver on an AMR navigating between outdoor and indoor warehouse zones."""

    def __init__(self):
        super().__init__('warehouse_gnss_simulator')

        # Declare parameters
        self.declare_parameter('ref_lat', 28.6139)
        self.declare_parameter('ref_lon', 77.2090)
        self.declare_parameter('ref_alt', 216.0)
        self.declare_parameter('indoor_x_threshold', -1.5)  # X >= -1.5m is inside warehouse
        self.declare_parameter('canopy_x_threshold', -3.5)  # -3.5 <= X < -1.5m is covered canopy
        self.declare_parameter('publish_rate_hz', 5.0)      # Standard GPS update rate (5Hz)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('frame_prefix', '')

        self.ref_lat = self.get_parameter('ref_lat').value
        self.ref_lon = self.get_parameter('ref_lon').value
        self.ref_alt = self.get_parameter('ref_alt').value
        self.indoor_x = self.get_parameter('indoor_x_threshold').value
        self.canopy_x = self.get_parameter('canopy_x_threshold').value
        pub_rate = self.get_parameter('publish_rate_hz').value
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.frame_prefix = str(self.get_parameter('frame_prefix').value).strip('/')

        # Robot simulated position in warehouse
        self.robot_x = -5.0
        self.robot_y = 3.5
        self.robot_z = 0.0
        self.robot_speed = 0.0
        self.robot_heading_deg = 0.0
        self.has_received_odom = False

        # Manual outage override toggle
        self.manual_outage_override = False

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.status_pub = self.create_publisher(String, 'gps/zone_status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.force_outage_sub = self.create_subscription(
            Bool, 'force_outage', self.force_outage_callback, 10
        )

        # Service to toggle outage
        self.toggle_srv = self.create_service(
            SetBool, 'toggle_gnss_outage', self.handle_toggle_service
        )

        # Timer
        self.timer = self.create_timer(1.0 / pub_rate, self.timer_callback)
        self.get_logger().info(
            f'Warehouse GNSS Simulator started. Physical Outage Boundary at X >= {self.indoor_x}m'
        )

    def odom_callback(self, msg: Odometry):
        self.has_received_odom = True
        self.robot_x = self.origin_x + msg.pose.pose.position.x
        self.robot_y = self.origin_y + msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.sqrt(vx * vx + vy * vy)

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        # Convert ENU yaw (0 = East) to Navigation heading (0 = North, 90 = East)
        heading_rad = (math.pi / 2.0) - yaw_rad
        self.robot_heading_deg = math.degrees(heading_rad) % 360.0

    def force_outage_callback(self, msg: Bool):
        self.manual_outage_override = msg.data
        self.get_logger().info(f'Manual Outage Override set to: {self.manual_outage_override}')

    def handle_toggle_service(self, request, response):
        self.manual_outage_override = request.data
        response.success = True
        response.message = f"GNSS Outage Forced: {self.manual_outage_override}"
        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now
        fix_msg.header.frame_id = f'{self.frame_prefix}/base_footprint' if self.frame_prefix else 'base_footprint'

        # Determine warehouse physical zone based on AMR's X position
        if self.manual_outage_override or self.robot_x >= self.indoor_x:
            # 1. INDOOR WAREHOUSE OUTAGE (Under metallic roof / between pallet racking)
            zone_name = "INDOOR_WAREHOUSE_OUTAGE"
            num_sats = 0
            hdop = 99.0
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.latitude = 0.0
            fix_msg.longitude = 0.0
            fix_msg.altitude = 0.0
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            fix_msg.position_covariance = [10000.0] * 9

        elif self.robot_x >= self.canopy_x:
            # 2. COVERED CANOPY / TRANSITION (Multipath reflection from warehouse entrance)
            zone_name = "COVERED_CANOPY_DEGRADED"
            num_sats = 5
            hdop = 2.8
            # Add multipath noise (~2.5m)
            noisy_e = self.robot_x + np.random.normal(0, 2.0)
            noisy_n = self.robot_y + np.random.normal(0, 2.0)
            lat, lon, alt = enu_to_wgs84(noisy_e, noisy_n, self.ref_lat, self.ref_lon, self.ref_alt)

            fix_msg.status.status = NavSatStatus.STATUS_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = alt
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            var = (hdop * 1.5) ** 2
            fix_msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var * 2]

        else:
            # 3. OUTDOOR LOADING BAY / OPEN APRON (Nominal High Accuracy GPS Fix)
            zone_name = "OUTDOOR_LOADING_BAY"
            num_sats = 11
            hdop = 0.8
            # Commercial GPS noise (~0.5m)
            noisy_e = self.robot_x + np.random.normal(0, 0.4)
            noisy_n = self.robot_y + np.random.normal(0, 0.4)
            lat, lon, alt = enu_to_wgs84(noisy_e, noisy_n, self.ref_lat, self.ref_lon, self.ref_alt)

            fix_msg.status.status = NavSatStatus.STATUS_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = alt
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            var = (hdop * 0.8) ** 2
            fix_msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var * 2]

        self.fix_pub.publish(fix_msg)

        # Publish diagnostic zone status
        status_msg = String()
        status_msg.data = (
            f'{{"zone": "{zone_name}", "robot_x": {self.robot_x:.2f}, "robot_y": {self.robot_y:.2f}, '
            f'"sats": {num_sats}, "hdop": {hdop:.1f}, "speed_mps": {self.robot_speed:.2f}}}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseGNSSSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
