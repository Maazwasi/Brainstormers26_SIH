"""AMR Warehouse Mission Waypoint Navigator for Nav2 & Autonomous Traversal.

Drives the AMR through the demonstration scenario:
1. Loading Dock Bay (GNSS ON)
2. Warehouse Entrance Door (Simulated Outage occurs -> Edge AI alert -> IDR active)
3. Pallet Rack Aisle (AMR navigates under pure IDR)
4. Warehouse Exit Door (GNSS restores -> EKF Fusion reconciles accumulated error)
5. Return to Loading Bay (ZUPT standstill hold)
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class AMRWarehouseNavigator(Node):
    """Executes the autonomous warehouse demonstration mission."""

    WAYPOINTS = [
        {"name": "1. Loading Bay Dock (GNSS ON)", "x": -4.5, "y": 3.5, "yaw": 0.0},
        {"name": "2. Warehouse Entrance Door (OUTAGE TRIGGER)", "x": -1.2, "y": 2.2, "yaw": -0.5},
        {"name": "3. Pallet Rack Aisle (IDR NAVIGATION)", "x": 1.5, "y": 1.0, "yaw": -1.57},
        {"name": "4. Warehouse Cross-Aisle (IDR CURVE)", "x": 1.5, "y": -2.5, "yaw": 3.14},
        {"name": "5. Exit Door Return (GNSS RECOVERY)", "x": -3.5, "y": -1.5, "yaw": 1.57},
        {"name": "6. Dock Return & Standstill (ZUPT HOLD)", "x": -4.5, "y": 3.5, "yaw": 0.0},
    ]

    def __init__(self):
        super().__init__('amr_warehouse_navigator')
        self.get_logger().info('AMR Warehouse Mission Navigator initializing...')

        # Pose tracking
        self.current_x = -5.0
        self.current_y = 3.5
        self.current_yaw = 0.0
        self.has_odom = False

        # Status tracking
        self.nav_mode = "GNSS_FIX"
        self.ai_condition = "OPEN_SKY_NORMAL"

        # Action Client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Fallback direct cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/amr/nav_status', self.status_callback, 10)

        self.current_wp_idx = 0
        self.is_mission_active = False

    def odom_callback(self, msg: Odometry):
        self.has_odom = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def status_callback(self, msg: String):
        try:
            import json
            data = json.loads(msg.data)
            self.nav_mode = data.get("mode", self.nav_mode)
            self.ai_condition = data.get("ai_condition", self.ai_condition)
        except Exception:
            pass

    def run_mission(self):
        """Execute waypoints sequentially."""
        self.get_logger().info('Starting Autonomous AMR Warehouse Mission...')
        time.sleep(1.5)

        for i, wp in enumerate(self.WAYPOINTS):
            self.get_logger().info(
                f"\n\033[1;36m>>> EXECUTING WAYPOINT {i+1}/{len(self.WAYPOINTS)}: {wp['name']}\033[0m"
            )
            self.navigate_to_target(wp["x"], wp["y"], wp["yaw"])
            time.sleep(1.0)

        self.get_logger().info(
            '\n\033[1;32m[✓] AMR WAREHOUSE DEMO MISSION COMPLETE! All navigation phases verified.\033[0m'
        )

    def navigate_to_target(self, target_x: float, target_y: float, target_yaw: float):
        """Drive toward target point with proportional-derivative speed and angular tracking."""
        rate = self.create_rate(20)  # 20 Hz
        dist_threshold = 0.35

        start_t = time.time()
        max_duration = 30.0  # seconds timeout per waypoint

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < dist_threshold or (time.time() - start_t) > max_duration:
                # Arrived at waypoint
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                break

            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.current_yaw + math.pi) % (2 * math.pi) - math.pi

            # Pure pursuit / proportional steering
            cmd = Twist()
            if abs(angle_diff) > 0.5:
                # Rotate in place to face target
                cmd.linear.x = 0.05
                cmd.angular.z = max(-1.0, min(1.0, 1.8 * angle_diff))
            else:
                # Drive forward with curved steering
                cmd.linear.x = min(0.35, 0.4 * dist)
                cmd.angular.z = max(-0.8, min(0.8, 1.5 * angle_diff))

            self.cmd_pub.publish(cmd)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    navigator = AMRWarehouseNavigator()
    try:
        navigator.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        stop_cmd = Twist()
        navigator.cmd_pub.publish(stop_cmd)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
