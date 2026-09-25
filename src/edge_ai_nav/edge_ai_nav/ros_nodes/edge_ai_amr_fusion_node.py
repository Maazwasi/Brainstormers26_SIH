"""Edge AI + GNSS / Intelligent Dead Reckoning AMR Fusion Node for Gazebo / Nav2.

Subscribes directly to real AMR simulation topics:
- /imu (sensor_msgs/msg/Imu)
- /odom (nav_msgs/msg/Odometry)
- /gps/fix (sensor_msgs/msg/NavSatFix)

Publishes:
- /amr/odometry/fused (nav_msgs/msg/Odometry)
- /amr/idr_odometry (nav_msgs/msg/Odometry)
- /amr/path_fused (nav_msgs/msg/Path)
- /amr/path_idr (nav_msgs/msg/Path)
- /amr/nav_status (std_msgs/msg/String)
- /amr/status_marker (visualization_msgs/msg/Marker)
"""

import json
import math
import time
from collections import deque
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String

from ..sensor_layer.data_types import SensorFrame, GNSSData, IMUData, NavigationOutput
from ..sensor_layer.scenario_generator import wgs84_to_enu, enu_to_wgs84
from ..outage_detector.detector import GNSSOutageDetector
from ..idr_engine.imu_preprocessor import IMUPreprocessor
from ..idr_engine.attitude_filter import AttitudeFilter
from ..idr_engine.dead_reckoner import IntelligentDeadReckoner
from ..fusion_engine.ekf_fusion import EKFNavFusion
from ..edge_ai.classifier import EdgeAIClassifier


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
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


class EdgeAIAMRFusionNode(Node):
    """ROS 2 Node executing real-time Edge AI + IDR Fusion for the simulated AMR."""

    def __init__(self):
        super().__init__('edge_ai_amr_fusion_node')

        # Parameters
        self.declare_parameter('ref_lat', 28.6139)
        self.declare_parameter('ref_lon', 77.2090)
        self.declare_parameter('ref_alt', 216.0)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('frame_prefix', '')

        self.ref_lat = self.get_parameter('ref_lat').value
        self.ref_lon = self.get_parameter('ref_lon').value
        self.ref_alt = self.get_parameter('ref_alt').value
        rate_hz = self.get_parameter('rate_hz').value
        self.frame_prefix = str(self.get_parameter('frame_prefix').value).strip('/')
        self.base_frame = f'{self.frame_prefix}/base_footprint' if self.frame_prefix else 'base_footprint'

        # Pipeline Engines
        self.imu_prep = IMUPreprocessor()
        self.att_filter = AttitudeFilter()
        self.idr = IntelligentDeadReckoner(ref_lat=self.ref_lat, ref_lon=self.ref_lon, ref_alt=self.ref_alt)
        self.outage_detector = GNSSOutageDetector()
        self.ekf = EKFNavFusion(ref_lat=self.ref_lat, ref_lon=self.ref_lon, ref_alt=self.ref_alt)
        self.ai_classifier = EdgeAIClassifier()

        # Telemetry Buffers
        self.latest_imu: Optional[Imu] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_gnss: Optional[NavSatFix] = None
        self.has_synced_init = False
        self.last_step_time = time.time()
        self.last_reported_mode: Optional[str] = None

        # Path history for RViz display (max 300 points)
        self.fused_path_msg = Path()
        self.fused_path_msg.header.frame_id = 'map'
        self.idr_path_msg = Path()
        self.idr_path_msg.header.frame_id = 'map'

        # Publishers
        self.fused_odom_pub = self.create_publisher(Odometry, 'odometry/fused', 10)
        self.idr_odom_pub = self.create_publisher(Odometry, 'idr_odometry', 10)
        self.status_pub = self.create_publisher(String, 'nav_status', 10)
        self.fused_path_pub = self.create_publisher(Path, 'path_fused', 10)
        self.idr_path_pub = self.create_publisher(Path, 'path_idr', 10)
        self.marker_pub = self.create_publisher(Marker, 'status_marker', 10)

        # Subscribers
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(NavSatFix, 'gps/fix', self.gnss_callback, 10)

        # Timer (50 Hz)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info('Edge AI AMR Fusion Node initialized and listening to /imu, /odom, /gps/fix')

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def gnss_callback(self, msg: NavSatFix):
        self.latest_gnss = msg

    def timer_callback(self):
        # Never manufacture a startup outage before the first receiver fix arrives.
        if self.latest_imu is None or self.latest_odom is None or self.latest_gnss is None:
            return

        now_ros = self.get_clock().now()
        now_msg = now_ros.to_msg()
        now_sec = time.time()
        dt = max(0.001, min(0.1, now_sec - self.last_step_time))
        self.last_step_time = now_sec

        # 1. Extract IMU data from Gazebo msg
        ax = self.latest_imu.linear_acceleration.x
        ay = self.latest_imu.linear_acceleration.y
        az = self.latest_imu.linear_acceleration.z
        gx = self.latest_imu.angular_velocity.x
        gy = self.latest_imu.angular_velocity.y
        gz = self.latest_imu.angular_velocity.z

        imu_data = IMUData(ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz, timestamp=now_sec)

        # 2. Extract GNSS data
        if self.latest_gnss is not None and self.latest_gnss.status.status != NavSatStatus.STATUS_NO_FIX:
            var = self.latest_gnss.position_covariance[0]
            hdop = max(0.6, math.sqrt(max(0.1, var)))
            sats = 11 if hdop < 1.5 else 5
            gnss_data = GNSSData(
                lat=self.latest_gnss.latitude,
                lon=self.latest_gnss.longitude,
                alt=self.latest_gnss.altitude,
                speed=math.sqrt(self.latest_odom.twist.twist.linear.x**2 + self.latest_odom.twist.twist.linear.y**2),
                heading=0.0,
                num_sats=sats,
                hdop=hdop,
                fix_type=1,
                valid=True,
                timestamp=now_sec
            )
            is_outage_flag = False
        else:
            gnss_data = GNSSData(
                lat=0.0, lon=0.0, alt=0.0, speed=0.0, heading=0.0,
                num_sats=0, hdop=99.0, fix_type=0, valid=False, timestamp=now_sec
            )
            is_outage_flag = True

        sensor_frame = SensorFrame(
            timestamp=now_sec,
            gnss=gnss_data,
            imu=imu_data,
            is_simulated_outage=is_outage_flag,
            source="gazebo_amr"
        )

        # 3. IMU Preprocessing (ZUPT & Bias Calibration)
        is_stationary, cal_imu, tilt_angles = self.imu_prep.process(imu_data)
        # Check wheel odometry to reinforce ZUPT
        wheel_speed = math.sqrt(self.latest_odom.twist.twist.linear.x**2 + self.latest_odom.twist.twist.linear.y**2)
        if wheel_speed < 0.02:
            is_stationary = True

        roll_rad, pitch_rad = tilt_angles

        # 4. Edge AI Condition Classification
        ai_cond, ai_conf, adaptive_scale, _ = self.ai_classifier.classify(sensor_frame, est_speed=wheel_speed)

        # 5. GNSS Outage Detection
        nav_mode, reason = self.outage_detector.update(sensor_frame, ai_condition=ai_cond)
        is_outage = (nav_mode == GNSSOutageDetector.MODE_IDR_ACTIVE)
        if nav_mode != self.last_reported_mode:
            self.get_logger().warning(f"LOCALIZATION_TRANSITION {self.last_reported_mode or 'START'} -> {nav_mode}: {reason}")
            self.last_reported_mode = nav_mode

        # 6. Attitude Filter
        roll_deg, pitch_deg, heading_deg = self.att_filter.update(
            cal_imu, tilt_angles, gnss_heading_deg=None, gnss_valid=False, speed=wheel_speed
        )

        # 7. Initialize Filters upon first valid GNSS / Odom
        if not self.has_synced_init:
            if gnss_data.valid:
                self.idr.sync_with_gnss(gnss_data)
                self.ekf.initialize(gnss_data.lat, gnss_data.lon, gnss_data.alt, heading_deg)
            else:
                # Use current odom pose
                ox = self.latest_odom.pose.pose.position.x
                oy = self.latest_odom.pose.pose.position.y
                lat, lon, alt = enu_to_wgs84(ox, oy, self.ref_lat, self.ref_lon, self.ref_alt)
                self.ekf.initialize(lat, lon, alt, heading_deg)
            self.has_synced_init = True

        # 8. Intelligent Dead Reckoning (IDR) Position Propagation
        idr_lat, idr_lon, idr_e, idr_n, idr_speed = self.idr.update(
            cal_imu, heading_deg, pitch_rad, is_stationary, is_outage
        )

        # 9. Extended Kalman Filter (EKF) Fusion
        self.ekf.predict(cal_imu, pitch_rad, is_stationary, dt)
        if gnss_data.valid and not is_outage:
            self.ekf.update_gnss(gnss_data, adaptive_scale=adaptive_scale)

        fused_lat, fused_lon, fused_e, fused_n, fused_speed, fused_heading = self.ekf.get_nav_state()

        # Heading to ROS ENU yaw (yaw = pi/2 - heading_rad)
        yaw_rad = (math.pi / 2.0) - math.radians(fused_heading)
        q_rot = euler_to_quaternion(0.0, 0.0, yaw_rad)

        # 10. Publish /amr/odometry/fused
        odom_fused = Odometry()
        odom_fused.header.stamp = now_msg
        odom_fused.header.frame_id = 'map'
        odom_fused.child_frame_id = self.base_frame
        odom_fused.pose.pose.position.x = fused_e
        odom_fused.pose.pose.position.y = fused_n
        odom_fused.pose.pose.position.z = 0.0
        odom_fused.pose.pose.orientation = q_rot
        odom_fused.twist.twist.linear.x = fused_speed
        odom_fused.twist.twist.angular.z = cal_imu.gz
        self.fused_odom_pub.publish(odom_fused)

        # 11. Publish /amr/idr_odometry
        idr_yaw = (math.pi / 2.0) - self.idr.heading_rad
        q_idr = euler_to_quaternion(0.0, 0.0, idr_yaw)
        odom_idr = Odometry()
        odom_idr.header.stamp = now_msg
        odom_idr.header.frame_id = 'map'
        odom_idr.child_frame_id = self.base_frame
        odom_idr.pose.pose.position.x = idr_e
        odom_idr.pose.pose.position.y = idr_n
        odom_idr.pose.pose.orientation = q_idr
        odom_idr.twist.twist.linear.x = idr_speed
        self.idr_odom_pub.publish(odom_idr)

        # 12. Publish Paths for RViz
        p_fused = PoseStamped()
        p_fused.header.stamp = now_msg
        p_fused.header.frame_id = 'map'
        p_fused.pose.position.x = fused_e
        p_fused.pose.position.y = fused_n
        p_fused.pose.orientation = q_rot
        self.fused_path_msg.poses.append(p_fused)
        if len(self.fused_path_msg.poses) > 300:
            self.fused_path_msg.poses.pop(0)
        self.fused_path_msg.header.stamp = now_msg
        self.fused_path_pub.publish(self.fused_path_msg)

        p_idr = PoseStamped()
        p_idr.header.stamp = now_msg
        p_idr.header.frame_id = 'map'
        p_idr.pose.position.x = idr_e
        p_idr.pose.position.y = idr_n
        p_idr.pose.orientation = q_idr
        self.idr_path_msg.poses.append(p_idr)
        if len(self.idr_path_msg.poses) > 300:
            self.idr_path_msg.poses.pop(0)
        self.idr_path_msg.header.stamp = now_msg
        self.idr_path_pub.publish(self.idr_path_msg)

        # 13. Publish 3D Floating Status Banner above AMR in RViz
        marker = Marker()
        marker.header.stamp = now_msg
        marker.header.frame_id = self.base_frame
        marker.ns = 'amr_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.65  # 65cm above robot
        marker.scale.z = 0.16         # Text height

        if nav_mode == "GNSS_FIX":
            marker.text = f"[ ● GNSS ACTIVE ]\nAI: {ai_cond}"
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.2
        elif nav_mode == "IDR_ACTIVE":
            marker.text = f"[ ⚡ IDR ACTIVE (OUTAGE) ]\nDrift: {self.idr.accumulated_drift:.2f}m | AI: {ai_cond}"
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
        elif nav_mode == "FUSION_CONVERGING":
            marker.text = f"[ 🔄 FUSION RECOVERY ]\nReconciling Drift... | AI: {ai_cond}"
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
        else:
            marker.text = f"[ {nav_mode} ]\nAI: {ai_cond}"
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
        marker.color.a = 0.95
        self.marker_pub.publish(marker)

        # 14. Publish JSON Status topic for HUD / Diagnostics
        status_dict = {
            "mode": nav_mode,
            "ai_condition": ai_cond,
            "ai_confidence": round(ai_conf, 3),
            "outage_duration_s": round(self.outage_detector.outage_duration, 2),
            "accumulated_drift_m": round(self.idr.accumulated_drift, 2),
            "fused_x": round(fused_e, 3),
            "fused_y": round(fused_n, 3),
            "idr_x": round(idr_e, 3),
            "idr_y": round(idr_n, 3),
            "speed_mps": round(fused_speed, 2),
            "heading_deg": round(fused_heading, 1),
            "zupt": is_stationary,
            "latency_ms": round(self.ai_classifier.last_latency_ms, 2),
            "localization_confidence": round(
                max(0.55, 0.92 - 0.012 * self.outage_detector.outage_duration)
                if nav_mode == "IDR_ACTIVE" else (0.85 if nav_mode == "FUSION_CONVERGING" else 0.98), 3
            )
        }
        status_msg = String()
        status_msg.data = json.dumps(status_dict)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeAIAMRFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
