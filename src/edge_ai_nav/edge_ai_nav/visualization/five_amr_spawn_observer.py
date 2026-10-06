"""Read-only Stage 2 observer: isolated TF trees and floating name labels."""

import os
import math
from typing import Dict

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class FiveAMRSpawnObserver(Node):
    """Observe odometry and publish visualization/TF only; never commands robots."""

    def __init__(self):
        super().__init__("five_amr_spawn_observer_read_only")
        self.declare_parameter("config_file", "")
        self.declare_parameter("slam_robot_id", "")
        self.declare_parameter("world_odom", False)
        config_file = str(self.get_parameter("config_file").value)
        self.slam_robot_id = str(self.get_parameter("slam_robot_id").value)
        self.world_odom = bool(self.get_parameter("world_odom").value)
        if not config_file or not os.path.isfile(config_file):
            raise FileNotFoundError(f"warehouse config not found: {config_file}")
        with open(config_file, "r", encoding="utf-8") as stream:
            warehouse = yaml.safe_load(stream)["warehouse"]
        self.spawns = warehouse["robot_spawns"]
        self.physical_scale=float(warehouse.get('scale_speed_upgrade',{}).get('linear_scale',1.0))
        low_lidar=warehouse.get('scale_speed_upgrade',{}).get('low_lidar_pose_m')
        self.low_lidar_pose=tuple(map(float,low_lidar)) if low_lidar else None
        self.visuals = warehouse["robot_visuals"]
        self.latest: Dict[str, Odometry] = {}
        self.dynamic_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "/fleet/spawn_labels", 10)

        for robot_id in self.spawns:
            self.create_subscription(
                Odometry, f"/{robot_id}/odom",
                lambda msg, rid=robot_id: self._odom(rid, msg), 20,
            )
        self._publish_static_trees()
        self.create_timer(0.2, self._publish_labels)
        self.get_logger().info(
            "Stage 2 observer active (READ-ONLY): five odometry inputs, TF and labels only"
        )

    def _publish_static_trees(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for robot_id, (x, y, yaw) in self.spawns.items():
            # slam_toolbox exclusively owns map -> ALPHA odom in the live
            # mapping demo.  A static seed for that same edge is a TF conflict.
            if robot_id != self.slam_robot_id:
                map_to_odom = TransformStamped()
                map_to_odom.header.stamp = stamp
                map_to_odom.header.frame_id = "map"
                map_to_odom.child_frame_id = f"{robot_id}/odom"
                # V2 physical odometry already reports warehouse world X/Y.
                map_to_odom.transform.translation.x = 0.0 if self.world_odom else float(x)
                map_to_odom.transform.translation.y = 0.0 if self.world_odom else float(y)
                map_to_odom.transform.rotation.z = 0.0 if self.world_odom else math.sin(float(yaw) / 2.0)
                map_to_odom.transform.rotation.w = 1.0 if self.world_odom else math.cos(float(yaw) / 2.0)
                transforms.append(map_to_odom)

            links = (("base_footprint", "base_link", (0.0, 0.0, 0.0)),
                     ("base_link", "base_scan", (0.12, 0.0, 0.33)),
                     ("base_link", "imu_link", (0.0, 0.0, 0.25))) if self.world_odom else (
                     ("base_footprint", "base_link", (0.0, 0.0, 0.01)),
                     ("base_link", "base_scan", (-0.064, 0.0, 0.271)),
                     ("base_link", "imu_link", (0.0, 0.0, 0.068)))
            if self.world_odom and self.low_lidar_pose:
                links += (("base_link", "base_scan_low",
                           tuple(value/self.physical_scale for value in self.low_lidar_pose)),)
            for parent, child, xyz in links:
                if self.world_odom:
                    xyz=tuple(value*self.physical_scale for value in xyz)
                static = TransformStamped()
                static.header.stamp = stamp
                static.header.frame_id = f"{robot_id}/{parent}"
                static.child_frame_id = f"{robot_id}/{child}"
                static.transform.translation.x = xyz[0]
                static.transform.translation.y = xyz[1]
                static.transform.translation.z = xyz[2]
                static.transform.rotation.w = 1.0
                transforms.append(static)
        self.static_tf.sendTransform(transforms)

    def _odom(self, robot_id: str, msg: Odometry) -> None:
        self.latest[robot_id] = msg
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = f"{robot_id}/odom"
        tf.child_frame_id = f"{robot_id}/base_footprint"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.dynamic_tf.sendTransform(tf)

    def _publish_labels(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, robot_id in enumerate(self.spawns):
            if robot_id not in self.latest:
                continue
            odom = self.latest[robot_id]
            spawn_x, spawn_y, spawn_yaw = self.spawns[robot_id]
            local_x = odom.pose.pose.position.x
            local_y = odom.pose.pose.position.y
            world_x = (local_x if self.world_odom else
                       spawn_x + math.cos(spawn_yaw) * local_x - math.sin(spawn_yaw) * local_y)
            world_y = (local_y if self.world_odom else
                       spawn_y + math.sin(spawn_yaw) * local_x + math.cos(spawn_yaw) * local_y)
            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = "map"
            label.ns = "stage2_robot_names"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = world_x
            label.pose.position.y = world_y
            label.pose.position.z = 0.78*self.physical_scale
            label.scale.z = 0.26
            colour = self.visuals[robot_id]["colour"]
            label.color.r, label.color.g, label.color.b = map(float, colour)
            label.color.a = 1.0
            label.text = self.visuals[robot_id]["label"]
            markers.markers.append(label)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FiveAMRSpawnObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
