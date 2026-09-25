"""Per-robot waypoint, LiDAR safety, and fleet coordination node."""

import json
import math
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .conflict_detector import predict_conflict, safety_radius, select_winner
from .missions import ROBOTS
from .peer_state import PeerState


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_error(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


class FleetRobotNode(Node):
    """A fully local controller; in decentralized mode it commands only itself."""

    def __init__(self):
        super().__init__("fleet_local_coordinator")
        self.declare_parameter("robot_id", "amr_alpha")
        self.declare_parameter("mode", "decentralized")
        self.declare_parameter("odom_is_local", True)
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.mode = str(self.get_parameter("mode").value)
        self.odom_is_local = bool(self.get_parameter("odom_is_local").value)
        if self.robot_id not in ROBOTS:
            raise ValueError(f"Unknown robot_id: {self.robot_id}")

        self.spec = ROBOTS[self.robot_id]
        self.spawn_x, self.spawn_y, self.spawn_yaw = self.spec["spawn"]
        self.x, self.y, self.heading = self.spawn_x, self.spawn_y, self.spawn_yaw
        self.speed = 0.0
        self.have_odom = False
        self.scan: Optional[LaserScan] = None
        self.peers: Dict[str, PeerState] = {}
        self.localization_mode = "GNSS_FIX"
        self.localization_confidence = 1.0
        self.outage_duration = 0.0
        self.state = "WAITING"
        self.waypoint_index = 0
        self.started_at = time.monotonic()
        self.wait_started: Optional[float] = None
        self.last_conflict_key = ""
        self.last_central_decision = 0.0
        self.central_decision = "STOP"
        self.central_reason = "awaiting coordinator"
        self.avoid_until = 0.0
        self.avoid_direction = 1.0
        self.obstacle_event_active = False
        self.completed_reported = False

        self.cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.peer_pub = self.create_publisher(String, "/fleet/peer_state", 20)
        self.event_pub = self.create_publisher(String, "/fleet/events", 20)
        self.create_subscription(Odometry, "odom", self._odom_cb, 20)
        self.create_subscription(LaserScan, "scan", self._scan_cb, 10)
        self.create_subscription(String, "/fleet/peer_state", self._peer_cb, 50)
        self.create_subscription(String, "/fleet/central_decision", self._central_cb, 20)
        if self.robot_id == "amr_charlie":
            self.create_subscription(String, "nav_status", self._nav_status_cb, 10)
        self.timer = self.create_timer(0.1, self._tick)
        architecture = "NO CENTRAL COORDINATOR REQUIRED" if self.mode == "decentralized" else "CENTRAL BASELINE"
        self.get_logger().info(f"[{self.spec['label']}] online | {architecture}")

    def _odom_cb(self, msg: Odometry) -> None:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = _yaw(msg.pose.pose.orientation)
        if self.odom_is_local:
            c, s = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
            self.x = self.spawn_x + c * px - s * py
            self.y = self.spawn_y + s * px + c * py
            self.heading = self.spawn_yaw + yaw
        else:
            self.x, self.y, self.heading = px, py, yaw
        self.speed = float(msg.twist.twist.linear.x)
        self.have_odom = True
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = f"{self.robot_id}/odom"
        transform.child_frame_id = f"{self.robot_id}/base_footprint"
        transform.transform.translation.x = px
        transform.transform.translation.y = py
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _peer_cb(self, msg: String) -> None:
        try:
            peer = PeerState.from_json(msg.data)
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            return
        if peer.robot_id != self.robot_id:
            self.peers[peer.robot_id] = peer

    def _central_cb(self, msg: String) -> None:
        if self.mode != "centralized":
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("robot_id") == self.robot_id:
            self.central_decision = str(data.get("decision", "STOP"))
            self.central_reason = str(data.get("reason", "central decision"))
            self.last_central_decision = time.monotonic()

    def _nav_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.localization_mode = str(data.get("mode", self.localization_mode))
        self.outage_duration = float(data.get("outage_duration_s", 0.0))
        if self.localization_mode == "IDR_ACTIVE":
            self.localization_confidence = max(0.55, 0.92 - 0.012 * self.outage_duration)
        elif self.localization_mode == "GNSS_DEGRADED":
            self.localization_confidence = 0.78
        elif self.localization_mode == "FUSION_CONVERGING":
            self.localization_confidence = 0.85
        else:
            self.localization_confidence = 0.98

    def _goal(self):
        if self.waypoint_index >= len(self.spec["waypoints"]):
            return None
        return self.spec["waypoints"][self.waypoint_index]

    def _intended_zone(self) -> str:
        for waypoint in self.spec["waypoints"][self.waypoint_index:]:
            if waypoint[2]:
                return waypoint[2]
        return ""

    def _waiting_time(self) -> float:
        return 0.0 if self.wait_started is None else time.monotonic() - self.wait_started

    def _state_message(self, now: float) -> PeerState:
        goal = self._goal()
        return PeerState(
            robot_id=self.robot_id,
            timestamp=now,
            x=self.x,
            y=self.y,
            heading=self.heading,
            linear_velocity=self.speed,
            current_goal="complete" if goal is None else f"{goal[0]:.1f},{goal[1]:.1f}",
            intended_zone=self._intended_zone(),
            task_priority=int(self.spec["priority"]),
            waiting_time=self._waiting_time(),
            localization_mode=self.localization_mode,
            localization_confidence=self.localization_confidence,
            coordination_state=self.state,
        )

    def _publish_event(self, event: str, **fields) -> None:
        payload = {"timestamp": time.time(), "robot_id": self.robot_id, "event": event, **fields}
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.event_pub.publish(msg)

    def _scan_clearances(self):
        if self.scan is None or not self.scan.ranges:
            return math.inf, math.inf, math.inf
        front, left, right = [], [], []
        for index, value in enumerate(self.scan.ranges):
            if not math.isfinite(value) or value < self.scan.range_min:
                continue
            angle = self.scan.angle_min + index * self.scan.angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if abs(angle) <= 0.35:
                front.append(value)
            elif 0.35 < angle < 1.40:
                left.append(value)
            elif -1.40 < angle < -0.35:
                right.append(value)
        return min(front, default=math.inf), min(left, default=math.inf), min(right, default=math.inf)

    def _publish_cmd(self, linear: float = 0.0, angular: float = 0.0) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_id}/base_footprint"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _tick(self) -> None:
        wall_now = time.time()
        monotonic_now = time.monotonic()
        me = self._state_message(wall_now)
        peer_msg = String()
        peer_msg.data = me.to_json()
        self.peer_pub.publish(peer_msg)

        if monotonic_now - self.started_at < float(self.spec["delay"]):
            self.state = "WAITING"
            self._publish_cmd()
            return
        goal = self._goal()
        if goal is None:
            self.state = "COMPLETE"
            self._publish_cmd()
            if not self.completed_reported:
                self.completed_reported = True
                self._publish_event("GOAL_COMPLETED")
                self.get_logger().info(f"[{self.spec['label']}] mission complete")
            return

        decision_start = time.perf_counter()
        conflict = predict_conflict(me, self.peers.values())
        decision = "GO"
        reason = "path clear"
        if self.mode == "centralized":
            if monotonic_now - self.last_central_decision > 1.5:
                decision, reason = "STOP", "CENTRAL COORDINATOR OFFLINE - fail safe"
            else:
                decision, reason = self.central_decision, self.central_reason
        elif conflict is not None:
            peer = self.peers.get(conflict.peer_id)
            if peer is not None:
                winner = select_winner(me, peer, conflict.zone)
                decision = "GO" if winner == self.robot_id else "YIELD"
                reason = f"winner={winner}; deterministic priority/confidence/robot-ID tie-break"
                latency_ms = (time.perf_counter() - decision_start) * 1000.0
                key = f"{conflict.peer_id}:{conflict.zone}:{decision}"
                if key != self.last_conflict_key:
                    self.last_conflict_key = key
                    self._publish_event(
                        "CONFLICT_DETECTED", peer=conflict.peer_id, zone=conflict.zone,
                        decision=decision, winner=winner, decision_latency_ms=latency_ms,
                        reason=reason, safety_radius=conflict.safety_radius,
                    )
                    self.get_logger().warning(
                        f"[{self.spec['label']}] CONFLICT_DETECTED peer={conflict.peer_id} "
                        f"zone={conflict.zone} DECISION={decision} {reason} "
                        f"DECISION_LATENCY_MS={latency_ms:.3f} safety_radius={conflict.safety_radius:.2f}m"
                    )
                    if conflict.zone == "narrow_aisle_1":
                        self._publish_event("DEADLOCK_PROTECTION_ACTIVATED", winner=winner)
            else:
                decision = "SLOW"
        else:
            self.last_conflict_key = ""

        if decision in ("YIELD", "STOP"):
            self.state = decision
            self.wait_started = self.wait_started or monotonic_now
            self._publish_cmd()
            return
        self.wait_started = None

        front, left, right = self._scan_clearances()
        if monotonic_now < self.avoid_until:
            self.state = "AVOID"
            self._publish_cmd(0.12, 0.75 * self.avoid_direction)
            return
        if front < 0.55:
            self.avoid_direction = 1.0 if left >= right else -1.0
            self.avoid_until = monotonic_now + 1.6
            self.state = "AVOID"
            if not self.obstacle_event_active:
                self.obstacle_event_active = True
                self._publish_event("LIDAR_OBSTACLE_DETECTED", range_m=front, response="controlled_turn")
                self.get_logger().warning(
                    f"[{self.spec['label']}] LiDAR obstacle {front:.2f}m - AVOID controlled turn"
                )
            self._publish_cmd(0.0, 0.75 * self.avoid_direction)
            return
        self.obstacle_event_active = False

        dx, dy = goal[0] - self.x, goal[1] - self.y
        distance = math.hypot(dx, dy)
        if distance < 0.34:
            self.waypoint_index += 1
            self._publish_event("WAYPOINT_REACHED", waypoint=self.waypoint_index)
            self._publish_cmd()
            return
        target_heading = math.atan2(dy, dx)
        error = _angle_error(target_heading, self.heading)
        max_speed = 0.18 if self.localization_mode == "IDR_ACTIVE" else 0.32
        if decision == "SLOW":
            max_speed *= 0.45
        linear = 0.03 if abs(error) > 0.65 else min(max_speed, 0.28 * distance)
        angular = max(-1.0, min(1.0, 1.8 * error))
        self.state = "IDR" if self.localization_mode == "IDR_ACTIVE" else decision
        self._publish_cmd(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = FleetRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
