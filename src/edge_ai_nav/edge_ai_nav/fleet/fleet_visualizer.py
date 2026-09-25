"""Read-only fleet observer, RViz labels, and run-derived metrics."""

import json
import math
import time
from typing import Dict, Set, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .missions import ROBOTS
from .peer_state import PeerState


class FleetVisualizer(Node):
    def __init__(self):
        super().__init__("fleet_visualizer_read_only")
        self.peers: Dict[str, PeerState] = {}
        self.latencies = []
        self.collisions = 0
        self.deadlock_resolutions = 0
        self.completed: Set[str] = set()
        self.active_collision_pairs: Set[Tuple[str, str]] = set()
        self.marker_pub = self.create_publisher(MarkerArray, "/fleet/markers", 10)
        self.metrics_pub = self.create_publisher(String, "/fleet/metrics", 10)
        self.create_subscription(String, "/fleet/peer_state", self._peer_cb, 100)
        self.create_subscription(String, "/fleet/events", self._event_cb, 100)
        self.create_timer(0.25, self._publish)
        self.create_timer(5.0, self._print_summary)
        self.get_logger().info("Passive fleet observer online (READ-ONLY: publishes no motion commands)")

    def _peer_cb(self, msg: String) -> None:
        try:
            state = PeerState.from_json(msg.data)
        except Exception:
            return
        self.peers[state.robot_id] = state

    def _event_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = data.get("event")
        if event == "CONFLICT_DETECTED" and "decision_latency_ms" in data:
            self.latencies.append(float(data["decision_latency_ms"]))
        elif event == "DEADLOCK_PROTECTION_ACTIVATED":
            self.deadlock_resolutions += 1
        elif event == "GOAL_COMPLETED":
            self.completed.add(str(data.get("robot_id")))

    def _metrics(self):
        now = time.time()
        live = {rid: p for rid, p in self.peers.items() if now - p.timestamp < 2.0}
        new_pairs: Set[Tuple[str, str]] = set()
        ids = sorted(live)
        for i, left in enumerate(ids):
            for right in ids[i + 1:]:
                if math.hypot(live[left].x - live[right].x, live[left].y - live[right].y) < 0.30:
                    new_pairs.add((left, right))
        self.collisions += len(new_pairs - self.active_collision_pairs)
        self.active_collision_pairs = new_pairs
        return {
            "peer_node_count": len(live),
            "average_decision_latency_ms": round(sum(self.latencies) / len(self.latencies), 3) if self.latencies else 0.0,
            "decision_count": len(self.latencies),
            "collisions": self.collisions,
            "deadlock_resolutions": self.deadlock_resolutions,
            "completed_goals": len(self.completed),
        }

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for index, (rid, state) in enumerate(sorted(self.peers.items())):
            if rid not in ROBOTS or time.time() - state.timestamp > 2.0:
                continue
            spec = ROBOTS[rid]
            color = spec["color"]
            body = Marker()
            body.header.frame_id = "map"
            body.header.stamp = now
            body.ns = "fleet_bodies"
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = state.x
            body.pose.position.y = state.y
            body.pose.position.z = 0.32
            body.scale.x = body.scale.y = 0.34
            body.scale.z = 0.08
            body.color.r, body.color.g, body.color.b = color
            body.color.a = 0.95
            markers.markers.append(body)

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = now
            text.ns = "fleet_labels"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = state.x
            text.pose.position.y = state.y
            text.pose.position.z = 0.72
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = (
                f"{spec['label']} | {state.coordination_state}\n"
                f"{state.localization_mode} {state.localization_confidence:.0%}\n"
                f"task→{state.current_goal}"
            )
            markers.markers.append(text)
        self.marker_pub.publish(markers)
        msg = String()
        msg.data = json.dumps(self._metrics(), sort_keys=True)
        self.metrics_pub.publish(msg)

    def _print_summary(self) -> None:
        metrics = self._metrics()
        states = ", ".join(
            f"{ROBOTS[rid]['label']}={peer.coordination_state}/{peer.localization_mode}"
            for rid, peer in sorted(self.peers.items()) if rid in ROBOTS
        )
        self.get_logger().info(f"FLEET {states} | METRICS {json.dumps(metrics, sort_keys=True)}")


def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._print_summary()
        node.destroy_node()
        rclpy.shutdown()
