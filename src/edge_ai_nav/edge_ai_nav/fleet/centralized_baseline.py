"""Comparison-only centralized right-of-way service."""

import json
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .conflict_detector import predict_conflict, select_winner
from .peer_state import PeerState


class CentralizedBaseline(Node):
    def __init__(self):
        super().__init__("central_fleet_coordinator")
        self.declare_parameter("simulated_network_delay_ms", 0.0)
        self.declare_parameter("failure_after_s", 45.0)
        self.delay_ms = max(0.0, float(self.get_parameter("simulated_network_delay_ms").value))
        self.failure_after = float(self.get_parameter("failure_after_s").value)
        self.started = time.monotonic()
        self.offline_announced = False
        self.peers: Dict[str, PeerState] = {}
        self.pending: List[Tuple[float, str]] = []
        self.pub = self.create_publisher(String, "/fleet/central_decision", 20)
        self.event_pub = self.create_publisher(String, "/fleet/events", 20)
        self.create_subscription(String, "/fleet/peer_state", self._peer_cb, 50)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            f"CENTRALIZED BASELINE active | SIMULATED network delay={self.delay_ms:.1f} ms | "
            f"intentional failure after={self.failure_after:.1f} s"
        )

    def _peer_cb(self, msg: String) -> None:
        try:
            peer = PeerState.from_json(msg.data)
        except Exception:
            return
        self.peers[peer.robot_id] = peer

    def _event(self, event: str, **fields) -> None:
        msg = String()
        msg.data = json.dumps({"timestamp": time.time(), "robot_id": "central", "event": event, **fields})
        self.event_pub.publish(msg)

    def _tick(self) -> None:
        now_mono = time.monotonic()
        if self.failure_after > 0.0 and now_mono - self.started >= self.failure_after:
            if not self.offline_announced:
                self.offline_announced = True
                self.get_logger().error("CENTRAL COORDINATOR OFFLINE - baseline single point of failure")
                self._event("CENTRAL_COORDINATOR_OFFLINE")
            return

        now_wall = time.time()
        states = {rid: state for rid, state in self.peers.items() if now_wall - state.timestamp < 2.0}
        decisions = {rid: ("GO", "central path clear") for rid in states}
        for rid, state in sorted(states.items()):
            conflict = predict_conflict(state, (p for pid, p in states.items() if pid != rid))
            if conflict is None or conflict.peer_id not in states:
                continue
            peer = states[conflict.peer_id]
            winner = select_winner(state, peer, conflict.zone)
            loser = peer.robot_id if winner == state.robot_id else state.robot_id
            decisions[winner] = ("GO", f"central winner={winner} at {conflict.zone}")
            decisions[loser] = ("YIELD", f"central winner={winner} at {conflict.zone}")

        release = now_mono + self.delay_ms / 1000.0
        for rid, (decision, reason) in decisions.items():
            payload = json.dumps({
                "robot_id": rid, "decision": decision, "reason": reason,
                "simulated_network_delay_ms": self.delay_ms,
                "delay_is_simulated": True,
            })
            self.pending.append((release, payload))
        ready = [item for item in self.pending if item[0] <= now_mono]
        self.pending = [item for item in self.pending if item[0] > now_mono]
        for _, payload in ready:
            msg = String()
            msg.data = payload
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CentralizedBaseline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
