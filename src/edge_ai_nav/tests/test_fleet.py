"""Tests for deterministic five-AMR coordination logic."""

import time
import unittest

from edge_ai_nav.fleet.conflict_detector import predict_conflict, safety_radius, select_winner
from edge_ai_nav.fleet.peer_state import PeerState


def state(robot_id, x, y, zone, priority=1, mode="GNSS_FIX", confidence=0.98):
    return PeerState(
        robot_id=robot_id, timestamp=time.time(), x=x, y=y, heading=0.0,
        linear_velocity=0.25, current_goal=zone, intended_zone=zone,
        task_priority=priority, waiting_time=0.0, localization_mode=mode,
        localization_confidence=confidence, coordination_state="GO",
    )


class TestFleetCoordination(unittest.TestCase):
    def test_peer_state_json_roundtrip(self):
        original = state("amr_alpha", -1.0, 0.0, "intersection_A")
        self.assertEqual(original, PeerState.from_json(original.to_json()))

    def test_both_peers_choose_same_priority_winner(self):
        alpha = state("amr_alpha", -1.0, 0.0, "intersection_A", priority=1)
        bravo = state("amr_bravo", 0.0, 1.0, "intersection_A", priority=2)
        self.assertEqual("amr_bravo", select_winner(alpha, bravo, "intersection_A"))
        self.assertEqual("amr_bravo", select_winner(bravo, alpha, "intersection_A"))

    def test_robot_id_breaks_symmetric_deadlock(self):
        delta = state("amr_delta", -0.8, -7.0, "narrow_aisle_1")
        echo = state("amr_echo", 0.8, -7.0, "narrow_aisle_1")
        self.assertEqual("amr_delta", select_winner(delta, echo, "narrow_aisle_1"))
        self.assertEqual("amr_delta", select_winner(echo, delta, "narrow_aisle_1"))

        # Waiting longer must never make the peers swap winners mid-conflict.
        echo.waiting_time = 30.0
        self.assertEqual("amr_delta", select_winner(delta, echo, "narrow_aisle_1"))
        self.assertEqual("amr_delta", select_winner(echo, delta, "narrow_aisle_1"))

    def test_shared_zone_conflict_is_predicted(self):
        alpha = state("amr_alpha", -1.0, 0.0, "intersection_A")
        bravo = state("amr_bravo", 0.0, 1.0, "intersection_A")
        conflict = predict_conflict(alpha, [bravo])
        self.assertIsNotNone(conflict)
        self.assertEqual("intersection_A", conflict.zone)

    def test_idr_expands_safety_radius(self):
        normal = state("amr_bravo", 0.0, 1.0, "intersection_B")
        idr = state("amr_charlie", -1.0, 7.0, "intersection_B", mode="IDR_ACTIVE", confidence=0.60)
        self.assertAlmostEqual(0.50, safety_radius(normal), places=2)
        self.assertGreaterEqual(safety_radius(idr), 0.80)


if __name__ == "__main__":
    unittest.main()
