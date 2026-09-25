"""Deterministic local conflict prediction and negotiation rules."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Tuple

from .peer_state import PeerState


ZONES: Dict[str, Tuple[float, float, float]] = {
    "intersection_A": (0.0, 0.0, 1.25),
    "intersection_B": (0.0, 7.0, 1.35),
    "narrow_aisle_1": (0.0, -7.0, 1.20),
}


@dataclass(frozen=True)
class Conflict:
    peer_id: str
    zone: str
    distance: float
    safety_radius: float


def safety_radius(state: PeerState) -> float:
    """Expand separation conservatively as localization confidence falls."""
    confidence = max(0.0, min(1.0, state.localization_confidence))
    if state.localization_mode == "IDR_ACTIVE":
        return 0.75 + 0.15 * (1.0 - confidence)
    return 0.50 + 0.10 * (1.0 - confidence)


def distance_to_zone(state: PeerState, zone: str) -> float:
    zx, zy, _ = ZONES[zone]
    return math.hypot(state.x - zx, state.y - zy)


def predict_conflict(me: PeerState, peers: Iterable[PeerState]) -> Optional[Conflict]:
    """Predict shared-zone or near-future proximity conflicts locally."""
    freshest = sorted(peers, key=lambda p: p.robot_id)
    for peer in freshest:
        if (
            peer.robot_id == me.robot_id
            or peer.coordination_state == "COMPLETE"
            or abs(me.timestamp - peer.timestamp) > 2.0
        ):
            continue
        radius = max(safety_radius(me), safety_radius(peer))
        separation = math.hypot(me.x - peer.x, me.y - peer.y)
        if me.intended_zone and me.intended_zone == peer.intended_zone:
            zone = me.intended_zone
            if zone in ZONES:
                approach = max(distance_to_zone(me, zone), distance_to_zone(peer, zone))
                if approach <= 5.5:
                    return Conflict(peer.robot_id, zone, separation, radius)
        horizon = 2.5
        me_px = me.x + horizon * me.linear_velocity * math.cos(me.heading)
        me_py = me.y + horizon * me.linear_velocity * math.sin(me.heading)
        peer_px = peer.x + horizon * peer.linear_velocity * math.cos(peer.heading)
        peer_py = peer.y + horizon * peer.linear_velocity * math.sin(peer.heading)
        if math.hypot(me_px - peer_px, me_py - peer_py) < radius:
            return Conflict(peer.robot_id, "predicted_proximity", separation, radius)
    return None


def _rank(state: PeerState, zone: str) -> tuple:
    # Priority and confidence are stable for a conflict; robot ID is the invariant
    # final tie-break. Waiting time is deliberately not allowed to swap the winner.
    priority = state.task_priority
    eta = distance_to_zone(state, zone) if zone in ZONES else 999.0
    return (-priority, -round(state.localization_confidence, 1), state.robot_id, round(eta, 1))


def select_winner(me: PeerState, peer: PeerState, zone: str) -> str:
    """Both peers produce the same winner from the same two published states."""
    return min((me, peer), key=lambda state: _rank(state, zone)).robot_id
