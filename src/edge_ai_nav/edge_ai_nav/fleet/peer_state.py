"""Compact JSON peer-state representation carried on ROS 2 DDS."""

from dataclasses import asdict, dataclass
import json
from typing import Any, Dict


@dataclass
class PeerState:
    robot_id: str
    timestamp: float
    x: float
    y: float
    heading: float
    linear_velocity: float
    current_goal: str
    intended_zone: str
    task_priority: int
    waiting_time: float
    localization_mode: str
    localization_confidence: float
    coordination_state: str

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> "PeerState":
        data: Dict[str, Any] = json.loads(payload)
        return cls(
            robot_id=str(data["robot_id"]),
            timestamp=float(data["timestamp"]),
            x=float(data["x"]),
            y=float(data["y"]),
            heading=float(data["heading"]),
            linear_velocity=float(data["linear_velocity"]),
            current_goal=str(data.get("current_goal", "")),
            intended_zone=str(data.get("intended_zone", "")),
            task_priority=int(data.get("task_priority", 0)),
            waiting_time=float(data.get("waiting_time", 0.0)),
            localization_mode=str(data.get("localization_mode", "GNSS_FIX")),
            localization_confidence=float(data.get("localization_confidence", 1.0)),
            coordination_state=str(data.get("coordination_state", "GO")),
        )
