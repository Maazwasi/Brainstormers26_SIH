"""Repeatable golden-demo routes and identities."""

ROBOTS = {
    "amr_alpha": {
        "label": "ALPHA", "color": (1.0, 0.05, 0.05), "spawn": (-2.0, -2.0, 0.0),
        "priority": 1, "delay": 0.0,
        "waypoints": [(-0.15, -2.0, ""), (0.0, 0.0, "intersection_A"), (2.0, 0.0, "")],
    },
    "amr_bravo": {
        "label": "BRAVO", "color": (0.05, 0.20, 1.0), "spawn": (0.0, 2.5, -1.57),
        "priority": 2, "delay": 2.0,
        "waypoints": [(0.0, 0.0, "intersection_A"), (0.0, 7.0, "intersection_B"), (1.5, 7.0, "")],
    },
    "amr_charlie": {
        "label": "CHARLIE", "color": (0.05, 0.95, 0.15), "spawn": (-5.0, 7.0, 0.0),
        "priority": 1, "delay": 8.0,
        "waypoints": [(0.0, 7.0, "intersection_B"), (2.0, 7.0, "")],
    },
    "amr_delta": {
        "label": "DELTA", "color": (1.0, 0.35, 0.02), "spawn": (-1.5, -7.0, 0.0),
        "priority": 1, "delay": 25.0,
        "waypoints": [(0.0, -7.0, "narrow_aisle_1"), (0.0, -5.5, "")],
    },
    "amr_echo": {
        "label": "ECHO", "color": (0.65, 0.05, 0.85), "spawn": (1.5, -7.0, 3.14159),
        "priority": 1, "delay": 25.0,
        "waypoints": [(0.0, -7.0, "narrow_aisle_1"), (-1.5, -7.0, "")],
    },
}
