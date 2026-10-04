"""Route cardinality and forward-progress regressions for the local follower."""
import math
from types import SimpleNamespace

import pytest

from edge_ai_nav.ros_nodes.local_waypoint_controller import LocalController


def follower(points, pose=(0.0, 0.0, 0.0), index=0):
    route = [tuple(point) for point in points]
    cumulative = [0.0]
    for a, b in zip(route, route[1:]):
        cumulative.append(cumulative[-1] + math.dist(a, b))
    return SimpleNamespace(route=route, route_cumulative=cumulative,
                           pose=pose, index=index, lookahead_progress=0.0,
                           p={'lookahead_distance': 0.55},
                           performance=SimpleNamespace(lookahead_factor=1.0))


def test_empty_route_has_no_target():
    assert LocalController.lookahead_target(follower([])) is None


def test_single_goal_far_uses_the_goal():
    assert LocalController.lookahead_target(follower([(2.0, 1.0)])) == (2.0, 1.0)


def test_single_goal_already_reached_is_safe():
    robot = follower([(2.0, 1.0)], pose=(2.0, 1.0, 0.0))
    assert LocalController.lookahead_target(robot) == (2.0, 1.0)
    assert math.dist(robot.pose[:2], robot.route[-1]) < 0.16


def test_two_waypoints_straight_progress():
    robot = follower([(0.0, 0.0), (2.0, 0.0)], pose=(0.4, 0.0, 0.0), index=1)
    assert LocalController.lookahead_target(robot) == pytest.approx((0.95, 0.0))


def test_multi_waypoint_turn_stays_on_forward_path():
    robot = follower([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
                     pose=(1.4, 0.0, 0.0), index=1)
    target = LocalController.lookahead_target(robot)
    assert target[0] >= 1.4
    assert target[1] == pytest.approx(0.0)


def test_final_segment_targets_final_goal():
    robot = follower([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
                     pose=(2.0, 1.8, math.pi / 2), index=2)
    assert LocalController.lookahead_target(robot) == pytest.approx((2.0, 2.0))


def test_overlapping_return_leg_does_not_capture_forward_lookahead():
    robot = follower([(0.0, 0.0), (3.0, 0.0), (3.0, 1.0),
                      (0.0, 1.0), (0.0, 0.0)], pose=(1.0, 0.0, 0.0), index=1)
    first = LocalController.lookahead_target(robot)
    robot.pose = (1.2, 0.0, 0.0)
    second = LocalController.lookahead_target(robot)
    assert first == pytest.approx((1.35, 0.0))
    assert second == pytest.approx((1.55, 0.0))
    assert robot.lookahead_progress == pytest.approx(1.55)
