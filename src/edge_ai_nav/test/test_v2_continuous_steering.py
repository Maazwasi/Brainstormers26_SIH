"""V2 angle-as-needed steering and forward-only adaptive lookahead."""
import math
from types import SimpleNamespace

import pytest

from edge_ai_nav.ros_nodes.local_waypoint_controller import (
    LocalController, heading_rate, passed_waypoint, wrap)


@pytest.mark.parametrize('degrees', [15, 30, 45, 60, 90, 135, -30, -90])
def test_heading_control_converges_to_actual_requested_angle(degrees):
    target = math.radians(degrees)
    yaw = 0.0
    for _ in range(240):
        error = wrap(target-yaw)
        yaw += heading_rate(error, 1.25, 0.75)*0.05
    assert wrap(target-yaw) == pytest.approx(0.0, abs=0.051)
    assert yaw == pytest.approx(target, abs=0.051)


@pytest.mark.parametrize('degrees', [15, 30, 45, 60, 90])
def test_v2_speed_slows_continuously_for_curvature(degrees):
    follower = SimpleNamespace(
        coordination_command={}, mission_kind='TASK',
        performance=SimpleNamespace(speed_factor=1.0),
        p={'smooth_steering': True, 'max_linear_speed': 0.40})
    speed = LocalController.desired_speed(follower, math.radians(degrees))
    assert 0 < speed <= 0.40
    if degrees >= 45:
        assert speed < LocalController.desired_speed(follower, math.radians(30))


def test_v2_successive_and_s_bend_lookahead_never_reverses():
    route = [(0, 0), (3, 0), (5, 1), (7, 3), (9, 3),
             (11, 1), (13, 1), (15, 3)]
    cumulative = [0.0]
    for a, b in zip(route, route[1:]):
        cumulative.append(cumulative[-1]+math.dist(a,b))
    follower = SimpleNamespace(
        route=route, route_cumulative=cumulative, pose=(0.5, 0, 0),
        index=1, lookahead_progress=0.0, last_linear_command=0.35,
        p={'lookahead_distance':0.75, 'max_linear_speed':0.40,
           'smooth_steering':True},
        performance=SimpleNamespace(lookahead_factor=1.0))
    observed=[]
    for index in range(1,len(route)):
        follower.index=index
        follower.pose=(*route[index-1],0)
        LocalController.lookahead_target(follower)
        observed.append(follower.lookahead_progress)
    assert observed == sorted(observed)
    assert all(observed[i] >= cumulative[i] for i in range(len(observed)))


def test_v2_overlapping_return_leg_does_not_capture_future_segment():
    route=[(0,0),(3,0),(3,1),(0,1),(0,0)]
    cumulative=[0.0]
    for a,b in zip(route,route[1:]):
        cumulative.append(cumulative[-1]+math.dist(a,b))
    follower=SimpleNamespace(route=route,route_cumulative=cumulative,
        pose=(1,0,0),index=1,lookahead_progress=0.0,last_linear_command=0.25,
        p={'lookahead_distance':0.75,'max_linear_speed':0.40,'smooth_steering':True},
        performance=SimpleNamespace(lookahead_factor=1.0))
    target=LocalController.lookahead_target(follower)
    assert target[1] == pytest.approx(0.0)
    assert 1 < target[0] < 3


def test_v2_waypoint_handoff_accepts_bounded_lateral_miss_before_corner():
    previous=(11.75,32.5)
    corner=(11.75,23.5)
    assert passed_waypoint((11.45,24.4),previous,corner,0.45)
    assert not passed_waypoint((11.0,24.4),previous,corner,0.45)
    assert not passed_waypoint((11.45,27.0),previous,corner,0.45)


def test_charging_entry_lookahead_stops_at_occupied_bay_safe_corner():
    route=[(25.,23.5),(25.,32.5),(28.,32.5),(28.,36.)]
    follower=SimpleNamespace(
        route=route,route_cumulative=[0.,9.,12.,15.5],
        pose=(25.,32.2,math.pi/2),index=1,lookahead_progress=8.8,
        last_linear_command=0.28,destination='CHARGE_04',
        graph=SimpleNamespace(forward_rejoin=True,charging=('CHARGE_04',)),
        p={'lookahead_distance':0.75,'max_linear_speed':0.40,
           'smooth_steering':True},
        performance=SimpleNamespace(lookahead_factor=1.0))
    assert LocalController.charging_bend(follower,1)
    assert LocalController.charging_bend(follower,2)
    assert LocalController.lookahead_target(follower)==pytest.approx((25.,32.5))
    assert follower.lookahead_progress==pytest.approx(9.)


def test_charging_departure_corner_is_protected_for_delivery_mission():
    follower=SimpleNamespace(route=[(19.,36.),(19.,32.5),(25.,32.5),(25.,23.5)],
        destination='DROP_07',graph=SimpleNamespace(forward_rejoin=True,
            charging=('CHARGE_01',),nodes={'CHARGE_01_APPROACH':(19.,32.5)}))
    assert LocalController.charging_bend(follower,1)
    assert not LocalController.charging_bend(follower,2)
