"""A valid infinite LaserScan return means clear range, not a None crash."""
import math
from types import SimpleNamespace
import pytest

from edge_ai_nav.ros_nodes.local_waypoint_controller import sectors
from edge_ai_nav.fleet.scale_speed import forward_scan


def scan(ranges):
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                           angle_increment=2*math.pi/360,
                           range_min=0.05, range_max=20.0)


def test_all_clear_infinite_returns_are_finite_range_max():
    result=sectors(scan([math.inf]*360))
    assert result == {'front':20.0,'left':20.0,'right':20.0}


def test_missing_nan_beams_are_unknown_not_false_clear():
    result=sectors(scan([math.nan]*360))
    assert result == {'front':None,'left':None,'right':None}


def test_real_obstacle_precedes_clear_beams():
    values=[math.inf]*360
    for index in (179,180,181): values[index]=0.8
    assert sectors(scan(values))['front'] == 0.8


def test_low_scan_detects_short_crate_missed_by_upper_scan():
    upper=scan([math.inf]*360)
    low=SimpleNamespace(ranges=[math.inf]*181, angle_min=-math.pi/2,
                        angle_increment=math.pi/180, range_min=.05, range_max=20.)
    for index in (89,90,91):
        low.ranges[index]=1.0
    assert forward_scan(upper,half_width=.76,lidar_x=.24)==20.0
    assert forward_scan(low,half_width=.76,lidar_x=1.04)==pytest.approx(math.cos(math.pi/180))
