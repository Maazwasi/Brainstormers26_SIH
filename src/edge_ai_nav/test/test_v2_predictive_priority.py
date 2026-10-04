"""V2 predictive cross-aisle zones and priority-first local arbitration."""
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from edge_ai_nav.fleet.negotiation import resolve_conflict, physically_committed
from edge_ai_nav.fleet.route_graph import WarehouseGraph
from edge_ai_nav.fleet.route_graph import segment_interval
from edge_ai_nav.fleet.zone_prediction import LocalDetector, load_zones


CFG = yaml.safe_load((Path(__file__).resolve().parents[2] /
                      'amr_simulation/config/warehouse_sih_v2.yaml').read_text())['warehouse']
GRAPH = WarehouseGraph(CFG)
ZONES = load_zones(CFG)


def active(name, priority, **extras):
    return dict(robot_id=name, task_priority=priority,
                local_nav_state='WAYPOINT_TRACK', zone_committed=False,
                zone_reserved=False, safety_emergency=False,
                waiting_time=0, **extras)


def test_physical_main_cross_aisles_and_side_corridors_are_coordination_zones():
    assert len(ZONES) == 8
    assert {'MAIN_NORTH_CROSS', 'MAIN_UPPER_CROSS',
            'MAIN_MID_CROSS', 'MAIN_LOWER_CROSS'} <= set(ZONES)
    for name, zone in ZONES.items():
        if not name.startswith('MAIN_'):
            assert zone['bounds']['y'] == [4.0,32.5]
            continue
        assert zone['center'] == GRAPH.nodes[name.replace('_CROSS', '')]
        assert zone['radius'] > CFG['amr_footprint_diameter_m']/2


def test_predicts_future_temporal_overlap_and_side_aisle_occupancy():
    left = GRAPH.intent((20, 23.5), [[25, 23.5], [30, 23.5]])
    right = GRAPH.intent((30, 23.5), [[25, 23.5], [20, 23.5]])
    assert left['zone'] == right['zone'] == 'MAIN_UPPER_CROSS'
    own = active('ALPHA 1', 4, x=20, y=23.5, yaw=0, linear_velocity=0.4,
                 goal_x=30, goal_y=23.5, next_zone=left['zone'], zone_intent=left)
    peer = active('ALPHA 2', 2, x=30, y=23.5, yaw=3.14159,
                  linear_velocity=0.4, goal_x=20, goal_y=23.5,
                  next_zone=right['zone'], zone_intent=right)
    detector = LocalDetector(ZONES, horizon=30.0)
    events = detector.update(own, [peer], now=1.0)
    assert events and events[0][0] == 'CONFLICT_PREDICTED'
    assert events[0][1]['state'] == 'PREDICTED'
    side = GRAPH.intent((11.75, 25), [[11.75, 16.5]])
    assert side['zone'] == 'LEFT_INNER_CORRIDOR'
    assert side['inside']


@pytest.mark.parametrize('higher,lower', [(4, 3), (3, 2), (2, 1)])
def test_mission_priority_beats_all_normal_secondary_metrics(higher, lower):
    high = active('ALPHA 1', higher, charging_urgency=0,
                  mission_progress=0, clearance_time_s=100)
    low = active('ALPHA 2', lower, charging_urgency=2,
                 mission_progress=99, clearance_time_s=1)
    decision = resolve_conflict(high, low, 'MAIN_MID_CROSS',
                                priority_first=True)
    assert (decision.winner, decision.reason) == ('ALPHA 1', 'TASK_PRIORITY')
    assert resolve_conflict(low, high, 'MAIN_MID_CROSS',
                            priority_first=True) == decision


def test_equal_priority_uses_charging_then_progress():
    high_charge = active('ALPHA 1', 3, charging_urgency=2,
                         mission_progress=0)
    no_charge = active('ALPHA 2', 3, charging_urgency=0,
                       mission_progress=99)
    result = resolve_conflict(high_charge, no_charge, 'MAIN_MID_CROSS',
                              priority_first=True)
    assert (result.winner, result.reason) == ('ALPHA 1', 'CHARGING_URGENCY')
    high_charge['charging_urgency'] = 0
    result = resolve_conflict(high_charge, no_charge, 'MAIN_MID_CROSS',
                              priority_first=True)
    assert (result.winner, result.reason) == ('ALPHA 2', 'MISSION_PROGRESS')


@pytest.mark.parametrize('winner_field,winner_value,loser_value,reason', [
    ('charging_urgency',2,0,'CHARGING_URGENCY'),
    ('deadline_remaining_s',10.,20.,'TASK_DEADLINE'),
    ('loaded',True,False,'LOADED'),
    ('mission_progress',.8,.2,'MISSION_PROGRESS'),
    ('waiting_time',12.,1.,'WAITING_TIME'),
    ('clearance_time_s',3.,9.,'CLEARANCE_TIME'),
])
def test_equal_priority_tie_breaks_in_required_order(winner_field,winner_value,
                                                      loser_value,reason):
    baseline=dict(charging_urgency=0,deadline_remaining_s=30.,loaded=False,
                  mission_progress=0.,waiting_time=0.,clearance_time_s=30.)
    first=active('ALPHA 1',3)
    second=active('ALPHA 2',3)
    first.update(baseline)
    second.update(baseline)
    first[winner_field]=winner_value
    second[winner_field]=loser_value
    # Stale reservation and earlier ETA do not precede normal V2 tie-breaks.
    second.update(zone_reserved=True,eta=.1)
    decision=resolve_conflict(first,second,'MAIN_MID_CROSS',priority_first=True)
    assert (decision.winner,decision.reason)==('ALPHA 1',reason)
    assert resolve_conflict(second,first,'MAIN_MID_CROSS',priority_first=True)==decision


def test_equal_everything_uses_stable_robot_id():
    first=active('ALPHA 1',3)
    second=active('ALPHA 2',3)
    second.update(zone_reserved=True,eta=.1)
    decision=resolve_conflict(second,first,'MAIN_MID_CROSS',priority_first=True)
    assert (decision.winner,decision.reason)==('ALPHA 1','ROBOT_ID')


def test_real_reroute_avoids_predicted_zone_after_smoothing():
    current=(28.18,34.38)
    remaining=[[25,32.5],[25,23.5],[16,25.1]]
    route, extra=GRAPH.reroute(current,remaining,'PICKUP_02','MAIN_NORTH_CROSS')
    assert route[0] == list(current)
    assert route[-1] == CFG['pickups']['PICKUP_02']
    assert all(GRAPH.visible(a,b) for a,b in zip(route,route[1:]))
    assert not any(segment_interval(a,b,ZONES['MAIN_NORTH_CROSS'])
                   for a,b in zip(route,route[1:]))
    assert extra >= 0


def test_physical_commitment_overrides_priority_only_past_stop_boundary():
    zone=ZONES['MAIN_UPPER_CROSS']
    policy={'safe_deceleration':0.65, 'processing_margin':0.15,
            'footprint_margin':0.45, 'safety_margin':0.55,
            'minimum_distance':1.5, 'maximum_distance':3.0}
    low=active('ALPHA 5',2, x=25.45,y=25.1,linear_velocity=0.4,
               zone_intent={'distance':0.1})
    high=active('ALPHA 1',4, x=25.2,y=27.7,linear_velocity=0.4,
                zone_intent={'distance':2.6})
    low['zone_committed']=physically_committed(low,zone,policy)
    high['zone_committed']=physically_committed(high,zone,policy)
    assert low['zone_committed'] and not high['zone_committed']
    decision=resolve_conflict(high,low,'MAIN_UPPER_CROSS',priority_first=True)
    assert (decision.winner,decision.reason)==('ALPHA 5','ZONE_COMMITTED')
    far_low=dict(low,x=25.45,y=30.0,zone_intent={'distance':4.9},
                 zone_committed=False)
    normal=resolve_conflict(high,far_low,'MAIN_UPPER_CROSS',priority_first=True)
    assert (normal.winner,normal.reason)==('ALPHA 1','TASK_PRIORITY')


def test_v2_can_disable_reroute_without_changing_priority():
    high=active('ALPHA 1',4)
    low=active('ALPHA 5',2)
    costs={'ALPHA 5': {'wait':20.0, 'reroute':1.0,
                       'route':[[27.0,33.0],[25.0,23.5]]}}
    legacy=resolve_conflict(high,low,'MAIN_NORTH_CROSS',route_costs=costs,
                            priority_first=True)
    safe=resolve_conflict(high,low,'MAIN_NORTH_CROSS',route_costs=costs,
                          priority_first=True,allow_reroute=False)
    assert legacy.loser_action == 'REROUTE'
    assert (safe.winner,safe.loser_action)==('ALPHA 1','YIELD_AND_WAIT')
    assert CFG['stage6']['allow_reroute'] is True


def test_forward_rejoin_preserves_destination_but_rejects_unsafe_peer_merge():
    start=(28.18,34.38)
    original=[[25,32.5],[25,23.5],[16,25.1]]
    alternate,extra=GRAPH.reroute_forward(
        start,original,'PICKUP_02','MAIN_NORTH_CROSS')
    assert alternate[0] == list(start)
    assert alternate[-1] == CFG['pickups']['PICKUP_02']
    assert [25,32.5] not in alternate
    assert [25,23.5] in alternate
    assert extra >= 0
    assert not any(segment_interval(a,b,ZONES['MAIN_NORTH_CROSS'])
                   for a,b in zip(alternate,alternate[1:]))
    winner=[[25.0,32.7],[25.0,23.5],[34.0,25.1]]
    assert GRAPH.route_clearance(alternate,winner) == 0.0
    assert GRAPH.route_clearance([[4,5],[4,15]],[[9,5],[9,15]]) == 5.0
    assert GRAPH.forward_rejoin


def test_forward_outer_aisle_bypass_is_safe_and_cheaper_than_long_corridor_wait():
    start=(41.,25.1)
    remaining=[[41.,23.5],[38.25,23.5],[38.25,8.],[45.,4.]]
    alternate,extra=GRAPH.reroute_forward(start,remaining,'DROP_07','RIGHT_INNER_CORRIDOR')
    winner_route=[[38.25,29.],[38.25,8.]]
    assert GRAPH.route_clearance(alternate,winner_route)>=2.35
    assert alternate[-1]==CFG['drops']['DROP_07']
    assert all(b[1]<=a[1] for a,b in zip(alternate,alternate[1:]))
    assert extra<2.
    winner=active('ALPHA 5',4)
    loser=active('ALPHA 3',2)
    decision=resolve_conflict(winner,loser,'RIGHT_INNER_CORRIDOR',priority_first=True,
        route_costs={'ALPHA 3':{'wait':60.,'reroute':extra/.4,'route':alternate}})
    assert decision.winner=='ALPHA 5'
    assert decision.loser_action=='REROUTE'
