"""Phase 2: the approved physical warehouse has a connected, redundant graph."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from edge_ai_nav.fleet.route_graph import WarehouseGraph


SIM = Path(__file__).resolve().parents[2] / 'amr_simulation'
CFG = yaml.safe_load((SIM / 'config/warehouse_sih_v2.yaml').read_text())['warehouse']
GRAPH = WarehouseGraph(CFG)
WORLD = ET.parse(SIM / 'worlds/warehouse_sih_v2.sdf').getroot()


def test_graph_obstacles_match_physical_world():
    solids = []
    for model in WORLD.findall('world/model'):
        name = model.attrib['name']
        if not name.startswith(('SHELF_', 'PALLET_')) and name != 'CHARGING_BACKSTOP':
            continue
        x, y = (float(value) for value in model.findtext('pose').split()[:2])
        sx, sy = (float(value) for value in model.findtext('link/collision/geometry/box/size').split()[:2])
        solids.append([round(x-sx/2, 3), round(x+sx/2, 3),
                       round(y-sy/2, 3), round(y+sy/2, 3)])
    assert len(solids) == 17
    assert sorted(solids) == sorted(GRAPH.obstacles)
    assert GRAPH.bounds == {'x': [0.125, 49.875], 'y': [0.125, 39.875]}
    for a, neighbors in GRAPH.edges.items():
        for b, _ in neighbors:
            assert GRAPH.visible(GRAPH.nodes[a], GRAPH.nodes[b]), (a, b)


@pytest.mark.parametrize('start,end', [
    ('PICKUP_01', 'DROP_07'),
    ('PICKUP_12', 'DROP_01'),
    ('CHARGE_03', 'DROP_04'),
    ('DROP_04', 'CHARGE_03'),
    ('PICKUP_05', 'PICKUP_08'),
])
def test_primary_and_blocked_main_routes(start, end):
    primary_cost, primary = GRAPH.astar(start, end)
    assert primary[0] == start and primary[-1] == end
    assert primary_cost > 0
    blocked = [('MAIN_UPPER', 'MAIN_MID'), ('MAIN_MID', 'MAIN_LOWER')]
    alternate_cost, alternate = GRAPH.astar(start, end, blocked_edges=blocked)
    assert alternate[0] == start and alternate[-1] == end
    assert alternate_cost >= math.dist(GRAPH.nodes[start], GRAPH.nodes[end])
    assert all(frozenset((a, b)) not in {frozenset(edge) for edge in blocked}
               for a, b in zip(alternate, alternate[1:]))


def test_all_locations_connected_with_main_spine_blocked():
    targets = [*CFG['pickups'], *CFG['drops'], *CFG['charging_bays']]
    for target in targets:
        _, route = GRAPH.astar('MAIN_NORTH', target,
                               blocked_edges=[('MAIN_UPPER', 'MAIN_MID')])
        assert route[-1] == target
    assert set(GRAPH.nodes) == set(GRAPH.edges)
    assert len(GRAPH.nodes) == 54


def test_normal_route_prefers_central_main_path():
    _, route = GRAPH.astar('CHARGE_03', 'DROP_04')
    assert route == ['CHARGE_03', 'CHARGE_03_APPROACH', 'MAIN_NORTH', 'MAIN_UPPER', 'MAIN_MID',
                     'MAIN_LOWER', 'MAIN_SOUTH', 'DROP_04']
    _, bypass = GRAPH.astar('CHARGE_03', 'DROP_04',
                            blocked_edges=[('MAIN_UPPER', 'MAIN_MID')])
    assert any('INNER_' in node or 'OUTER_' in node for node in bypass)


def test_each_charger_has_its_own_south_approach_clear_of_occupied_neighbours():
    for slot in CFG['charging_bays']:
        _,route=GRAPH.route_to_with_cost((25.,16.5),slot)
        assert route[-2]==GRAPH.nodes[f'{slot}_APPROACH']
        assert route[-1]==GRAPH.nodes[slot]
        assert all(GRAPH.visible(a,b) for a,b in zip(route,route[1:]))
        for occupied in CFG['charging_bays']:
            if occupied==slot:
                continue
            clearance=min(GRAPH.point_segment_distance(
                GRAPH.nodes[occupied],a,b) for a,b in zip(route,route[1:]))
            assert clearance>=2.0,(slot,occupied,route,clearance)


def test_delivery_routes_do_not_traverse_other_drop_pads():
    # A robot may remain at a delivery pad while another task is still active.
    # The y=4 south row therefore cannot be used as a through-traffic edge.
    drops=[name for name in GRAPH.nodes if name.startswith('DROP_')]
    pickups=[name for name in GRAPH.nodes if name.startswith('PICKUP_')]
    for pickup in pickups:
        for destination in drops:
            _,route=GRAPH.astar(pickup,destination)
            for occupied in drops:
                if occupied==destination:
                    continue
                clearance=min(GRAPH.point_segment_distance(
                    GRAPH.nodes[occupied],GRAPH.nodes[a],GRAPH.nodes[b])
                    for a,b in zip(route,route[1:]))
                assert clearance>=CFG['amr_footprint_diameter_m'], (
                    pickup,destination,occupied,route,clearance)


@pytest.mark.parametrize('slot', list(CFG['charging_bays']))
def test_docked_departure_uses_own_straight_entry(slot):
    position=GRAPH.nodes[slot]
    entry=GRAPH.nodes[f'{slot}_APPROACH']
    plan=GRAPH.mission_plan(position,'PICKUP_03','DROP_07')
    for key in ('raw_waypoints','smoothed_waypoints'):
        assert plan[key][:2]==[position,entry]
    _,route=GRAPH.route_to_with_cost(position,'DROP_04')
    assert route[:2]==[position,entry]
    for other in CFG['charging_bays']:
        if other != slot:
            assert GRAPH.point_segment_distance(GRAPH.nodes[other],position,entry)>=3.0
    assert GRAPH.charging_departure(entry) is None


def test_v2_fleet_ids_and_spawn_bays():
    robots = [f'alpha_{index}' for index in range(1, 6)]
    assert list(CFG['robot_spawns']) == robots
    assert list(CFG['robot_visuals']) == robots
    assert list(CFG['stage3_missions']) == robots
    for index, rid in enumerate(robots, 1):
        label = f'ALPHA {index}'
        assert CFG['robot_visuals'][rid]['label'] == label
        assert CFG['stage4_peer_metadata'][label]['namespace'] == f'/{rid}'
        assert CFG['robot_spawns'][rid][:2] == CFG['charging_bays'][f'CHARGE_{index:02d}']
