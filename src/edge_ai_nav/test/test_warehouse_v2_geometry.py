"""Phase 1 physical-layout invariants; the route graph is a Phase 2 concern."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest
import yaml


SIM = Path(__file__).resolve().parents[2] / 'amr_simulation'
WAREHOUSE = yaml.safe_load((SIM / 'config/warehouse_sih_v2.yaml').read_text())['warehouse']
WORLD = ET.parse(SIM / 'worlds/warehouse_sih_v2.sdf').getroot()
MODELS = {m.attrib['name']: m for m in WORLD.findall('world/model')}
RADIUS = WAREHOUSE['amr_footprint_diameter_m'] / 2


def pose(model):
    return tuple(float(v) for v in model.findtext('pose').split()[:2])


def box(model):
    values = model.findtext('link/collision/geometry/box/size')
    return tuple(float(v) for v in values.split()) if values else None


def test_world_geometry_and_semantics():
    assert WAREHOUSE['bounds'] == {'x': [0.0, 50.0], 'y': [0.0, 40.0]}
    assert box(MODELS['warehouse_floor']) == (50.0, 40.0, 0.1)
    assert WAREHOUSE['wall_thickness_m'] == 0.25
    assert len(WAREHOUSE['shelves']) == len(WAREHOUSE['pickups']) == 12
    assert len(WAREHOUSE['drops']) == 7
    assert len(WAREHOUSE['charging_bays']) == 5
    assert len(WAREHOUSE['static_obstacles']) == 4
    for name, spec in WAREHOUSE['shelves'].items():
        assert pose(MODELS[name]) == tuple(spec['center'])
        assert box(MODELS[name]) == tuple(spec['size'])
    for kind in ('pickups', 'drops', 'charging_bays'):
        for name, point in WAREHOUSE[kind].items():
            assert pose(MODELS[name]) == tuple(point)


def test_targets_and_aisles_have_robot_clearance():
    solid = []
    for name, model in MODELS.items():
        size = box(model)
        if size and name != 'warehouse_floor':
            x, y = pose(model)
            solid.append((name, x-size[0]/2, x+size[0]/2,
                          y-size[1]/2, y+size[1]/2))

    def clear(point, ignored=()):
        x, y = point
        assert RADIUS <= x <= 50-RADIUS and RADIUS <= y <= 40-RADIUS
        for name, x0, x1, y0, y1 in solid:
            if name in ignored:
                continue
            distance = math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1))
            assert distance >= RADIUS, (point, name, distance)

    for point in (*WAREHOUSE['pickups'].values(), *WAREHOUSE['drops'].values(),
                  *WAREHOUSE['charging_bays'].values()):
        clear(point)
    for item in WAREHOUSE['aisle_centrelines'].values():
        for step in range(101):
            y = item['y'][0] + (item['y'][1]-item['y'][0])*step/100
            clear((item['x'], y))
    for y in WAREHOUSE['cross_aisle_centrelines']:
        for step in range(101):
            clear((2.5+45.0*step/100, y))
    assert 31.1-18.9 >= 12.0  # dominant centre gap
    assert 13.1-10.4 >= 2*RADIUS  # narrowest shelf-to-shelf inner aisle
    assert 3.0 >= 2*RADIUS  # charge bay centre spacing


def test_solid_geometry_has_matching_visuals_and_no_overlap():
    solids = []
    for name, model in MODELS.items():
        size = box(model)
        if not size or name == 'warehouse_floor':
            continue
        visual_size = model.findtext('link/visual/geometry/box/size')
        assert visual_size is not None, name
        assert tuple(float(v) for v in visual_size.split()) == size, name
        if name.startswith('wall_'):
            continue  # boundary wall corners intentionally meet
        x, y = pose(model)
        solids.append((name, x-size[0]/2, x+size[0]/2,
                       y-size[1]/2, y+size[1]/2))
    assert 'CHARGING_BACKSTOP' in {item[0] for item in solids}
    for index, a in enumerate(solids):
        for b in solids[index+1:]:
            assert not (min(a[2], b[2]) > max(a[1], b[1]) + 1e-6
                        and min(a[4], b[4]) > max(a[3], b[3]) + 1e-6), (a[0], b[0])
