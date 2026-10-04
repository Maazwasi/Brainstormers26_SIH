"""Keep the configured navigation footprint tied to Gazebo collision geometry."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2] / 'amr_simulation'


def test_navigation_footprint_covers_every_collision():
    model = ET.parse(ROOT / 'models/warehouse_amr.sdf.xacro').getroot()
    warehouse = yaml.safe_load((ROOT / 'config/warehouse_sih_demo.yaml').read_text())['warehouse']
    nav2 = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    bounds = [math.inf, -math.inf, math.inf, -math.inf]
    for link in model.findall('.//model/link'):
        pose = [float(n) for n in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        for collision in link.findall('collision'):
            local = [float(n) for n in (collision.findtext('pose') or '0 0 0 0 0 0').split()]
            box = collision.find('geometry/box/size')
            cylinder = collision.find('geometry/cylinder')
            if box is not None:
                sx, sy, _ = (float(n) for n in box.text.split())
                half_x, half_y = sx / 2, sy / 2
            else:
                radius = float(cylinder.findtext('radius'))
                length = float(cylinder.findtext('length'))
                # Wheels have a -90 degree roll: cylinder length points along Y.
                assert math.isclose(abs(pose[3] + local[3]), math.pi / 2, abs_tol=1e-6)
                half_x, half_y = radius, length / 2
            x, y = pose[0] + local[0], pose[1] + local[1]
            bounds = [min(bounds[0], x-half_x), max(bounds[1], x+half_x),
                      min(bounds[2], y-half_y), max(bounds[3], y+half_y)]

    assert bounds == pytest_approx([-0.30, 0.30, -0.255, 0.255])
    assert warehouse['amr_collision_bounds_m'] == {'x': [-0.30, 0.30], 'y': [-0.255, 0.255]}
    polygon = warehouse['amr_footprint_polygon_m']
    assert [min(p[0] for p in polygon), max(p[0] for p in polygon),
            min(p[1] for p in polygon), max(p[1] for p in polygon)] == pytest_approx(bounds)
    assert warehouse['amr_footprint_diameter_m'] / 2 == pytest_approx(
        math.hypot(0.30, 0.225))
    assert warehouse['route_graph']['clearance'] >= 0.375
    for kind in ('local_costmap', 'global_costmap'):
        configured = yaml.safe_load(nav2[kind][kind]['ros__parameters']['footprint'])
        assert configured == polygon


def pytest_approx(value):
    # Local import keeps this test usable without pytest at module import time.
    import pytest
    return pytest.approx(value)
