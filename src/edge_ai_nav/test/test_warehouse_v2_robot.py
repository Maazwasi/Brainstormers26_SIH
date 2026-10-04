"""The V2 navigation footprint must contain every physical robot collision."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


SIM = Path(__file__).resolve().parents[2] / 'amr_simulation'
MODEL = ET.parse(SIM / 'models/warehouse_amr_v2.sdf.xacro').getroot()
CFG = yaml.safe_load((SIM / 'config/warehouse_sih_v2.yaml').read_text())['warehouse']
NAV2 = yaml.safe_load((SIM / 'config/nav2_params_v2.yaml').read_text())


def test_v2_collision_bounds_and_navigation_footprint():
    bounds = [math.inf, -math.inf, math.inf, -math.inf]
    for link in MODEL.findall('.//model/link'):
        pose = [float(value) for value in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        for collision in link.findall('collision'):
            local = [float(value) for value in
                     (collision.findtext('pose') or '0 0 0 0 0 0').split()]
            box = collision.find('geometry/box/size')
            if box is not None:
                sx, sy, _ = (float(value) for value in box.text.split())
                half_x, half_y = sx / 2, sy / 2
            else:
                wheel = collision.find('geometry/cylinder')
                half_x = float(wheel.findtext('radius'))
                half_y = float(wheel.findtext('length')) / 2
                assert abs(pose[3] + local[3]) == pytest.approx(math.pi/2)
            x, y = pose[0] + local[0], pose[1] + local[1]
            bounds = [min(bounds[0], x-half_x), max(bounds[1], x+half_x),
                      min(bounds[2], y-half_y), max(bounds[3], y+half_y)]
    assert bounds == pytest.approx([-0.45, 0.45, -0.38, 0.38])
    assert CFG['amr_collision_bounds_m'] == {'x': [-0.45, 0.45],
                                             'y': [-0.38, 0.38]}
    polygon = CFG['amr_footprint_polygon_m']
    assert [min(p[0] for p in polygon), max(p[0] for p in polygon),
            min(p[1] for p in polygon), max(p[1] for p in polygon)] == pytest.approx(bounds)
    assert CFG['amr_footprint_diameter_m'] / 2 >= math.hypot(0.45, 0.38)
    assert CFG['route_graph']['clearance'] >= CFG['amr_footprint_diameter_m'] / 2
    for layer in ('local_costmap', 'global_costmap'):
        actual = yaml.safe_load(NAV2[layer][layer]['ros__parameters']['footprint'])
        assert actual == polygon


def test_v2_visual_body_and_drive_geometry():
    body = MODEL.find('.//model/link[@name="base_link"]')
    collision = body.findtext('collision/geometry/box/size')
    visual = body.findtext('visual[@name="chassis_visual"]/geometry/box/size')
    assert collision == visual == '0.90 0.70 0.20'
    plugin = MODEL.find('.//model/plugin[@name="gz::sim::systems::DiffDrive"]')
    assert float(plugin.findtext('wheel_separation')) == pytest.approx(0.70)
    assert float(plugin.findtext('wheel_radius')) == pytest.approx(0.11)
    assert plugin.findtext('max_linear_velocity') == '$(arg max_linear_velocity)'
    for speed in (0.40,0.50,0.60):
        expanded=subprocess.run(['xacro',str(SIM/'models/warehouse_amr_v2.sdf.xacro'),
                                 'namespace:=alpha_1',f'max_linear_velocity:={speed}'],
                                capture_output=True,text=True,check=True)
        drive=ET.fromstring(expanded.stdout).find(
            './/model/plugin[@name="gz::sim::systems::DiffDrive"]')
        assert float(drive.findtext('max_linear_velocity')) == pytest.approx(speed)
        assert float(drive.findtext('min_linear_velocity')) == pytest.approx(-speed)
