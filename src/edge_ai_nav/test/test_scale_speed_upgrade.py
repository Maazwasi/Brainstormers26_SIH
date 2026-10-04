"""Static preparation gates; none of these assert physical acceptance."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import pytest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edge_ai_nav.fleet.route_graph import WarehouseGraph
from edge_ai_nav.fleet.scale_speed import braking_distance, obstacle_distance, peer_envelope, forward_scan, travel_time, projected_peer_gap
from edge_ai_nav.fleet.zone_prediction import LocalDetector

SIM=Path(__file__).resolve().parents[2]/'amr_simulation'
CFG=yaml.safe_load((SIM/'config/warehouse_sih_v2_large.yaml').read_text())['warehouse']
GRAPH=WarehouseGraph(CFG)


def test_every_physical_length_and_inertial_scales():
    old=ET.parse(SIM/'models/warehouse_amr_v2.sdf.xacro').getroot()
    new=ET.parse(SIM/'models/warehouse_amr_v2_large.sdf.xacro').getroot()
    for tag,factor in [('size',2),('radius',2),('length',2),('mass',8),
                       ('ixx',32),('iyy',32),('izz',32),('wheel_radius',2),('wheel_separation',2)]:
        a=list(old.iter(tag));b=list(new.iter(tag));assert len(a)==len(b)
        for x,y in zip(a,b):
            assert list(map(float,y.text.split()))==pytest.approx([float(v)*factor for v in x.text.split()])
    for x,y in zip(old.iter('pose'),new.iter('pose')):
        a=list(map(float,x.text.split()));b=list(map(float,y.text.split()))
        assert b[:3]==pytest.approx([v*2 for v in a[:3]])
        assert b[3:]==a[3:]


def test_full_collision_and_separate_navigation_margin():
    assert CFG['amr_collision_bounds_m']=={'x':[-.90, .90],'y':[-.76,.76]}
    # Conservative enclosing radius is derived from full wheel-inclusive bounds.
    assert CFG['amr_footprint_diameter_m']/2>=math.hypot( .90,.76)
    assert GRAPH.clearance-CFG['amr_footprint_diameter_m']/2==pytest.approx(.15)
    assert CFG['scale_speed_upgrade']['target_speed_mps']==pytest.approx(.6*3)
    lidar_x=CFG['scale_speed_upgrade']['lidar_pose_m'][0]
    front_overhang=max(CFG['amr_collision_bounds_m']['x'])-lidar_x
    assert obstacle_distance(0,front_overhang=front_overhang)==pytest.approx(1.30)
    own={'x':0.,'y':0.,'yaw':0.,'linear_velocity':0.}
    peer=dict(own,x=10.)
    assert peer_envelope(own,peer,physical_diameter=CFG['amr_footprint_diameter_m'])[0]==pytest.approx(3.18)
    # Measure the model, not just repeated configuration constants. The wheel
    # cylinders are rotated about X, so their length contributes to Y bounds.
    model=ET.parse(SIM/'models/warehouse_amr_v2_large.sdf.xacro').getroot()
    bounds=[]
    for link in model.findall('model/link'):
        pose=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
        for collision in link.findall('collision'):
            offset=list(map(float,collision.findtext('pose','0 0 0 0 0 0').split()))
            box=collision.find('geometry/box/size')
            if box is not None:
                sx,sy,_=map(float,box.text.split());hx,hy=sx/2,sy/2
            else:
                assert abs(abs(pose[3])-math.pi/2)<1e-8
                hx=float(collision.findtext('geometry/cylinder/radius'))
                hy=float(collision.findtext('geometry/cylinder/length'))/2
            x,y=pose[0]+offset[0],pose[1]+offset[1]
            bounds.append((x-hx,x+hx,y-hy,y+hy))
    assert (min(b[0] for b in bounds),max(b[1] for b in bounds),
            min(b[2] for b in bounds),max(b[3] for b in bounds))==pytest.approx((-.90, .90,-.76,.76))


def test_world_collision_boxes_match_graph():
    world=ET.parse(SIM/'worlds/warehouse_sih_v2_large.sdf').getroot()
    solids=[]
    for model in world.findall('world/model'):
        if not model.attrib['name'].startswith(('SHELF_','PALLET_','CHARGING_BACKSTOP')):continue
        x,y=map(float,model.findtext('pose').split()[:2])
        sx,sy=map(float,model.findtext('link/collision/geometry/box/size').split()[:2])
        solids.append([round(x-sx/2,6),round(x+sx/2,6),round(y-sy/2,6),round(y+sy/2,6)])
    assert sorted(solids)==sorted(GRAPH.obstacles)
    for a,neighbours in GRAPH.edges.items():
        for b,_ in neighbours: assert GRAPH.visible(GRAPH.nodes[a],GRAPH.nodes[b]),(a,b)


def test_all_420_delivery_plans_and_charger_departures():
    for slot in CFG['charging_bays']:
        start=GRAPH.nodes[slot]
        for pickup in CFG['pickups']:
            for drop in CFG['drops']:
                plan=GRAPH.mission_plan(start,pickup,drop)
                for key in ('raw_waypoints','smoothed_waypoints'):
                    route=plan[key]
                    assert all(GRAPH.visible(a,b) for a,b in zip(route,route[1:]))
                    assert route[-1]==GRAPH.nodes[drop]
        for other in CFG['charging_bays']:
            if other!=slot:
                assert GRAPH.point_segment_distance(GRAPH.nodes[other],start,
                    GRAPH.nodes[slot+'_APPROACH'])>=CFG['stage6']['physical_stop_distance_m']
        for origin in ([25.,16.5],[2.225,8.],[47.775,23.5]):
            _,route=GRAPH.route_to_with_cost(origin,slot)
            for other in CFG['charging_bays']:
                if other!=slot:
                    assert min(GRAPH.point_segment_distance(GRAPH.nodes[other],a,b)
                        for a,b in zip(route,route[1:]))>=CFG['stage6']['physical_stop_distance_m']


@pytest.mark.parametrize('speed',CFG['scale_speed_upgrade']['speed_stages_mps'])
def test_braking_and_reaction_not_old_three_metre_clamp(speed):
    assert obstacle_distance(speed)==pytest.approx(.66+.64+.35*speed+speed*speed/1.3)
    own={'x':0.,'y':0.,'yaw':0.,'linear_velocity':speed}
    peer={'x':20.,'y':0.,'yaw':math.pi,'linear_velocity':speed}
    distance,closing=peer_envelope(own,peer)
    assert closing==pytest.approx(2*speed)
    assert distance==pytest.approx(3.18+.7*speed+speed*speed/.65)


def test_parallel_charger_departures_do_not_use_head_on_envelope():
    own={'x':20.8,'y':35.,'yaw':-math.pi/2,'linear_velocity':2.4}
    peer=dict(own,x=25.)
    distance,closing=peer_envelope(own,peer)
    assert distance==pytest.approx(3.18)
    assert closing==pytest.approx(0.,abs=1e-12)


def test_forward_scan_checks_strip_not_distant_side_shelves():
    scan=SimpleNamespace(angle_min=0.,angle_increment=.2,range_min=.05,range_max=20.,ranges=[float('inf'),10.,10.])
    assert forward_scan(scan)==20.
    scan.ranges[0]=3.
    assert forward_scan(scan)==3.


def test_invalid_scan_and_deceleration_fail_closed():
    scan=SimpleNamespace(angle_min=0.,angle_increment=.1,range_min=.05,range_max=20.,ranges=[float('nan')])
    assert forward_scan(scan) is None
    with pytest.raises(ValueError): braking_distance(2.4,0)


def test_accelerating_eta_is_not_instant_cruise_or_old_fixed_speed():
    assert travel_time(0.,0.,2.4)==0.
    assert travel_time(2.,0.,2.4)==pytest.approx(math.sqrt(4/.65))
    assert travel_time(10.,0.,2.4)>10/2.4
    assert travel_time(10.,2.4,2.4)==pytest.approx(10/2.4)


def test_winner_sweep_clears_peer_waiting_outside_crossing():
    own={'x':23.,'y':23.5,'yaw':0.,'linear_velocity':2.4}
    peer={'x':25.,'y':28.,'yaw':-math.pi/2,'linear_velocity':0.}
    assert projected_peer_gap(own,peer)==pytest.approx(4.5)
    peer['y']=26.
    assert projected_peer_gap(own,peer)==pytest.approx(2.5)


def test_upcoming_zone_conflict_detected_before_nearest_zone_matches():
    north={'zone':'MAIN_NORTH_CROSS','eta':0.,'window':[0.,4.],'inside':True,'distance':0.}
    upper={'zone':'MAIN_UPPER_CROSS','eta':7.,'window':[7.,15.],'inside':False,'distance':7.}
    own={'robot_id':'ALPHA 2','next_zone':'MAIN_NORTH_CROSS','zone_intent':north,
         'zone_intents':{'MAIN_NORTH_CROSS':north,'MAIN_UPPER_CROSS':upper}}
    peer={'robot_id':'ALPHA 1','next_zone':'MAIN_UPPER_CROSS','zone_intent':upper,
          'zone_intents':{'MAIN_UPPER_CROSS':upper}}
    detector=LocalDetector(GRAPH.zones,horizon=12.)
    events=detector.update(own,[peer],1.)
    assert events[0][1]['conflict_id']=='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 2'
    # The existing baseline single-nearest-zone semantics stay unchanged.
    own.pop('zone_intents');peer.pop('zone_intents')
    assert not LocalDetector(GRAPH.zones,horizon=12.).update(own,[peer],1.)
