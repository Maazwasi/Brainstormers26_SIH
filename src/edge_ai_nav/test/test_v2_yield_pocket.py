"""Collision-checked lateral waiting positions preserve the original mission."""
from pathlib import Path
from types import SimpleNamespace

import yaml

from edge_ai_nav.fleet.path_safety import yield_pocket
from edge_ai_nav.fleet.route_graph import WarehouseGraph
from edge_ai_nav.fleet.peer_state_node import PeerState


CONFIG=Path(__file__).resolve().parents[2]/'amr_simulation/config/warehouse_sih_v2.yaml'
GRAPH=WarehouseGraph(yaml.safe_load(CONFIG.read_text())['warehouse'])


def states():
    own=dict(robot_id='ALPHA 3',x=38.25,y=16.5,remaining_route=[[38.25,8.],[45.,4.]],
             local_nav_state='COORDINATION_HOLD',task_priority=3)
    peer=dict(robot_id='ALPHA 5',x=38.25,y=18.4,remaining_route=[[38.25,8.],[38.5,4.]],
              task_priority=3)
    return own,peer


def test_side_aisle_pocket_clears_peer_swept_path_and_preserves_mission():
    own,peer=states()
    original=[list(p) for p in own['remaining_route']]
    target=yield_pocket(GRAPH,own,peer,[peer])
    assert target is not None
    assert GRAPH.visible((own['x'],own['y']),target)
    assert abs(target[0]-38.25)>=2.35
    assert GRAPH.point_segment_distance((peer['x'],peer['y']),
                                       (own['x'],own['y']),target)>=1.55
    assert own['remaining_route']==original


def test_no_escape_when_nearby_robots_occupy_both_sides():
    own,peer=states()
    peers=[peer]+[dict(robot_id=f'BLOCK-{i}',x=x,y=y) for i,(x,y) in enumerate(
        [(36.75,16.5),(39.75,16.5),(38.25,15.)])]
    assert yield_pocket(GRAPH,own,peer,peers) is None


def test_shared_yield_starts_clear_pocket_before_emergency_proximity():
    own=dict(robot_id='ALPHA 2',x=30.,y=23.5,
             remaining_route=[[25.,23.5],[20.,23.5]],local_nav_state='WAYPOINT_TRACK')
    peer=dict(robot_id='ALPHA 1',x=20.,y=23.5,
              remaining_route=[[25.,23.5],[30.,23.5]])
    node=SimpleNamespace(robot_id='ALPHA 2',peer_states={'ALPHA 1':peer},
        physical_guard_peer='',yield_relocation=None,graph=GRAPH,
        status={'state':'WAYPOINT_TRACK'},book=SimpleNamespace(values=lambda:[
            dict(winner='ALPHA 1',loser='ALPHA 2')]),emit=lambda *args,**kwargs:None)
    command=PeerState.relocation_command(node,own,set(),
        dict(state='YIELD_AND_WAIT',winner='ALPHA 1',loser='ALPHA 2'))
    assert command['state']=='YIELD_RELOCATE'
    assert command['winner']=='ALPHA 1'
    assert node.yield_relocation['origin']==[30.,23.5]
    # Include the endpoint beyond the old 8 m horizon while the loser waits.
    assert GRAPH.point_segment_distance(command['target'],(20.,23.5),(30.,23.5))>=3.0


def test_shared_loser_relocates_then_waits_for_clear_before_resuming():
    own,peer=states()
    decisions=[dict(winner='ALPHA 5',loser='ALPHA 3')]
    node=SimpleNamespace(robot_id='ALPHA 3',peer_states={'ALPHA 5':peer},
        physical_guard_peer='ALPHA 5',yield_relocation=None,graph=GRAPH,
        status={'state':'COORDINATION_HOLD'},book=SimpleNamespace(values=lambda:decisions),
        emit=lambda *args,**kwargs:None)
    first=PeerState.relocation_command(node,own,set(),
        {'state':'SAFE_WAIT','reason':'PEER_FOOTPRINT_BLOCKED'})
    assert first['state']=='YIELD_RELOCATE'
    target=first['target']
    own.update(x=target[0],y=target[1])
    node.status={'state':'YIELD_POCKET'}
    peer['y']=10.
    assert PeerState.relocation_command(node,own,set(),{'state':'NONE'})['state']=='YIELD_RELOCATE'
    decisions.clear()
    peer['y']=4.  # Winner has physically cleared the return leg to y=8.
    peer['remaining_route']=[[38.5,4.]]
    # Direct pocket-to-next-waypoint crosses a shelf corner. Return over the
    # checked lateral escape first, while preserving the mission route.
    assert PeerState.relocation_command(node,own,set(),{'state':'RESUME'})['state']=='YIELD_RELOCATE'
    origin=node.yield_relocation['origin']
    assert node.yield_relocation['target']==origin
    own.update(x=origin[0],y=origin[1])
    assert PeerState.relocation_command(node,own,set(),{'state':'RESUME'})['state']=='RESUME'
    assert node.yield_relocation is None


def test_cleared_book_does_not_release_pocket_into_winners_future_path():
    own=dict(robot_id='ALPHA 1',x=22.,y=26.5,
             remaining_route=[[25.,23.5],[30.,23.5]])
    peer=dict(robot_id='ALPHA 2',x=28.,y=23.5,
              local_nav_state='WAYPOINT_TRACK',remaining_route=[[25.,23.5],[20.,23.5]])
    node=SimpleNamespace(robot_id='ALPHA 1',peer_states={'ALPHA 2':peer},
        yield_relocation=dict(peer='ALPHA 2',target=[22.,26.5],id='POCKET:A:B'),
        graph=GRAPH,status={'state':'YIELD_POCKET'},
        book=SimpleNamespace(values=lambda:[]),emit=lambda *args,**kwargs:None)
    assert PeerState.relocation_command(node,own,set(),{'state':'RESUME'})['state']=='YIELD_RELOCATE'
    assert node.yield_relocation is not None
    peer.update(x=20.,local_nav_state='DEMO_STAGED')
    assert PeerState.relocation_command(node,own,set(),{'state':'RESUME'})['state']=='RESUME'


def test_cleared_pocket_resumes_when_winner_is_near_but_off_remaining_route():
    own=dict(robot_id='ALPHA 2',x=27.82,y=26.45,
             remaining_route=[[25.,23.5],[20.,23.5]])
    peer=dict(robot_id='ALPHA 1',x=29.57,y=23.47)
    node=SimpleNamespace(robot_id='ALPHA 2',peer_states={'ALPHA 1':peer},
        yield_relocation=dict(peer='ALPHA 1',target=[27.82,26.45],id='POCKET:A:B'),
        graph=GRAPH,status={'state':'YIELD_POCKET'},
        book=SimpleNamespace(values=lambda:[]),emit=lambda *args,**kwargs:None)
    assert 2.2<((own['x']-peer['x'])**2+(own['y']-peer['y'])**2)**.5<4.
    result=PeerState.relocation_command(node,own,set(),{'state':'RESUME'})
    assert result['state']=='RESUME'
    assert result['resume_route']==[[27.82,26.45],[25.,23.5],[20.,23.5]]
    assert GRAPH.point_segment_distance((29.57,23.47),*result['resume_route'][:2])>=2.2
    assert node.yield_relocation is None
