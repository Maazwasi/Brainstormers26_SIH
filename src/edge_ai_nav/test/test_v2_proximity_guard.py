"""The V2 physical brake follows the shared winner when robots get close."""
from types import SimpleNamespace
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edge_ai_nav.fleet.peer_state_node import PeerState


class Book:
    def __init__(self, decisions=()):
        self.decisions=decisions

    def values(self):
        return self.decisions


def state(rid, x, priority=3, active='TASK-1'):
    return dict(robot_id=rid,x=x,y=0.,task_priority=priority,active_task=active)


def test_canonical_loser_brakes_before_footprints_converge():
    # The shared decision can choose ALPHA 4 despite the ID tie-break.
    node=SimpleNamespace(robot_id='ALPHA 2',peer_states={'ALPHA 4':state('ALPHA 4',3.5)},
                         book=Book([dict(winner='ALPHA 4',loser='ALPHA 2',
                                         conflict_id='MAIN_NORTH_CROSS:ALPHA 2:ALPHA 4')]))
    guard=PeerState.proximity_guard(node,state('ALPHA 2',0.),set())
    assert guard['state']=='SAFE_WAIT'
    assert guard['winner']=='ALPHA 4'
    assert guard['distance_m']==3.5


def test_guard_is_inactive_for_distant_or_idle_peer():
    node=SimpleNamespace(robot_id='ALPHA 2',peer_states={'ALPHA 4':state('ALPHA 4',4.)},book=Book())
    assert PeerState.proximity_guard(node,state('ALPHA 2',0.),set()) is None
    node.peer_states['ALPHA 4']=state('ALPHA 4',3.,active='NONE')
    assert PeerState.proximity_guard(node,state('ALPHA 2',0.),set()) is None


def physical_node(peer, decisions=()):
    return SimpleNamespace(robot_id='ALPHA 5',peer_states={'ALPHA 3':peer},
        cfg={'stage6':{'physical_stop_distance_m':2.0}},
        physical_guard_peer='',book=Book(decisions))


def test_physical_brake_overrides_winner_approaching_stopped_loser():
    decision=dict(winner='ALPHA 5',loser='ALPHA 3')
    node=physical_node(state('ALPHA 3',1.9),[decision])
    guard=PeerState.physical_guard(node,state('ALPHA 5',0.),set())
    assert guard['state']=='SAFE_WAIT'
    assert guard['reason']=='PEER_FOOTPRINT_BLOCKED'
    assert guard['winner']=='ALPHA 5'  # Braking does not transfer ownership.


def test_physical_brake_protects_idle_and_returning_robots():
    node=physical_node(state('ALPHA 3',1.9,active='NONE'))
    assert PeerState.physical_guard(node,state('ALPHA 5',0.,active='NONE'),set())


def test_physical_hold_releases_only_after_separation_margin():
    node=physical_node(state('ALPHA 3',1.9))
    own=state('ALPHA 5',0.)
    assert PeerState.physical_guard(node,own,set())
    node.peer_states['ALPHA 3']['x']=2.10
    assert PeerState.physical_guard(node,own,set())
    node.peer_states['ALPHA 3']['x']=2.21
    assert PeerState.physical_guard(node,own,set()) is None
    assert node.physical_guard_peer==''


def test_stationary_yield_pocket_does_not_reverse_cleared_shared_winner():
    peer=state('ALPHA 1',2.93)
    peer['local_nav_state']='YIELD_POCKET'
    node=SimpleNamespace(robot_id='ALPHA 2',peer_states={'ALPHA 1':peer},
        book=Book(),path_guard_active='',emit=lambda *args,**kwargs:None,
        cfg={'stage6':{'physical_stop_distance_m':2.0}},physical_guard_peer='')
    own=state('ALPHA 2',0.)
    assert PeerState.proximity_guard(node,own,set()) is None
    assert PeerState.path_guard(node,own,set()) is None
    peer.update(local_nav_state='WAYPOINT_TRACK',yielding_to='ALPHA 2')
    assert PeerState.proximity_guard(node,own,set()) is None
    assert PeerState.path_guard(node,own,set()) is None
    # The same pocket remains a solid physical body at an unsafe distance.
    peer['x']=1.9
    assert PeerState.physical_guard(node,own,set())['reason']=='PEER_FOOTPRINT_BLOCKED'
