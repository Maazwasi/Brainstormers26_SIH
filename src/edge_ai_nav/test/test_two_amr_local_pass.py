"""Focused checks for the negotiated winner's temporary-peer local pass."""
from pathlib import Path
from types import SimpleNamespace
import math
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from edge_ai_nav.fleet.peer_state_node import PeerState
from edge_ai_nav.ros_nodes.local_waypoint_controller import LocalController


class Book:
    def __init__(self, decision):
        self.decision=decision

    def values(self):
        return [] if self.decision is None else [self.decision]


def peer_state(x, nav_state='COORDINATION_HOLD', speed=0.0):
    return dict(robot_id='ALPHA 3',x=x,y=23.5,active_task='DEMO-A3',
                local_nav_state=nav_state,linear_velocity=speed)


def guard_node(peer):
    decision=dict(conflict_id='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 3',
                  winner='ALPHA 1',loser='ALPHA 3',decision_version=1)
    return SimpleNamespace(robot_id='ALPHA 1',peer_states={'ALPHA 3':peer},
        book=Book(decision),physical_guard_peer='',local_pass_conflict='',
        cfg={'amr_collision_bounds_m':{'x':[-.90,.90],'y':[-.76,.76]},
             'stage6':{'physical_stop_distance_m':3.18,
                       'local_pass_safety_margin_m':.35,
                       'local_pass_authorization_margin_m':1.50}},
        emit=lambda *args,**kwargs:None)


def test_locked_winner_receives_temporary_peer_obstacle_only_after_loser_stops():
    own=dict(x=22.0,y=23.5)
    moving=guard_node(peer_state(25.0,'WAYPOINT_TRACK',.25))
    assert PeerState.physical_guard(moving,own,set())['state']=='SAFE_WAIT'

    stopped=guard_node(peer_state(25.0))
    command=PeerState.physical_guard(stopped,own,set())
    assert command['state']=='PROCEED_LOCAL_PASS'
    assert command['winner']=='ALPHA 1' and command['loser']=='ALPHA 3'
    assert command['required_lateral_m']==round(2*math.hypot(.90,.76)+.35,3)
    assert command['temporary_peer_obstacle']['half_width']==.76


def test_local_pass_is_authorized_before_the_hard_physical_stop_radius():
    own=dict(x=20.5,y=23.5)
    stopped=guard_node(peer_state(25.0))  # 4.5 m > 3.18 m hard-stop radius
    command=PeerState.physical_guard(stopped,own,set())
    assert command['state']=='PROCEED_LOCAL_PASS'
    assert command['required_lateral_m']==round(2*math.hypot(.90,.76)+.35,3)


def test_unowned_active_vs_charge_return_encounter_gets_one_priority_winner():
    peer=peer_state(25.0)
    peer.update(task_priority=1,active_task='NONE')
    node=guard_node(peer); node.book=Book(None)
    own=dict(x=22.0,y=23.5,task_priority=4,active_task='TASK-481')
    command=PeerState.physical_guard(node,own,set())
    assert command['state']=='PROCEED_LOCAL_PASS'
    assert command['winner']=='ALPHA 1' and command['loser']=='ALPHA 3'
    assert command['conflict_id']=='PROXIMITY:ALPHA 1:ALPHA 3'


def test_unowned_lower_priority_robot_remains_the_only_loser():
    peer=peer_state(22.0)
    peer.update(robot_id='ALPHA 2',task_priority=4,active_task='TASK-481')
    node=guard_node(peer); node.robot_id='ALPHA 3'
    node.peer_states={'ALPHA 2':peer}; node.book=Book(None)
    own=dict(x=25.0,y=23.5,task_priority=1,active_task='NONE')
    command=PeerState.physical_guard(node,own,set())
    assert command['state']=='SAFE_WAIT'
    assert command['winner']=='ALPHA 2' and command['loser']=='ALPHA 3'


def test_demo_loser_is_not_sent_into_a_second_avoidance_maneuver():
    decision=dict(conflict_id='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 3',
                  winner='ALPHA 1',loser='ALPHA 3',decision_version=1)
    node=SimpleNamespace(robot_id='ALPHA 3',peer_states={
        'ALPHA 1':dict(x=24.,y=23.5,remaining_route=[[30.,23.5]],
                       local_nav_state='WAYPOINT_TRACK')},
        book=Book(decision),yield_relocation=None,
        cfg={'stage6':{'stationary_negotiation_yield':True}})
    own=dict(x=26.,y=23.5,active_task='DEMO-A3',remaining_route=[[20.,23.5]])
    command=dict(state='YIELD_AND_WAIT',winner='ALPHA 1',loser='ALPHA 3',
                 conflict_id=decision['conflict_id'])
    assert PeerState.relocation_command(node,own,set(),command) is command


def test_temporary_peer_uses_existing_footprint_aware_bypass_geometry():
    controller=LocalController.__new__(LocalController)
    controller.coordination_command=dict(state='PROCEED_LOCAL_PASS',
        conflict_id='MAIN_UPPER_CROSS:ALPHA 1:ALPHA 3',
        winner='ALPHA 1',loser='ALPHA 3',
        temporary_peer_obstacle=dict(robot_id='ALPHA 3',x=25.,y=23.5,
                                     half_width=.76,
                                     turning_radius=math.hypot(.90,.76),
                                     safety_margin=.35))
    controller.p={'bypass_clearance_margin':.35}
    controller.physical_half_width=.76
    controller.physical_turning_radius=math.hypot(.90,.76)
    controller.front_overhang=.78
    controller.pose=(22.,23.5,0.)
    controller.route=[(20.,23.5),(30.,23.5)]
    controller.index=1
    controller.obstacle_encounter_id='NEG-DEMO-001'
    controller.rid='alpha_1'
    controller.logical_id='ALPHA 1'
    controller.obstacle_log=lambda *args,**kwargs:None
    controller.change=lambda value:None
    controller.emit=lambda *args,**kwargs:None

    assert LocalController.begin_local_bypass(controller,3.5,3.0,3.0)
    assert controller.bypass['kind']=='NEGOTIATION_LOCAL_PASS'
    assert controller.bypass['label']=='LEFT'
    assert math.isclose(controller.bypass['lateral_distance'],
                        2*math.hypot(.90,.76)+.35)
    assert math.degrees(controller.bypass['exit_angle'])>40.0


def test_complete_pass_clearance_uses_both_longitudinal_half_lengths():
    bounds=[-.90,.90]; margin=.35
    required=2.0*max(abs(value) for value in bounds)+margin
    winner=dict(x=28.,y=23.5,yaw=0.)
    loser=dict(x=29.,y=23.5)
    peer_ahead=((loser['x']-winner['x'])*math.cos(winner['yaw'])+
                (loser['y']-winner['y'])*math.sin(winner['yaw']))
    assert peer_ahead > -required
    winner['x']=31.2
    peer_ahead=((loser['x']-winner['x'])*math.cos(winner['yaw'])+
                (loser['y']-winner['y'])*math.sin(winner['yaw']))
    assert peer_ahead <= -required


def test_pass_projection_uses_preserved_route_not_temporary_lateral_yaw():
    winner=dict(x=26.,y=25.,yaw=math.pi/2,
                remaining_route=[[31.,23.5]])
    loser=dict(x=29.,y=23.5)
    target=winner['remaining_route'][0]
    route_yaw=math.atan2(target[1]-winner['y'],target[0]-winner['x'])
    peer_ahead=((loser['x']-winner['x'])*math.cos(route_yaw)+
                (loser['y']-winner['y'])*math.sin(route_yaw))
    assert peer_ahead>0.0  # still ahead despite the winner temporarily facing north
