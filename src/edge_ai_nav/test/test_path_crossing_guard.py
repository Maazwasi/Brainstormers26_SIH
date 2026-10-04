"""V2 path guard catches a crossing missed by fixed main-aisle zones."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from edge_ai_nav.fleet.path_safety import approaching_crossing, should_yield_for_path


def state(name, priority, x, y, route):
    return dict(robot_id=name,task_priority=priority,x=x,y=y,
                linear_velocity=0.35,remaining_route=route)


def test_return_to_charge_crossing_is_detected_before_proximity():
    returning=state('ALPHA 2',0,14.0,6.3,[[25.0,16.5],[25.0,32.5]])
    delivering=state('ALPHA 4',2,16.0,11.1,[[18.0,4.0]])
    conflict=approaching_crossing(returning,delivering)
    assert conflict is not None
    assert 16.0 < conflict['point'][0] < 17.0
    assert 8.0 < conflict['point'][1] < 10.0
    assert conflict['own_distance'] > 2.0
    assert should_yield_for_path(returning,delivering)
    assert not should_yield_for_path(delivering,returning)


def test_guard_clears_after_winner_passes_and_ignores_parallel_lanes():
    returning=state('ALPHA 2',0,14.0,6.3,[[25.0,16.5]])
    passed=state('ALPHA 4',2,17.8,4.7,[[18.0,4.0]])
    parallel=state('ALPHA 4',2,14.0,9.0,[[25.0,19.2]])
    assert approaching_crossing(returning,passed) is None
    assert approaching_crossing(returning,parallel) is None


def test_equal_priority_has_one_deterministic_yielder():
    one=state('ALPHA 1',2,0,0,[[2,2]])
    two=state('ALPHA 2',2,2,0,[[0,2]])
    assert approaching_crossing(one,two)
    assert not should_yield_for_path(one,two)
    assert should_yield_for_path(two,one)
