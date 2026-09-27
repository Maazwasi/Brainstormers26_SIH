import unittest

from edge_ai_nav.fleet.negotiation import (NegotiationBook, choose_winner, resolve_conflict,
                                            motion_gate, outside_with_margin,
                                            reservation_distance)


def state(robot_id, priority=3, waiting=0):
    return {'robot_id': robot_id, 'task_priority': priority, 'waiting_time': waiting}


class NegotiationTest(unittest.TestCase):
    def test_priority_and_reverse_invocation(self):
        alpha, bravo = state('ALPHA', 3), state('BRAVO', 4)
        first = choose_winner(alpha, bravo, {'ALPHA': 3, 'BRAVO': 3})
        reverse = choose_winner(bravo, alpha, {'ALPHA': 3, 'BRAVO': 3})
        self.assertEqual(('BRAVO', 'ALPHA', 'TASK_PRIORITY'), (first.winner, first.loser, first.reason))
        self.assertEqual(first, reverse)

    def test_wait_eta_and_id_ties(self):
        # Starvation/waiting time precedes ETA in the shared hierarchy.
        self.assertEqual('BRAVO', choose_winner(state('ALPHA', 3, 1), state('BRAVO', 3, 4), {'ALPHA': 1, 'BRAVO': 9}).winner)
        self.assertEqual('ALPHA', choose_winner(state('ALPHA'), state('BRAVO'), {'ALPHA': 1, 'BRAVO': 3}).winner)
        self.assertEqual('ALPHA', choose_winner(state('ALPHA'), state('BRAVO'), {'ALPHA': 1, 'BRAVO': 1}).winner)

    def test_hierarchy_activity_commitment_and_cost_action(self):
        active=lambda rid,p: dict(state(rid,p),local_nav_state='WAYPOINT_TRACK')
        idle=dict(state('BRAVO',4),local_nav_state='PARKED')
        self.assertEqual('ALPHA',resolve_conflict(active('ALPHA',1),idle,'intersection_A',
            etas={'ALPHA':4,'BRAVO':1}).winner)
        committed=dict(active('BRAVO',1),zone_committed=True)
        self.assertEqual('BRAVO',resolve_conflict(active('ALPHA',4),committed,'intersection_A',
            etas={'ALPHA':1,'BRAVO':5}).winner)
        costs={'ALPHA':{'wait':2,'reroute':99},
               'BRAVO':{'wait':8,'reroute':3,'route':[[1,0],[2,0]]}}
        result=resolve_conflict(active('ALPHA',4),active('BRAVO',1),'intersection_B',costs,
                                {'ALPHA':1,'BRAVO':2})
        self.assertEqual(('ALPHA','BRAVO','REROUTE'),
                         (result.winner,result.loser,result.loser_action))
        costs['BRAVO']={'wait':2,'reroute':20,'route':[[1,0],[2,0]]}
        self.assertEqual('YIELD_AND_WAIT',resolve_conflict(active('ALPHA',4),active('BRAVO',1),
            'intersection_B',costs,{'ALPHA':1,'BRAVO':2}).loser_action)

    def test_latch_does_not_flip(self):
        book = NegotiationBook(); alpha, bravo = state('ALPHA', 3), state('BRAVO', 4)
        initial = book.decide('intersection_A:ALPHA:BRAVO', alpha, bravo, {'ALPHA': 1, 'BRAVO': 5}, 12, 11)
        later = book.decide('intersection_A:ALPHA:BRAVO', alpha, bravo, {'ALPHA': .1, 'BRAVO': 99}, 13, 11)
        self.assertEqual('BRAVO', initial['winner'])
        self.assertEqual(initial, later)

    def test_only_loser_changes_and_idle_moves_aside(self):
        high=dict(state('ALPHA',4),local_nav_state='WAYPOINT_TRACK')
        low=dict(state('BRAVO',1),local_nav_state='WAYPOINT_TRACK')
        decision=resolve_conflict(high,low,'intersection_A',
            {'BRAVO':{'wait':2.,'reroute':9.}}, {'ALPHA':2.,'BRAVO':2.})
        self.assertEqual(('ALPHA','BRAVO','YIELD_AND_WAIT'),
                         (decision.winner,decision.loser,decision.loser_action))
        idle=dict(state('BRAVO',4),local_nav_state='AVAILABLE')
        decision=resolve_conflict(high,idle,'intersection_A',etas={'ALPHA':3.,'BRAVO':1.})
        self.assertEqual(('ALPHA','BRAVO','MOVE_ASIDE'),
                         (decision.winner,decision.loser,decision.loser_action))

    def test_speed_aware_reservation_is_conservatively_bounded(self):
        self.assertEqual(1.5,reservation_distance(0.0))
        self.assertGreaterEqual(reservation_distance(0.5),reservation_distance(0.1))
        self.assertLessEqual(reservation_distance(4.0),2.0)

    def test_connected_three_robot_pairs_remain_asymmetric(self):
        alpha=dict(state('ALPHA',4),local_nav_state='WAYPOINT_TRACK')
        bravo=dict(state('BRAVO',2),local_nav_state='WAYPOINT_TRACK')
        charlie=dict(state('CHARLIE',1),local_nav_state='WAYPOINT_TRACK')
        first=resolve_conflict(alpha,bravo,'intersection_A',etas={'ALPHA':2,'BRAVO':2})
        second=resolve_conflict(bravo,charlie,'intersection_B',etas={'BRAVO':2,'CHARLIE':2})
        self.assertEqual(('ALPHA','BRAVO'),(first.winner,first.loser))
        self.assertEqual(('BRAVO','CHARLIE'),(second.winner,second.loser))
        self.assertNotEqual(first.winner,first.loser)
        self.assertNotEqual(second.winner,second.loser)

    def test_narrow_aisle_has_one_latched_winner_with_measured_clock(self):
        book=NegotiationBook()
        alpha=dict(state('ALPHA',3),local_nav_state='WAYPOINT_TRACK')
        bravo=dict(state('BRAVO',3),local_nav_state='WAYPOINT_TRACK')
        decision=book.decide('narrow_aisle_1:ALPHA:BRAVO',alpha,bravo,
            {'ALPHA':3,'BRAVO':3},now=10.,first_detected=10.,
            zone='narrow_aisle_1')
        self.assertEqual('ALPHA',decision['winner'])
        self.assertEqual('BRAVO',decision['loser'])
        self.assertGreater(decision['detection_monotonic_ns'],0)
        self.assertGreaterEqual(decision['decision_latency_ms'],0.0)

    def test_lidar_precedence_and_clearance_margin(self):
        self.assertEqual('LIDAR_STOP', motion_gate(.2, .65, True))
        self.assertEqual('COORDINATION_HOLD', motion_gate(2, .65, True))
        zone = {'center': [0, 0], 'radius': 1}
        self.assertFalse(outside_with_margin({'x': 1.2, 'y': 0}, zone, .4))
        self.assertTrue(outside_with_margin({'x': 1.4, 'y': 0}, zone, .4))


if __name__ == '__main__':
    unittest.main()
