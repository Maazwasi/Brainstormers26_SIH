import unittest

from edge_ai_nav.fleet.negotiation import (NegotiationBook, choose_winner,
                                            motion_gate, outside_with_margin)


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
        self.assertEqual('BRAVO', choose_winner(state('ALPHA', 3, 1), state('BRAVO', 3, 4), {'ALPHA': 1, 'BRAVO': 9}).winner)
        self.assertEqual('ALPHA', choose_winner(state('ALPHA'), state('BRAVO'), {'ALPHA': 1, 'BRAVO': 3}).winner)
        self.assertEqual('ALPHA', choose_winner(state('ALPHA'), state('BRAVO'), {'ALPHA': 1, 'BRAVO': 1}).winner)

    def test_latch_does_not_flip(self):
        book = NegotiationBook(); alpha, bravo = state('ALPHA', 3), state('BRAVO', 4)
        initial = book.decide('intersection_A:ALPHA:BRAVO', alpha, bravo, {'ALPHA': 1, 'BRAVO': 5}, 12, 11)
        later = book.decide('intersection_A:ALPHA:BRAVO', alpha, bravo, {'ALPHA': .1, 'BRAVO': 99}, 13, 11)
        self.assertEqual('BRAVO', initial['winner'])
        self.assertEqual(initial, later)

    def test_lidar_precedence_and_clearance_margin(self):
        self.assertEqual('LIDAR_STOP', motion_gate(.2, .65, True))
        self.assertEqual('COORDINATION_HOLD', motion_gate(2, .65, True))
        zone = {'center': [0, 0], 'radius': 1}
        self.assertFalse(outside_with_margin({'x': 1.2, 'y': 0}, zone, .4))
        self.assertTrue(outside_with_margin({'x': 1.4, 'y': 0}, zone, .4))


if __name__ == '__main__':
    unittest.main()
