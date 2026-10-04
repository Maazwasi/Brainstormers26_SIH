import unittest

from edge_ai_nav.fleet.negotiation import (NegotiationBook, choose_winner, resolve_conflict,
                                            conflict_identity, coordination_action,
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


class SharedDecisionProtocolTest(unittest.TestCase):
    cid='intersection_A:ALPHA:BRAVO'

    @staticmethod
    def active(rid,priority=3,**fields):
        value=dict(robot_id=rid,task_priority=priority,waiting_time=0,
                   local_nav_state='WAYPOINT_TRACK',zone_reserved=False,
                   zone_committed=False,safety_emergency=False)
        value.update(fields)
        return value

    def authority_decision(self,costs=None):
        book=NegotiationBook()
        decision=book.create_authoritative(self.cid,'ALPHA',
            self.active('ALPHA',4),self.active('BRAVO',2),
            {'ALPHA':2.0,'BRAVO':2.5},now=10.,first_detected=9.5,
            zone='intersection_A',route_costs=costs)
        return book,decision

    def test_shared_01_canonical_authority_from_both_perspectives(self):
        self.assertEqual(('intersection_A',('ALPHA','BRAVO'),'ALPHA'),
                         conflict_identity(self.cid))
        self.assertEqual('ALPHA',conflict_identity(
            'intersection_A:ALPHA:BRAVO')[2])

    def test_shared_02_staggered_peers_adopt_same_version_and_winner(self):
        authority,wire=self.authority_decision()
        peer=NegotiationBook()
        delayed=dict(wire,decision_timestamp=wire['decision_timestamp']+0.7)
        self.assertTrue(peer.adopt(delayed))
        adopted=peer.get(self.cid)
        self.assertEqual((wire['winner'],wire['loser'],wire['decision_version']),
                         (adopted['winner'],adopted['loser'],adopted['decision_version']))
        self.assertEqual(authority.get(self.cid)['authority_robot_id'],'ALPHA')

    def test_shared_03_double_loser_is_impossible_after_adoption(self):
        _,wire=self.authority_decision({'BRAVO':{
            'wait':8.,'reroute':2.,'route':[[0,0],[1,0]]}})
        peer=NegotiationBook(); self.assertTrue(peer.adopt(wire))
        actions={rid:coordination_action(peer.get(self.cid),rid)
                 for rid in ('ALPHA','BRAVO')}
        self.assertEqual(1,sum(action=='REROUTE' for action in actions.values()))
        self.assertEqual(1,sum(action=='PROCEED' for action in actions.values()))

    def test_shared_04_winner_never_reroutes(self):
        _,wire=self.authority_decision({'BRAVO':{
            'wait':8.,'reroute':2.,'route':[[0,0],[1,0]]}})
        self.assertEqual('PROCEED',coordination_action(wire,wire['winner']))
        self.assertNotEqual('REROUTE',coordination_action(wire,wire['winner']))

    def test_shared_05_loser_yields_and_preserves_route_policy(self):
        _,wire=self.authority_decision({'BRAVO':{
            'wait':2.,'reroute':20.,'route':[[0,0],[1,0]]}})
        self.assertEqual('YIELD_AND_WAIT',coordination_action(wire,wire['loser']))
        self.assertEqual('WAIT_FOR_CLEAR',coordination_action(wire,wire['loser'],True))

    def test_shared_06_only_loser_reroutes_when_cheaper(self):
        _,wire=self.authority_decision({'BRAVO':{
            'wait':8.,'reroute':2.,'route':[[0,0],[1,0]]}})
        self.assertEqual('REROUTE',coordination_action(wire,'BRAVO'))
        self.assertEqual('PROCEED',coordination_action(wire,'ALPHA'))

    def test_shared_07_clear_is_versioned_and_enables_resume(self):
        authority,wire=self.authority_decision()
        peer=NegotiationBook(); self.assertTrue(peer.adopt(wire))
        clear=authority.clear_message(self.cid,'WINNER_CLEARED',20.)
        self.assertEqual(wire['decision_version'],clear['decision_version'])
        removed=peer.adopt_clear(clear)
        self.assertEqual('BRAVO',removed['loser'])
        self.assertIsNone(peer.get(self.cid))

    def test_shared_08_stale_and_wrong_authority_decisions_are_rejected(self):
        authority,wire=self.authority_decision()
        peer=NegotiationBook(); self.assertTrue(peer.adopt(wire))
        clear=authority.clear_message(self.cid,'WINNER_CLEARED',20.)
        self.assertIsNotNone(authority.adopt_clear(clear))
        self.assertIsNotNone(peer.adopt_clear(clear))
        self.assertFalse(peer.adopt(wire))
        newer=dict(wire,decision_version=wire['decision_version']+1,
                   authority_robot_id='BRAVO')
        self.assertFalse(peer.adopt(newer))
        next_encounter=authority.create_authoritative(self.cid,'ALPHA',
            self.active('ALPHA',4),self.active('BRAVO',2),
            {'ALPHA':2.,'BRAVO':3.},now=30.,first_detected=29.,
            zone='intersection_A')
        self.assertEqual(wire['decision_version']+1,
                         next_encounter['decision_version'])

    def test_shared_09_missing_authority_decision_is_safe_wait(self):
        self.assertEqual('SAFE_WAIT',coordination_action(None,'BRAVO'))

    def test_shared_10_existing_hierarchy_is_preserved(self):
        alpha=self.active('ALPHA',4)
        bravo=self.active('BRAVO',1,zone_committed=True)
        decision=resolve_conflict(alpha,bravo,'intersection_A',
                                  etas={'ALPHA':1.,'BRAVO':5.})
        self.assertEqual(('BRAVO','ALPHA','ZONE_COMMITTED'),
                         (decision.winner,decision.loser,decision.reason))


if __name__ == '__main__':
    unittest.main()
