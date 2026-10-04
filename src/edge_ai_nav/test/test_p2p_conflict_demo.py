import math
import os
import unittest
import yaml

from edge_ai_nav.fleet.p2p_conflict_demo import EAST, NORTH, SOUTH_HOLD, WEST_HOLD
from edge_ai_nav.fleet.route_graph import WarehouseGraph, segment_interval


class P2PConflictDemoRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        with open(os.path.join(root,'amr_simulation','config','warehouse_sih_demo.yaml')) as stream:
            cls.cfg=yaml.safe_load(stream)['warehouse']
        cls.graph=WarehouseGraph(cls.cfg)

    def test_staging_holds_are_real_safe_graph_nodes(self):
        zone=self.graph.zones['intersection_A']
        for name in (WEST_HOLD,SOUTH_HOLD):
            self.assertIn(name,self.graph.nodes)
            self.assertAlmostEqual(1.95,
                math.dist(self.graph.nodes[name],zone['center'])-zone['radius'],places=2)

    def test_both_demo_routes_genuinely_cross_intersection_a(self):
        zone=self.graph.zones['intersection_A']
        bravo=self.graph.mission_plan(self.graph.nodes[WEST_HOLD],WEST_HOLD,EAST)
        delta=self.graph.mission_plan(self.graph.nodes[SOUTH_HOLD],SOUTH_HOLD,NORTH)
        for route in (bravo,delta):
            self.assertIn('INTERSECTION_A',route['raw_astar_nodes'])
            self.assertTrue(any(segment_interval(a,b,zone)
                for a,b in zip(route['smoothed_waypoints'],route['smoothed_waypoints'][1:])))


if __name__ == '__main__':
    unittest.main()
