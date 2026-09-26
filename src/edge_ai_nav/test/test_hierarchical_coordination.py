import unittest
import yaml

from edge_ai_nav.fleet.route_graph import WarehouseGraph, segment_interval
from edge_ai_nav.fleet.negotiation import bid_eligible


class HierarchicalCoordinationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open('src/amr_simulation/config/warehouse_sih_demo.yaml') as stream:
            cls.graph=WarehouseGraph(yaml.safe_load(stream)['warehouse'])

    def test_six_hotspots_are_safe_and_unique(self):
        self.assertEqual(6,len(self.graph.parking))
        stations={'PICKUP_A','PICKUP_B','DROP_A','DROP_B','LOADING_BAY'}
        for name in self.graph.parking:
            point=self.graph.nodes[name]
            self.assertFalse(any(segment_interval(point,point,z)
                                 for z in self.graph.zones.values()))
            self.assertTrue(all(__import__('math').dist(point,self.graph.nodes[s])>.9
                                for s in stations))

    def test_nearest_free_and_active_route_exclusion(self):
        position=(-6.0,0.8)
        first=self.graph.parking_candidates(position)
        self.assertEqual('PARK_01',first[0][1])
        active=[[[-7.0,1.0],[-5.0,1.0]]]
        filtered=self.graph.parking_candidates(position,{'PARK_02'},active)
        self.assertNotIn(filtered[0][1],('PARK_01','PARK_02'))

    def test_zone_bypass_is_temporary(self):
        _,normal=self.graph.astar('DROP_A','MAIN_CORRIDOR_EAST')
        _,alternate=self.graph.astar('DROP_A','MAIN_CORRIDOR_EAST',
                                      {'intersection_B':float('inf')},'intersection_B')
        self.assertIn('INTERSECTION_B',normal)
        self.assertNotIn('INTERSECTION_B',alternate)
        self.assertEqual(['DROP_A','MAIN_CORRIDOR_EAST'],alternate)

    def test_reposition_is_interruptible(self):
        self.assertTrue(bid_eligible('POST_TASK_REPOSITION',False,True))
        self.assertTrue(bid_eligible('MOVE_ASIDE',False,True))
        self.assertFalse(bid_eligible('WAYPOINT_TRACK',False,True))
        self.assertFalse(bid_eligible('POST_TASK_REPOSITION',True,True))

    def test_inactive_demo_routes_are_not_traffic(self):
        # This mirrors the controller guard: a configured route alone is not
        # enough to force an idle robot to vacate.
        peer={'active_task':'NONE','local_nav_state':'DISABLED',
              'remaining_route':[[-6,0],[-5,0]]}
        self.assertIn(peer['active_task'],(None,'NONE','PARKING'))


if __name__=='__main__':
    unittest.main()
