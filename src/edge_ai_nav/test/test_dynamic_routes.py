"""Focused checks for the dynamic route/controlled-zone integration."""
import pathlib
import unittest
import yaml
from edge_ai_nav.fleet.route_graph import WarehouseGraph
from edge_ai_nav.fleet.zone_prediction import LocalDetector


class DynamicRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=pathlib.Path(__file__).resolve().parents[2]/'amr_simulation/config/warehouse_sih_demo.yaml'
        cls.graph=WarehouseGraph(yaml.safe_load(path.read_text())['warehouse'])

    def test_current_start_and_selected_destinations(self):
        g=self.graph; p=[-7.5,.3]
        a=g.mission(p,'PICKUP_A','DROP_A'); b=g.mission(p,'PICKUP_A','DROP_B')
        self.assertEqual(a[0],p); self.assertEqual(b[0],p)
        self.assertIn(g.nodes['PICKUP_A'],a); self.assertIn(g.nodes['PICKUP_A'],b)
        self.assertEqual(a[-1],g.nodes['DROP_A']); self.assertEqual(b[-1],g.nodes['DROP_B'])
        self.assertNotEqual(a[-2:],b[-2:])
        self.assertEqual(g.mission(p,'PICKUP_B','LOADING_BAY')[-1],g.nodes['LOADING_BAY'])

    def test_bid_distance_and_unknown_station(self):
        g=self.graph
        self.assertLess(g.pickup_distance([-9.2,-4.7],'PICKUP_A'),g.pickup_distance([0,0],'PICKUP_A'))
        with self.assertRaises(ValueError): g.mission([0,0],'PICKUP_A','NOT_A_STATION')
        self.assertFalse(g.visible([-5,4],[0,4]))

    def test_astar_penalty_uses_valid_alternate_only_when_requested(self):
        g=self.graph
        normal_cost,normal=g.astar('DROP_A','MAIN_CORRIDOR_EAST')
        alternate_cost,alternate=g.astar('DROP_A','MAIN_CORRIDOR_EAST',
            {'intersection_B':float('inf')},'intersection_B')
        self.assertIn('INTERSECTION_B',normal)
        self.assertNotIn('INTERSECTION_B',alternate)
        self.assertGreaterEqual(alternate_cost,0.)
        self.assertGreaterEqual(normal_cost,0.)

    def test_astar_reports_real_search_work_and_smooth_plan(self):
        stats={}
        _, raw=self.graph.astar('DROP_A','MAIN_CORRIDOR_EAST',stats=stats)
        plan=self.graph.mission_plan([-7.5,.3],'PICKUP_A','DROP_A')
        self.assertGreater(stats['expanded_nodes'],0)
        self.assertEqual(stats['heuristic_weight'],1.0)
        self.assertEqual(raw[0],'DROP_A')
        self.assertEqual(plan['planner'],'A*')
        self.assertGreater(plan['expanded_nodes'],0)
        self.assertLessEqual(len(plan['smoothed_waypoints']),len(plan['raw_waypoints']))

    def test_aisle_wait_pockets_and_exclusive_prediction(self):
        g=self.graph
        a=g.mission([8,.65],'NARROW_AISLE_SOUTH','AISLE_NORTH')
        b=g.mission([8,8.15],'NARROW_AISLE_NORTH','AISLE_SOUTH')
        self.assertIn(g.nodes['AISLE_SOUTH_HOLD'],a)
        self.assertIn(g.nodes['AISLE_NORTH_HOLD'],b)
        states=[]
        for rid,route in [('DELTA',a),('ECHO',b)]:
            intent=g.intent(route[0],route)
            states.append(dict(robot_id=rid,next_zone=intent['zone'],zone_intent=intent))
        left=LocalDetector(g.zones); right=LocalDetector(g.zones)
        left.update(states[0],[states[1]],0); right.update(states[1],[states[0]],0)
        self.assertEqual(set(left.active),{'narrow_aisle_1:DELTA:ECHO'})
        self.assertEqual(set(left.active),set(right.active))


if __name__=='__main__': unittest.main()
