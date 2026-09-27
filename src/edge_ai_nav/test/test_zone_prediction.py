import unittest
from edge_ai_nav.fleet.zone_prediction import LocalDetector, conflict_id, overlap, window

ZONE = {'center': [0., 0.], 'radius': 1.05}


def state(rid, x=-1.55, y=0., zone='intersection_A'):
    return dict(robot_id=rid, x=x, y=y, yaw=0., linear_velocity=0.,
                goal_x=0., goal_y=0., next_zone=zone, local_nav_state='WAYPOINT_TRACK')


class PredictionTests(unittest.TestCase):
    def test_overlap(self):
        self.assertAlmostEqual(overlap((2, 3), (2.2, 4)), .8)

    def test_pair_order(self):
        self.assertEqual(conflict_id('intersection_A','ALPHA','BRAVO'), conflict_id('intersection_A','BRAVO','ALPHA'))

    def test_stopped_eta(self):
        self.assertAlmostEqual(window(state('A'), ZONE)['eta'], 2.)

    def test_horizon(self):
        self.assertIsNone(window(state('A', -4.), ZONE))

    def test_geometry_misses(self):
        a = state('A'); a.update(goal_x=-1.55, goal_y=3.)
        self.assertIsNone(window(a, ZONE))

    def test_moving_away(self):
        a = state('A'); a.update(yaw=3.14159265359, linear_velocity=.25)
        self.assertIsNone(window(a, ZONE))

    def test_detection_dedup_clear_loss(self):
        detector = LocalDetector({'intersection_A':ZONE})
        a, b = state('ALPHA'), state('BRAVO', 0., -1.6)
        self.assertEqual(detector.update(a, [b], 10)[0][0], 'CONFLICT_PREDICTED')
        self.assertEqual(detector.update(a, [b], 11), [])
        self.assertEqual(detector.event_count, 1)
        self.assertEqual(next(iter(detector.active.values()))['conflict_first_detected_time'], 10)
        self.assertGreater(next(iter(detector.active.values()))['conflict_first_detected_monotonic_ns'],0)
        b['next_zone'] = ''
        self.assertEqual(detector.update(a, [b], 12)[0][0], 'CONFLICT_CLEARED')
        b['next_zone'] = 'intersection_A'
        detector.update(a, [b], 13)
        self.assertEqual(detector.update(a, [], 15, {'BRAVO'})[0][0], 'CONFLICT_INVALIDATED_PEER_OFFLINE')

    def test_same_zone_different_time(self):
        detector = LocalDetector({'intersection_A':ZONE})
        detector.update(state('ALPHA'), [state('BRAVO', 0., -3.3)], 1)
        self.assertFalse(detector.active)

    def test_nonoverlap_within_horizon(self):
        detector = LocalDetector({'intersection_A':{'center':[0.,0.], 'radius':.1}})
        detector.update(state('ALPHA', -.2), [state('BRAVO', 0., -1.5)], 1)
        self.assertFalse(detector.active)

    def test_disabled_outside(self):
        a = state('ALPHA'); a['local_nav_state']='DISABLED'
        self.assertIsNone(window(a, ZONE))


if __name__ == '__main__': unittest.main()
