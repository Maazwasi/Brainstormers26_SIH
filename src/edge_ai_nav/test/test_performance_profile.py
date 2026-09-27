import json
import tempfile
import unittest

from edge_ai_nav.fleet.performance_profile import PerformanceProfile


def mission(**updates):
    data = dict(route_efficiency=.94, completion_time_s=20.,
                average_heading_error_rad=.08, negotiation_wait_time_s=.4,
                unnecessary_stop_count=0, distance_rerouted_m=0.,
                heading_correction_count=0, turn_overshoot_rad=.05,
                task_success=True)
    data.update(updates)
    return data


class ProfileTests(unittest.TestCase):
    def test_shared_baseline_then_slow_bounded_adaptation(self):
        p=PerformanceProfile('amr_alpha')
        p.record(mission()); p.record(mission())
        self.assertEqual((1.,1.,1.),(p.speed_factor,p.steering_factor,p.lookahead_factor))
        for _ in range(20):
            p.record(mission(heading_correction_count=8,turn_overshoot_rad=.4))
        self.assertEqual(.95,p.speed_factor)
        self.assertEqual(.95,p.steering_factor)
        self.assertEqual(1.,p.lookahead_factor)

    def test_real_averages_and_persistence(self):
        p=PerformanceProfile('amr_bravo')
        p.record(mission(route_efficiency=.8,completion_time_s=10.))
        p.record(mission(route_efficiency=1.,completion_time_s=20.))
        self.assertAlmostEqual(.9,p.average_route_efficiency)
        self.assertAlmostEqual(15.,p.average_task_time)
        with tempfile.TemporaryDirectory() as folder:
            path=folder+'/profile.json'; p.save(path)
            loaded=PerformanceProfile.load('amr_bravo',path)
            self.assertEqual(2,loaded.missions_completed)
            self.assertEqual(p.dictionary(),loaded.dictionary())
            self.assertEqual('amr_bravo',json.load(open(path))['robot_id'])


if __name__ == '__main__': unittest.main()
