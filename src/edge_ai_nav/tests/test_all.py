"""Comprehensive test suite for Edge AI + GNSS/IDR prototype."""

import unittest
import math
import time
import numpy as np

from edge_ai_nav.sensor_layer.data_types import SensorFrame, GNSSData, IMUData, NavigationOutput
from edge_ai_nav.sensor_layer.scenario_generator import ScenarioGenerator, enu_to_wgs84, wgs84_to_enu
from edge_ai_nav.outage_detector.detector import GNSSOutageDetector
from edge_ai_nav.idr_engine.imu_preprocessor import IMUPreprocessor
from edge_ai_nav.idr_engine.attitude_filter import AttitudeFilter
from edge_ai_nav.idr_engine.dead_reckoner import IntelligentDeadReckoner
from edge_ai_nav.fusion_engine.ekf_fusion import EKFNavFusion
from edge_ai_nav.edge_ai.feature_extractor import NavigationFeatureExtractor
from edge_ai_nav.edge_ai.model import EdgeAIEnvironmentModel
from edge_ai_nav.edge_ai.classifier import EdgeAIClassifier
from edge_ai_nav.pipeline import NavigationPipeline


class TestSensorLayer(unittest.TestCase):
    def test_geodetic_conversions(self):
        ref_lat, ref_lon = 28.6139, 77.2090
        e_orig, n_orig = 150.0, -85.0
        lat, lon, _ = enu_to_wgs84(e_orig, n_orig, ref_lat, ref_lon)
        e_rec, n_rec = wgs84_to_enu(lat, lon, ref_lat, ref_lon)
        self.assertAlmostEqual(e_orig, e_rec, places=3)
        self.assertAlmostEqual(n_orig, n_rec, places=3)

    def test_scenario_generator_stepping(self):
        gen = ScenarioGenerator(rate_hz=50.0)
        f1 = gen.step()
        self.assertIsInstance(f1, SensorFrame)
        self.assertEqual(f1.frame_id, 1)
        self.assertAlmostEqual(f1.imu.az, 9.81, delta=1.5)


class TestOutageDetector(unittest.TestCase):
    def test_outage_detection_criteria(self):
        det = GNSSOutageDetector(min_satellites=4, max_hdop_outage=3.5)

        # 1. Normal frame
        f_good = SensorFrame(
            timestamp=time.time(),
            gnss=GNSSData(lat=28.6, lon=77.2, valid=True, fix_type=1, num_sats=10, hdop=1.0)
        )
        mode, _ = det.update(f_good)
        self.assertEqual(mode, GNSSOutageDetector.MODE_GNSS_FIX)

        # 2. Satellites drop (Tunnel entry)
        f_bad_sats = SensorFrame(
            timestamp=time.time(),
            gnss=GNSSData(lat=28.6, lon=77.2, valid=True, fix_type=1, num_sats=2, hdop=1.0)
        )
        mode, reason = det.update(f_bad_sats)
        self.assertEqual(mode, GNSSOutageDetector.MODE_IDR_ACTIVE)
        self.assertIn("Satellites Dropped", reason)

        # 3. Restoration
        for _ in range(5):
            mode, _ = det.update(f_good)
        self.assertIn(mode, (GNSSOutageDetector.MODE_FUSION_CONVERGING, GNSSOutageDetector.MODE_GNSS_FIX))

    def test_late_first_valid_fix_does_not_latch_startup_timeout(self):
        det = GNSSOutageDetector(timeout_threshold_sec=1.2)
        det.last_valid_gnss_time = time.time() - 5.0
        valid = SensorFrame(
            timestamp=time.time(),
            gnss=GNSSData(lat=28.6, lon=77.2, valid=True, fix_type=1, num_sats=10, hdop=1.0),
        )

        mode, _ = det.update(valid)

        self.assertEqual(GNSSOutageDetector.MODE_GNSS_FIX, mode)


class TestIDREngine(unittest.TestCase):
    def test_zupt_detection(self):
        prep = IMUPreprocessor(window_size=5)
        # Push 5 stationary frames
        for _ in range(6):
            is_stat, cal_imu, tilt = prep.process(IMUData(ax=0.01, ay=0.01, az=9.81, gx=0.0, gy=0.0, gz=0.0))
        self.assertTrue(is_stat)
        self.assertAlmostEqual(tilt[0], 0.0, places=2)

    def test_dead_reckoning_motion(self):
        dr = IntelligentDeadReckoner(ref_lat=28.6139, ref_lon=77.2090)
        # Apply forward acceleration heading North (0 deg)
        for _ in range(10):
            dr.update(IMUData(ax=2.0, ay=0.0, az=9.81), heading_deg=0.0, pitch_rad=0.0, is_stationary=False, is_outage=True)
        self.assertGreater(dr.north, 0.0)
        self.assertGreater(dr.speed, 0.0)
        self.assertGreater(dr.accumulated_drift, 0.0)


class TestEKFFusion(unittest.TestCase):
    def test_ekf_predict_update(self):
        ekf = EKFNavFusion(ref_lat=28.6139, ref_lon=77.2090)
        ekf.initialize(28.6139, 77.2090, 216.0, heading_deg=0.0)
        self.assertTrue(ekf.is_initialized)

        # Predict step
        ekf.predict(IMUData(ax=1.0, ay=0.0, az=9.81, gz=0.0), pitch_rad=0.0, is_stationary=False, dt=0.02)
        self.assertGreater(ekf.x[3, 0], 0.0) # North velocity increased

        # GNSS Update step
        gnss = GNSSData(lat=28.61391, lon=77.2090, valid=True, speed=1.0, heading=0.0, hdop=1.0)
        updated = ekf.update_gnss(gnss)
        self.assertTrue(updated)


class TestEdgeAI(unittest.TestCase):
    def test_edge_ai_inference(self):
        classifier = EdgeAIClassifier(window_size=5)
        
        # Test open sky classification
        f_open = SensorFrame(
            gnss=GNSSData(lat=28.6, lon=77.2, valid=True, num_sats=11, hdop=0.8, speed=5.0),
            imu=IMUData(ax=0.1, ay=0.0, az=9.81)
        )
        for _ in range(5):
            c_name, conf, scale, probs = classifier.classify(f_open, est_speed=5.0)
        
        self.assertEqual(c_name, "OPEN_SKY_NORMAL")
        self.assertGreater(conf, 0.7)
        self.assertLess(classifier.last_latency_ms, 2.0) # Sub-millisecond!

        # Test tunnel outage classification
        f_tunnel = SensorFrame(
            gnss=GNSSData(valid=False, num_sats=0, hdop=99.0),
            imu=IMUData(ax=0.0, ay=0.5, az=9.81, gz=0.2), # curving inside tunnel
            is_simulated_outage=True
        )
        for _ in range(5):
            c_name, conf, scale, probs = classifier.classify(f_tunnel, est_speed=5.0)
        
        self.assertEqual(c_name, "TUNNEL_OUTAGE")
        self.assertGreater(conf, 0.8)


class TestFullPipeline(unittest.TestCase):
    def test_pipeline_transition_flow(self):
        pipeline = NavigationPipeline(enable_logging=False)
        
        # Step through initial open sky
        for _ in range(10):
            out = pipeline.step()
        self.assertIn(out.mode, ("GNSS_FIX", "GNSS_DEGRADED"))

        # Inject outage
        pipeline.toggle_outage(force=True)
        for _ in range(5):
            out = pipeline.step()
        self.assertEqual(out.mode, "IDR_ACTIVE")
        self.assertGreater(out.outage_duration, 0.0)

        # Restore GNSS
        pipeline.toggle_outage(force=False)
        for _ in range(5):
            out = pipeline.step()
        self.assertIn(out.mode, ("FUSION_CONVERGING", "GNSS_FIX"))

    def test_step_retains_the_processed_sensor_frame(self):
        pipeline = NavigationPipeline(enable_logging=False)

        out = pipeline.step()

        self.assertIsNotNone(pipeline.last_frame)
        self.assertEqual(out.timestamp, pipeline.last_frame.timestamp)


if __name__ == '__main__':
    unittest.main()
