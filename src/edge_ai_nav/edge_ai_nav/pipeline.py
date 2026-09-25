"""Central Navigation Pipeline orchestrating Sensor Layer, AI, IDR, Outage Detector, and Fusion."""

import time
import math
from typing import Optional, Callable, Dict, Any, List

from .sensor_layer.data_types import SensorFrame, NavigationOutput
from .sensor_layer.scenario_generator import ScenarioGenerator
from .sensor_layer.mobile_streamer import MobileStreamReceiver
from .sensor_layer.logger import TelemetryLogger
from .idr_engine.imu_preprocessor import IMUPreprocessor
from .idr_engine.attitude_filter import AttitudeFilter
from .idr_engine.dead_reckoner import IntelligentDeadReckoner
from .outage_detector.detector import GNSSOutageDetector
from .fusion_engine.ekf_fusion import EKFNavFusion
from .edge_ai.classifier import EdgeAIClassifier


class NavigationPipeline:
    """Master pipeline executing the full Edge AI + GNSS/IDR navigation cycle."""

    def __init__(self,
                 ref_lat: float = 28.6139,
                 ref_lon: float = 77.2090,
                 ref_alt: float = 216.0,
                 enable_logging: bool = True):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

        # Core Components
        self.scenario_gen = ScenarioGenerator(ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt)
        self.mobile_receiver = MobileStreamReceiver()
        self.imu_prep = IMUPreprocessor()
        self.att_filter = AttitudeFilter()
        self.idr = IntelligentDeadReckoner(ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt)
        self.outage_detector = GNSSOutageDetector()
        self.ekf = EKFNavFusion(ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt)
        self.ai_classifier = EdgeAIClassifier()

        self.logger = TelemetryLogger() if enable_logging else None
        self.output_subscribers: List[Callable[[SensorFrame, NavigationOutput], None]] = []

        # Internal state
        self.last_step_time = time.time()
        self.use_mobile_source = False
        self.initialized = False
        self.last_frame: Optional[SensorFrame] = None

    def add_subscriber(self, callback: Callable[[SensorFrame, NavigationOutput], None]) -> None:
        """Register a callback to receive real-time navigation outputs."""
        self.output_subscribers.append(callback)

    def toggle_outage(self, force: Optional[bool] = None) -> bool:
        """Simulate or toggle GNSS outage manually."""
        return self.scenario_gen.toggle_manual_outage(force)

    def set_mobile_source(self, enable: bool) -> None:
        """Switch input source between scenario generator and live smartphone."""
        self.use_mobile_source = enable

    def ingest_mobile_payload(self, payload: Dict[str, Any]) -> None:
        """Ingest live smartphone sensor payload."""
        self.mobile_receiver.ingest(payload)
        self.use_mobile_source = True

    def process_frame(self, frame: SensorFrame) -> NavigationOutput:
        """Run one end-to-end iteration across all 6 pipeline phases."""
        self.last_frame = frame
        now = frame.timestamp
        dt = max(0.001, min(0.1, now - self.last_step_time))
        self.last_step_time = now

        # Phase 4A: IMU Preprocessing (ZUPT Stationary Detection & Gyro Bias Removal)
        is_stationary, cal_imu, tilt_angles = self.imu_prep.process(frame.imu)
        roll_rad, pitch_rad = tilt_angles

        # Phase 6: Edge AI Condition Classification
        ai_cond, ai_conf, adaptive_scale, _ = self.ai_classifier.classify(frame, est_speed=self.idr.speed)

        # Phase 3: GNSS Outage Detection
        nav_mode, reason = self.outage_detector.update(frame, ai_condition=ai_cond)
        is_outage = (nav_mode == GNSSOutageDetector.MODE_IDR_ACTIVE)

        # Phase 4B: Attitude & Heading Filter
        gnss_heading = frame.gnss.heading if frame.gnss.valid else None
        roll_deg, pitch_deg, heading_deg = self.att_filter.update(
            cal_imu, tilt_angles,
            gnss_heading_deg=gnss_heading,
            gnss_valid=frame.gnss.valid,
            speed=self.idr.speed
        )

        # Phase 4C: Intelligent Dead Reckoning (IDR) Position Propagation
        if frame.gnss.valid and not is_outage and not self.initialized:
            # First-time sync
            self.idr.sync_with_gnss(frame.gnss)
            self.ekf.initialize(frame.gnss.lat, frame.gnss.lon, frame.gnss.alt, heading_deg)
            self.initialized = True
        elif not self.initialized:
            self.ekf.initialize(self.ref_lat, self.ref_lon, self.ref_alt, heading_deg)
            self.initialized = True

        idr_lat, idr_lon, idr_e, idr_n, idr_speed = self.idr.update(
            cal_imu, heading_deg, pitch_rad, is_stationary, is_outage
        )

        # Phase 5: GNSS + IDR Kalman Fusion
        # Predict at IMU rate
        self.ekf.predict(cal_imu, pitch_rad, is_stationary, dt)

        # Correct when valid GNSS arrives
        if frame.gnss.valid and not is_outage:
            self.ekf.update_gnss(frame.gnss, adaptive_scale=adaptive_scale)
            # Continually keep IDR baseline synchronized during long open sky periods
            self.idr.east = float(self.ekf.x[0, 0])
            self.idr.north = float(self.ekf.x[1, 0])
            self.idr.lat, self.idr.lon, _ = self.scenario_gen.ref_lat, self.scenario_gen.ref_lon, self.scenario_gen.ref_alt

        # Get final estimated fused state
        fused_lat, fused_lon, fused_e, fused_n, fused_speed, fused_heading = self.ekf.get_nav_state()

        # Build comprehensive output
        output = NavigationOutput(
            timestamp=now,
            mode=nav_mode,
            ai_condition=ai_cond,
            ai_confidence=ai_conf,
            est_lat=fused_lat,
            est_lon=fused_lon,
            est_alt=self.ref_alt,
            est_east=fused_e,
            est_north=fused_n,
            est_speed=fused_speed,
            est_heading=fused_heading,
            raw_lat=frame.gnss.lat if frame.gnss.valid else None,
            raw_lon=frame.gnss.lon if frame.gnss.valid else None,
            raw_speed=frame.gnss.speed if frame.gnss.valid else None,
            raw_heading=frame.gnss.heading if frame.gnss.valid else None,
            idr_lat=idr_lat,
            idr_lon=idr_lon,
            idr_east=idr_e,
            idr_north=idr_n,
            outage_duration=self.outage_detector.outage_duration,
            accumulated_drift=self.idr.accumulated_drift,
            zupt_active=is_stationary,
            gnss_hdop=frame.gnss.hdop,
            gnss_sats=frame.gnss.num_sats,
            latency_ms=self.ai_classifier.last_latency_ms
        )

        # Log to file
        if self.logger:
            self.logger.log(frame, output)

        # Notify subscribers
        for sub in self.output_subscribers:
            try:
                sub(frame, output)
            except Exception:
                pass

        return output

    def step(self) -> NavigationOutput:
        """Fetch next frame from active source and execute pipeline."""
        if self.use_mobile_source and self.mobile_receiver.is_connected():
            frame = self.mobile_receiver.last_frame or self.scenario_gen.step()
        else:
            frame = self.scenario_gen.step()

        return self.process_frame(frame)
