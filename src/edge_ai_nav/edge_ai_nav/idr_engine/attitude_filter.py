"""Attitude and Heading Filter for Intelligent Dead Reckoning."""

import math
from typing import Tuple, Optional
from ..sensor_layer.data_types import IMUData


class AttitudeFilter:
    """Tracks vehicle 3D orientation (Roll, Pitch, Yaw / Heading) with complementary filtering."""

    def __init__(self, initial_heading_deg: float = 0.0, alpha_tilt: float = 0.05):
        # Heading in radians: 0 = North, pi/2 = East, pi = South, 3pi/2 = West
        self.heading_rad = math.radians(initial_heading_deg) % (2 * math.pi)
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.alpha_tilt = alpha_tilt
        self.last_timestamp: Optional[float] = None

    def update(self,
               calibrated_imu: IMUData,
               tilt_angles: Tuple[float, float],
               gnss_heading_deg: Optional[float] = None,
               gnss_valid: bool = False,
               speed: float = 0.0) -> Tuple[float, float, float]:
        """Update orientation state.
        
        Returns:
            (roll_deg, pitch_deg, heading_deg)
        """
        now = calibrated_imu.timestamp
        if self.last_timestamp is None:
            self.last_timestamp = now
            dt = 0.02
        else:
            dt = max(0.001, min(0.1, now - self.last_timestamp))
            self.last_timestamp = now

        meas_roll, meas_pitch = tilt_angles
        gz = calibrated_imu.gz  # yaw rate around vehicle body Z

        # 1. Update Pitch & Roll with complementary filter
        self.roll_rad = (1 - self.alpha_tilt) * self.roll_rad + self.alpha_tilt * meas_roll
        self.pitch_rad = (1 - self.alpha_tilt) * self.pitch_rad + self.alpha_tilt * meas_pitch

        # 2. Integrate Yaw Rate
        self.heading_rad = (self.heading_rad + gz * dt) % (2 * math.pi)

        # 3. Heading alignment with GNSS when moving at speed (> 1.5 m/s) in open sky
        if gnss_valid and gnss_heading_deg is not None and speed > 1.5:
            target_rad = math.radians(gnss_heading_deg) % (2 * math.pi)
            # Shortest angular difference
            diff = (target_rad - self.heading_rad + math.pi) % (2 * math.pi) - math.pi
            # Subtle nudging (alpha = 0.03) to prevent yaw drift during good GNSS
            self.heading_rad = (self.heading_rad + 0.03 * diff) % (2 * math.pi)

        heading_deg = math.degrees(self.heading_rad) % 360.0
        roll_deg = math.degrees(self.roll_rad)
        pitch_deg = math.degrees(self.pitch_rad)

        return roll_deg, pitch_deg, heading_deg
