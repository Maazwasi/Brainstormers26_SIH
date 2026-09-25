"""Intelligent Dead Reckoning (IDR) Engine."""

import math
from typing import Tuple, Optional
from ..sensor_layer.data_types import IMUData, GNSSData
from ..sensor_layer.scenario_generator import enu_to_wgs84, wgs84_to_enu


class IntelligentDeadReckoner:
    """Propagates vehicle state during GNSS outages using IMU, ZUPT, and non-holonomic constraints."""

    def __init__(self,
                 ref_lat: float = 28.6139,
                 ref_lon: float = 77.2090,
                 ref_alt: float = 216.0,
                 max_speed: float = 30.0,
                 velocity_damping: float = 0.999):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt
        self.max_speed = max_speed
        self.velocity_damping = velocity_damping

        # State Variables
        self.east = 0.0
        self.north = 0.0
        self.speed = 0.0
        self.vel_east = 0.0
        self.vel_north = 0.0
        self.heading_rad = 0.0
        
        # WGS-84 Coordinates
        self.lat = ref_lat
        self.lon = ref_lon
        self.alt = ref_alt

        # Outage and Drift Tracking
        self.last_timestamp: Optional[float] = None
        self.outage_time: float = 0.0
        self.accumulated_drift: float = 0.0

    def sync_with_gnss(self, gnss: GNSSData) -> None:
        """Synchronize internal dead reckoned state with fresh valid GNSS fix."""
        if not gnss.valid:
            return
        
        # Convert GNSS lat/lon to local ENU
        e, n = wgs84_to_enu(gnss.lat, gnss.lon, self.ref_lat, self.ref_lon)
        self.east = e
        self.north = n
        self.lat = gnss.lat
        self.lon = gnss.lon
        self.alt = gnss.alt
        self.speed = gnss.speed
        self.heading_rad = math.radians(gnss.heading) % (2 * math.pi)
        
        self.vel_east = self.speed * math.sin(self.heading_rad)
        self.vel_north = self.speed * math.cos(self.heading_rad)
        
        # Reset drift upon good GNSS lock
        self.outage_time = 0.0
        self.accumulated_drift = 0.0

    def update(self,
               calibrated_imu: IMUData,
               heading_deg: float,
               pitch_rad: float,
               is_stationary: bool,
               is_outage: bool) -> Tuple[float, float, float, float, float]:
        """Propagate position and velocity.
        
        Returns:
            (est_lat, est_lon, est_east, est_north, est_speed)
        """
        now = calibrated_imu.timestamp
        if self.last_timestamp is None:
            self.last_timestamp = now
            dt = 0.02
        else:
            dt = max(0.001, min(0.1, now - self.last_timestamp))
            self.last_timestamp = now

        self.heading_rad = math.radians(heading_deg) % (2 * math.pi)

        if is_stationary:
            # Zero-Velocity Update (ZUPT): enforce complete standstill
            self.speed = 0.0
            self.vel_east = 0.0
            self.vel_north = 0.0
        else:
            # Vehicle forward acceleration along body X axis
            # Correct for pitch tilt: a_forward = ax * cos(pitch) - (az - 9.81) * sin(pitch)
            a_forward = calibrated_imu.ax * math.cos(pitch_rad)
            
            # Apply small acceleration deadband to prevent drift from minor sensor chatter (< 0.1 m/s^2)
            if abs(a_forward) < 0.08:
                a_forward = 0.0

            # Forward velocity integration with slight damping factor
            self.speed = (self.speed + a_forward * dt) * self.velocity_damping
            self.speed = max(0.0, min(self.max_speed, self.speed))

            # Non-holonomic ground vehicle constraint: lateral velocity is 0
            self.vel_east = self.speed * math.sin(self.heading_rad)
            self.vel_north = self.speed * math.cos(self.heading_rad)

            # Propagate local ENU position
            self.east += self.vel_east * dt
            self.north += self.vel_north * dt

        # Update WGS-84 Geodetic Coordinates
        self.lat, self.lon, self.alt = enu_to_wgs84(self.east, self.north, self.ref_lat, self.ref_lon, self.ref_alt)

        # Track Outage Drift Radius Estimate: r = 0.5 * sigma_a * t^2 + sigma_v * t
        if is_outage:
            self.outage_time += dt
            self.accumulated_drift = 0.04 * (self.outage_time ** 1.6) + 0.1 * self.outage_time
        else:
            self.outage_time = 0.0
            self.accumulated_drift = 0.0

        return self.lat, self.lon, self.east, self.north, self.speed
