"""Realistic scenario generator for Edge AI + GNSS/IDR prototype testing."""

import math
import time
import numpy as np
from typing import Generator, Tuple, Optional
from .data_types import SensorFrame, GNSSData, IMUData

# Earth radius in meters (WGS-84 approximate spherical)
EARTH_RADIUS = 6378137.0


def enu_to_wgs84(east: float, north: float, ref_lat: float, ref_lon: float, ref_alt: float = 216.0) -> Tuple[float, float, float]:
    """Convert local ENU Cartesian coordinates (meters) to WGS-84 (lat, lon, alt)."""
    d_lat = (north / EARTH_RADIUS) * (180.0 / math.pi)
    d_lon = (east / (EARTH_RADIUS * math.cos(math.radians(ref_lat)))) * (180.0 / math.pi)
    return ref_lat + d_lat, ref_lon + d_lon, ref_alt


def wgs84_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert WGS-84 (lat, lon) to local ENU Cartesian coordinates (meters)."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north = d_lat * EARTH_RADIUS
    east = d_lon * EARTH_RADIUS * math.cos(math.radians(ref_lat))
    return east, north


class ScenarioGenerator:
    """Generates continuous realistic sensor frames with programmed or interactive GNSS outages."""

    def __init__(self,
                 scenario_name: str = "sih_demo",
                 rate_hz: float = 50.0,
                 ref_lat: float = 28.6139,
                 ref_lon: float = 77.2090,
                 ref_alt: float = 216.0):
        self.scenario_name = scenario_name
        self.dt = 1.0 / rate_hz
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt
        
        # Vehicle ground truth state
        self.t = 0.0
        self.frame_count = 0
        self.east = 0.0
        self.north = 0.0
        self.speed = 0.0
        self.heading_rad = 0.0  # 0 = North, pi/2 = East
        
        # Manual outage override toggle
        self.manual_outage_override: Optional[bool] = None

        # Noise characteristics (simulating low-cost MEMS IMU + commercial GNSS)
        self.accel_noise_std = 0.08      # m/s^2
        self.gyro_noise_std = 0.005       # rad/s (~0.3 deg/s)
        self.gyro_bias_z = 0.008          # rad/s constant uncalibrated bias
        self.gps_pos_noise_std = 1.2      # meters horizontal
        self.gps_speed_noise_std = 0.15   # m/s

    def toggle_manual_outage(self, force_outage: Optional[bool] = None) -> bool:
        """Toggle or explicitly set manual outage override."""
        if force_outage is None:
            if self.manual_outage_override is None:
                self.manual_outage_override = True
            else:
                self.manual_outage_override = not self.manual_outage_override
        else:
            self.manual_outage_override = force_outage
        return bool(self.manual_outage_override)

    def step(self) -> SensorFrame:
        """Advance simulation by dt and generate next synchronized SensorFrame."""
        t = self.t
        self.frame_count += 1
        
        # Determine kinematics based on scenario progression
        # Total cycle: 55 seconds, then repeats or runs continuously
        cycle_t = t % 55.0
        
        # 1. Kinematic Profile
        if cycle_t < 4.0:
            # Stationary / Engine Start (ZUPT phase)
            target_accel = 0.0
            yaw_rate = 0.0
            self.speed = 0.0
            phase_type = "OPEN_SKY"
            natural_outage = False
            sats = 11
            hdop = 0.8
        elif cycle_t < 10.0:
            # Acceleration to 8.0 m/s heading North-East (45 deg)
            target_accel = 1.33
            self.speed = min(8.0, self.speed + target_accel * self.dt)
            yaw_rate = 0.12 # turning towards 45 degrees
            phase_type = "OPEN_SKY"
            natural_outage = False
            sats = 10
            hdop = 0.9
        elif cycle_t < 15.0:
            # Cruise, Approaching Urban Canyon / Overpass
            target_accel = 0.0
            yaw_rate = 0.0
            phase_type = "URBAN_CANYON"
            natural_outage = False
            sats = 5
            hdop = 2.8
        elif cycle_t < 35.0:
            # TUNNEL OUTAGE (20 seconds duration under tunnel!)
            # Vehicle does an S-curve turn inside the tunnel
            phase_type = "TUNNEL_OUTAGE"
            natural_outage = True
            sats = 0
            hdop = 99.0
            target_accel = -0.05  # slight coasting deceleration
            self.speed = max(5.0, self.speed + target_accel * self.dt)
            if cycle_t < 25.0:
                yaw_rate = -0.18 # Curve left (-10 deg/s)
            else:
                yaw_rate = 0.18  # Curve right (+10 deg/s)
        elif cycle_t < 45.0:
            # Tunnel Exit / GNSS Recovery
            phase_type = "RECOVERY"
            natural_outage = False
            target_accel = 0.0
            yaw_rate = 0.0
            # Sats quickly re-acquire
            sats = 8 if cycle_t < 38.0 else 11
            hdop = 1.8 if cycle_t < 38.0 else 0.8
        else:
            # Decelerate to stop
            phase_type = "BRAKING"
            natural_outage = False
            target_accel = -1.0
            self.speed = max(0.0, self.speed + target_accel * self.dt)
            yaw_rate = 0.0
            sats = 11
            hdop = 0.8

        # Outage condition logic: check manual override or natural scenario
        is_outage = natural_outage if self.manual_outage_override is None else self.manual_outage_override
        if self.manual_outage_override is True:
            sats = 0
            hdop = 99.0
        elif self.manual_outage_override is False and is_outage:
            # Force restored
            sats = 10
            hdop = 0.9

        # Update vehicle pose ground truth
        self.heading_rad = (self.heading_rad + yaw_rate * self.dt) % (2 * math.pi)
        vx = self.speed * math.sin(self.heading_rad) # East velocity
        vy = self.speed * math.cos(self.heading_rad) # North velocity
        self.east += vx * self.dt
        self.north += vy * self.dt
        
        gt_lat, gt_lon, gt_alt = enu_to_wgs84(self.east, self.north, self.ref_lat, self.ref_lon, self.ref_alt)

        # 2. Generate Synthetic IMU Data (with physics: centripetal accel + road noise + gravity)
        # Body frame: X forward, Y left, Z up
        centripetal_ay = self.speed * yaw_rate
        ax_body = target_accel + np.random.normal(0, self.accel_noise_std)
        ay_body = centripetal_ay + np.random.normal(0, self.accel_noise_std)
        az_body = 9.81 + np.random.normal(0, self.accel_noise_std * 1.5) # gravity + bump noise
        
        gx_body = np.random.normal(0, self.gyro_noise_std) # Roll
        gy_body = np.random.normal(0, self.gyro_noise_std) # Pitch
        gz_body = yaw_rate + self.gyro_bias_z + np.random.normal(0, self.gyro_noise_std) # Yaw rate

        imu_data = IMUData(
            ax=float(ax_body),
            ay=float(ay_body),
            az=float(az_body),
            gx=float(gx_body),
            gy=float(gy_body),
            gz=float(gz_body),
            timestamp=time.time()
        )

        # 3. Generate Synthetic GNSS Data (GNSS updates typically at 1-5Hz, but frame published at IMU rate)
        # We simulate GNSS fix validity according to outage status
        heading_deg = math.degrees(self.heading_rad) % 360.0
        
        if is_outage:
            gnss_data = GNSSData(
                lat=0.0,
                lon=0.0,
                alt=0.0,
                speed=0.0,
                heading=0.0,
                num_sats=0,
                hdop=99.9,
                fix_type=0,
                valid=False,
                timestamp=time.time()
            )
        else:
            # Add multipath / HDOP proportional position noise
            pos_noise_e = np.random.normal(0, self.gps_pos_noise_std * (hdop / 0.8))
            pos_noise_n = np.random.normal(0, self.gps_pos_noise_std * (hdop / 0.8))
            noisy_lat, noisy_lon, _ = enu_to_wgs84(self.east + pos_noise_e, self.north + pos_noise_n, self.ref_lat, self.ref_lon)
            noisy_speed = max(0.0, self.speed + np.random.normal(0, self.gps_speed_noise_std))
            noisy_heading = (heading_deg + np.random.normal(0, 1.5)) % 360.0

            gnss_data = GNSSData(
                lat=float(noisy_lat),
                lon=float(noisy_lon),
                alt=self.ref_alt + np.random.normal(0, 0.5),
                speed=float(noisy_speed),
                heading=float(noisy_heading),
                num_sats=int(sats),
                hdop=float(hdop),
                fix_type=1 if sats >= 4 else 0,
                valid=True if sats >= 4 else False,
                timestamp=time.time()
            )

        self.t += self.dt
        return SensorFrame(
            timestamp=time.time(),
            frame_id=self.frame_count,
            gnss=gnss_data,
            imu=imu_data,
            is_simulated_outage=is_outage,
            source="scenario_generator"
        )
