"""Standardized data structures for Edge AI + GNSS/IDR prototype."""

from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any
import time


@dataclass
class GNSSData:
    """Standardized GNSS measurement frame."""
    lat: float = 0.0          # Latitude in degrees
    lon: float = 0.0          # Longitude in degrees
    alt: float = 0.0          # Altitude in meters
    speed: float = 0.0        # Horizontal speed in m/s
    heading: float = 0.0      # Heading in degrees (0 = North, 90 = East)
    num_sats: int = 0         # Visible/tracked satellites
    hdop: float = 99.9        # Horizontal Dilution of Precision
    fix_type: int = 0         # 0=Invalid, 1=Autonomous GPS, 2=DGPS, 3=RTK Fix
    valid: bool = False       # Hardware/sensor validity flag
    timestamp: float = 0.0    # Epoch seconds


@dataclass
class IMUData:
    """Standardized 6-DOF IMU measurement frame."""
    ax: float = 0.0           # Body-frame acceleration X (Forward) in m/s^2
    ay: float = 0.0           # Body-frame acceleration Y (Left) in m/s^2
    az: float = 9.81          # Body-frame acceleration Z (Up) in m/s^2
    gx: float = 0.0           # Body-frame angular velocity X (Roll rate) in rad/s
    gy: float = 0.0           # Body-frame angular velocity Y (Pitch rate) in rad/s
    gz: float = 0.0           # Body-frame angular velocity Z (Yaw rate) in rad/s
    timestamp: float = 0.0    # Epoch seconds


@dataclass
class SensorFrame:
    """Unified synchronized sensor frame holding GNSS, IMU and status."""
    timestamp: float = field(default_factory=time.time)
    frame_id: int = 0
    gnss: GNSSData = field(default_factory=GNSSData)
    imu: IMUData = field(default_factory=IMUData)
    is_simulated_outage: bool = False
    source: str = "synthetic"  # 'synthetic', 'mobile_stream', 'ros_topic', 'file_replay'

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "source": self.source,
            "is_simulated_outage": self.is_simulated_outage,
            "gnss": asdict(self.gnss),
            "imu": asdict(self.imu),
        }


@dataclass
class NavigationOutput:
    """Full navigation pipeline output for telemetry, UI, and ROS 2."""
    timestamp: float = 0.0
    mode: str = "GNSS_FIX"       # 'GNSS_FIX', 'GNSS_DEGRADED', 'IDR_ACTIVE', 'FUSION_CONVERGING'
    ai_condition: str = "OPEN_SKY" # 'OPEN_SKY', 'URBAN_CANYON', 'TUNNEL_OUTAGE', 'JAMMING_SPOOFING'
    ai_confidence: float = 1.0

    # Estimated Position (WGS-84 & Local ENU)
    est_lat: float = 0.0
    est_lon: float = 0.0
    est_alt: float = 0.0
    est_east: float = 0.0        # Local meters East
    est_north: float = 0.0       # Local meters North
    est_speed: float = 0.0       # m/s
    est_heading: float = 0.0     # degrees (0 = North, 90 = East)

    # Raw GNSS Position (may be null/invalid during outage)
    raw_lat: Optional[float] = None
    raw_lon: Optional[float] = None
    raw_speed: Optional[float] = None
    raw_heading: Optional[float] = None

    # Dead Reckoned Position (always continuous)
    idr_lat: float = 0.0
    idr_lon: float = 0.0
    idr_east: float = 0.0
    idr_north: float = 0.0

    # Diagnostics & Health
    outage_duration: float = 0.0 # Seconds currently in outage
    accumulated_drift: float = 0.0 # Estimated drift radius in meters
    zupt_active: bool = False
    gnss_hdop: float = 1.0
    gnss_sats: int = 10
    latency_ms: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
