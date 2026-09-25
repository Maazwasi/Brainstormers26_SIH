"""Mobile device sensor ingestion handler for phone-based testing."""

import time
import math
from typing import Optional, Dict, Any
from .data_types import SensorFrame, GNSSData, IMUData


class MobileStreamReceiver:
    """Receives, validates and converts raw smartphone sensor frames into unified SensorFrame."""

    def __init__(self):
        self.last_frame: Optional[SensorFrame] = None
        self.last_received_time: float = 0.0
        self.frame_counter: int = 0
        self.active: bool = False

    def ingest(self, payload: Dict[str, Any]) -> SensorFrame:
        """Parse incoming JSON payload from mobile phone web client or sensor logging app.
        
        Supported formats:
        - Web Client: {lat, lon, speed, heading, ax, ay, az, gx, gy, gz, accuracy}
        - Sensor Logger App format
        """
        now = time.time()
        self.frame_counter += 1
        self.last_received_time = now
        self.active = True

        # Extract GNSS
        lat = float(payload.get("lat", payload.get("latitude", 0.0)))
        lon = float(payload.get("lon", payload.get("longitude", 0.0)))
        alt = float(payload.get("alt", payload.get("altitude", 0.0)))
        speed = float(payload.get("speed", 0.0))
        heading = float(payload.get("heading", 0.0))
        accuracy = float(payload.get("accuracy", 5.0)) # meters
        
        # Approximate HDOP from horizontal accuracy (accuracy / 5.0)
        hdop = max(0.5, accuracy / 5.0)
        valid_gnss = (lat != 0.0 and lon != 0.0 and accuracy < 50.0)

        gnss = GNSSData(
            lat=lat,
            lon=lon,
            alt=alt,
            speed=speed,
            heading=heading,
            num_sats=payload.get("sats", 8 if valid_gnss else 0),
            hdop=hdop,
            fix_type=1 if valid_gnss else 0,
            valid=valid_gnss,
            timestamp=now
        )

        # Extract IMU
        # Phone coordinate system: X right, Y forward/up, Z outward
        # We map to Robot/Vehicle Body Frame: X Forward, Y Left, Z Up
        phone_ax = float(payload.get("ax", payload.get("accel_x", 0.0)))
        phone_ay = float(payload.get("ay", payload.get("accel_y", 0.0)))
        phone_az = float(payload.get("az", payload.get("accel_z", 9.81)))

        phone_gx = float(payload.get("gx", payload.get("gyro_x", 0.0)))
        phone_gy = float(payload.get("gy", payload.get("gyro_y", 0.0)))
        phone_gz = float(payload.get("gz", payload.get("gyro_z", 0.0)))

        # Convert phone deg/s to rad/s if necessary
        if abs(phone_gz) > 10.0:  # Likely deg/s
            phone_gx = math.radians(phone_gx)
            phone_gy = math.radians(phone_gy)
            phone_gz = math.radians(phone_gz)

        imu = IMUData(
            ax=phone_ay,   # Phone Y is forward
            ay=-phone_ax,  # Phone -X is left
            az=phone_az,
            gx=phone_gy,
            gy=-phone_gx,
            gz=phone_gz,
            timestamp=now
        )

        frame = SensorFrame(
            timestamp=now,
            frame_id=self.frame_counter,
            gnss=gnss,
            imu=imu,
            is_simulated_outage=not valid_gnss,
            source="mobile_phone"
        )
        self.last_frame = frame
        return frame

    def is_connected(self, timeout_sec: float = 2.0) -> bool:
        """Check if mobile device is actively streaming."""
        return self.active and ((time.time() - self.last_received_time) < timeout_sec)
