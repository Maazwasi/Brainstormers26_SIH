"""Rolling feature extraction window for Edge AI condition classifier."""

import math
import numpy as np
from collections import deque
from typing import List, Optional
from ..sensor_layer.data_types import SensorFrame


class NavigationFeatureExtractor:
    """Computes dynamic statistical features over a sliding temporal window of sensor frames."""

    def __init__(self, window_size: int = 15):
        self.window_size = window_size
        self.frames = deque(maxlen=window_size)

    def push_and_extract(self, frame: SensorFrame, est_speed: float = 0.0) -> np.ndarray:
        """Add latest SensorFrame and extract 8 normalized features for the Edge AI model.
        
        Features:
        [0]: accel_variance (vibration / road dynamics)
        [1]: gyro_variance (angular rate fluctuation)
        [2]: hdop_mean (satellite geometry quality)
        [3]: hdop_rate (speed of degradation)
        [4]: sat_count_normalized (sats / 15.0)
        [5]: gnss_valid_ratio (packet health fraction: 0.0 to 1.0)
        [6]: speed_discrepancy (difference between IMU integrated speed and GNSS speed)
        [7]: is_outage_flag (direct outage indicator: 1.0 or 0.0)
        """
        self.frames.append((frame, est_speed))

        if len(self.frames) < 3:
            # Return baseline nominal open-sky vector if buffer is filling
            return np.array([0.05, 0.01, 0.9, 0.0, 0.7, 1.0, 0.0, 0.0], dtype=np.float32)

        # Extract buffer arrays
        accel_mags = []
        gyro_norms = []
        hdops = []
        sats = []
        valids = []
        speed_diffs = []

        for f, v_est in self.frames:
            # Accel mag
            a_mag = math.sqrt(f.imu.ax**2 + f.imu.ay**2 + f.imu.az**2)
            accel_mags.append(a_mag)
            
            # Gyro norm
            g_norm = math.sqrt(f.imu.gx**2 + f.imu.gy**2 + f.imu.gz**2)
            gyro_norms.append(g_norm)

            # GNSS stats
            hdops.append(min(15.0, f.gnss.hdop))
            sats.append(f.gnss.num_sats)
            valids.append(1.0 if f.gnss.valid else 0.0)

            if f.gnss.valid:
                speed_diffs.append(abs(v_est - f.gnss.speed))
            else:
                speed_diffs.append(0.5)

        accel_var = float(np.var(accel_mags))
        gyro_var = float(np.var(gyro_norms))
        hdop_mean = float(np.mean(hdops))
        hdop_rate = float(hdops[-1] - hdops[0])
        sat_norm = float(np.mean(sats)) / 12.0
        gnss_valid_ratio = float(np.mean(valids))
        speed_discrepancy = float(np.mean(speed_diffs))
        is_outage = 1.0 if (not frame.gnss.valid or frame.is_simulated_outage) else 0.0

        feature_vector = np.array([
            min(5.0, accel_var),
            min(2.0, gyro_var),
            min(15.0, hdop_mean),
            min(10.0, max(-10.0, hdop_rate)),
            min(1.5, sat_norm),
            gnss_valid_ratio,
            min(10.0, speed_discrepancy),
            is_outage
        ], dtype=np.float32)

        return feature_vector
