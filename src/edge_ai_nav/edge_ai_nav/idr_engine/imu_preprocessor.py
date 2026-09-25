"""IMU Preprocessing: ZUPT Stationary Detection, Bias Calibration and Gravity Compensation."""

import math
from typing import Tuple
import numpy as np
from collections import deque
from ..sensor_layer.data_types import IMUData


class IMUPreprocessor:
    """Detects stationary states (ZUPT), removes gravity, and tracks gyroscope bias."""

    def __init__(self,
                 window_size: int = 15,
                 accel_var_threshold: float = 0.05,
                 gyro_norm_threshold: float = 0.04,
                 gravity_nominal: float = 9.80665):
        self.window_size = window_size
        self.accel_var_threshold = accel_var_threshold
        self.gyro_norm_threshold = gyro_norm_threshold
        self.gravity_nominal = gravity_nominal

        # Sliding buffers for variance computation
        self.accel_mag_buffer = deque(maxlen=window_size)
        self.gyro_norm_buffer = deque(maxlen=window_size)

        # Gyroscope Bias Estimator (low-pass filter updated during ZUPT)
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.is_stationary = False
        self.bias_alpha = 0.02  # Slow learning rate for stationary bias tracking

    def process(self, imu: IMUData) -> Tuple[bool, IMUData, Tuple[float, float, float]]:
        """Preprocess IMU sample.
        
        Returns:
            is_stationary (bool): True if vehicle is stopped (ZUPT condition)
            calibrated_imu (IMUData): IMU with gyro biases subtracted
            tilt_angles (roll, pitch): Tilt estimation in radians
        """
        ax, ay, az = imu.ax, imu.ay, imu.az
        gx, gy, gz = imu.gx, imu.gy, imu.gz

        # 1. Magnitudes
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)

        self.accel_mag_buffer.append(accel_mag)
        self.gyro_norm_buffer.append(gyro_norm)

        # 2. Zero-Velocity Detection (ZUPT)
        if len(self.accel_mag_buffer) >= self.window_size:
            accel_var = float(np.var(self.accel_mag_buffer))
            accel_mean = float(np.mean(self.accel_mag_buffer))
            gyro_mean = float(np.mean(self.gyro_norm_buffer))

            # Stationary criteria: accel mag close to 1g, low accel variance, low gyro rate
            near_1g = abs(accel_mean - self.gravity_nominal) < 0.6
            low_accel_var = accel_var < self.accel_var_threshold
            low_gyro = gyro_mean < self.gyro_norm_threshold

            self.is_stationary = bool(near_1g and low_accel_var and low_gyro)
        else:
            self.is_stationary = False

        # 3. Dynamic Bias Calibration when Stationary
        if self.is_stationary:
            self.gyro_bias_x = (1 - self.bias_alpha) * self.gyro_bias_x + self.bias_alpha * gx
            self.gyro_bias_y = (1 - self.bias_alpha) * self.gyro_bias_y + self.bias_alpha * gy
            self.gyro_bias_z = (1 - self.bias_alpha) * self.gyro_bias_z + self.bias_alpha * gz

        # Subtract gyro biases
        calibrated_gx = gx - self.gyro_bias_x
        calibrated_gy = gy - self.gyro_bias_y
        calibrated_gz = gz - self.gyro_bias_z

        # 4. Tilt Estimation from Gravity Vector
        # Roll (phi) around X, Pitch (theta) around Y
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll = math.atan2(ay, az)

        calibrated_imu = IMUData(
            ax=ax,
            ay=ay,
            az=az,
            gx=calibrated_gx,
            gy=calibrated_gy,
            gz=calibrated_gz,
            timestamp=imu.timestamp
        )

        return self.is_stationary, calibrated_imu, (roll, pitch)
