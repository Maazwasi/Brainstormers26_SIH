"""Extended Kalman Filter (EKF) and Complementary Fusion Engine."""

import math
import numpy as np
from typing import Tuple, Optional
from ..sensor_layer.data_types import GNSSData, IMUData, NavigationOutput
from ..sensor_layer.scenario_generator import enu_to_wgs84, wgs84_to_enu


class EKFNavFusion:
    """Fuses IMU dead reckoning with GNSS measurements via an adaptive Kalman Filter."""

    def __init__(self,
                 ref_lat: float = 28.6139,
                 ref_lon: float = 77.2090,
                 ref_alt: float = 216.0):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

        # State Vector x: [pos_east, pos_north, vel_east, vel_north, heading_rad]
        self.x = np.zeros((5, 1), dtype=np.float64)

        # State Error Covariance Matrix P (5x5)
        self.P = np.diag([1.0, 1.0, 0.5, 0.5, 0.05])

        # Base Process Noise Q (5x5)
        self.Q_base = np.diag([0.05, 0.05, 0.15, 0.15, 0.01])
        self.Q = self.Q_base.copy()

        # Measurement Noise R (5x5)
        self.R_base = np.diag([1.5, 1.5, 0.2, 0.2, 0.08])
        self.R = self.R_base.copy()

        # Observation Matrix H
        self.H = np.eye(5, dtype=np.float64)

        self.last_timestamp: Optional[float] = None
        self.is_initialized = False

    def initialize(self, lat: float, lon: float, alt: float, heading_deg: float = 0.0) -> None:
        """Initialize filter state at specified coordinates."""
        e, n = wgs84_to_enu(lat, lon, self.ref_lat, self.ref_lon)
        psi = math.radians(heading_deg) % (2 * math.pi)
        self.x = np.array([[e], [n], [0.0], [0.0], [psi]], dtype=np.float64)
        self.is_initialized = True

    def predict(self, calibrated_imu: IMUData, pitch_rad: float, is_stationary: bool, dt: float) -> None:
        """EKF Prediction Step driven by high-rate IMU."""
        if not self.is_initialized:
            return

        dt = max(0.001, min(0.1, dt))
        psi = float(self.x[4, 0])

        if is_stationary:
            # Standstill: enforce zero velocity
            self.x[2, 0] = 0.0
            self.x[3, 0] = 0.0
            # Low process noise when stationary
            self.P += self.Q * 0.1 * dt
            return

        # Acceleration and angular rate inputs
        a_fwd = calibrated_imu.ax * math.cos(pitch_rad)
        if abs(a_fwd) < 0.08:
            a_fwd = 0.0
        omega_z = calibrated_imu.gz

        # Kinematic state transition
        # x_new = x + v_E * dt
        # y_new = y + v_N * dt
        # v_E_new = v_E + a_fwd * sin(psi) * dt
        # v_N_new = v_N + a_fwd * cos(psi) * dt
        # psi_new = psi + omega_z * dt
        e = self.x[0, 0] + self.x[2, 0] * dt
        n = self.x[1, 0] + self.x[3, 0] * dt
        ve = self.x[2, 0] + a_fwd * math.sin(psi) * dt
        vn = self.x[3, 0] + a_fwd * math.cos(psi) * dt
        psi_new = (psi + omega_z * dt) % (2 * math.pi)

        self.x = np.array([[e], [n], [ve], [vn], [psi_new]], dtype=np.float64)

        # State Transition Jacobian Matrix F (5x5)
        F = np.eye(5, dtype=np.float64)
        F[0, 2] = dt
        F[1, 3] = dt
        F[2, 4] = a_fwd * math.cos(psi) * dt
        F[3, 4] = -a_fwd * math.sin(psi) * dt

        # Propagate covariance: P = F * P * F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_gnss(self, gnss: GNSSData, adaptive_scale: float = 1.0) -> bool:
        """EKF Correction Step when valid GNSS measurement arrives."""
        if not gnss.valid or not self.is_initialized:
            return False

        # Convert GNSS lat/lon to ENU
        z_e, z_n = wgs84_to_enu(gnss.lat, gnss.lon, self.ref_lat, self.ref_lon)
        z_psi = math.radians(gnss.heading) % (2 * math.pi)
        z_ve = gnss.speed * math.sin(z_psi)
        z_vn = gnss.speed * math.cos(z_psi)

        z = np.array([[z_e], [z_n], [z_ve], [z_vn], [z_psi]], dtype=np.float64)

        # Scale measurement noise R by GNSS HDOP and adaptive scale
        hdop_factor = max(0.5, (gnss.hdop / 1.0) ** 2) * adaptive_scale
        R_scaled = self.R_base * hdop_factor

        # Innovation (Residual) y = z - H * x
        y = z - (self.H @ self.x)
        
        # Wrap heading error to [-pi, pi]
        y[4, 0] = (y[4, 0] + math.pi) % (2 * math.pi) - math.pi

        # Innovation Covariance S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + R_scaled

        # Innovation Gating (Mahalanobis Distance)
        try:
            S_inv = np.linalg.inv(S)
            d_mahalanobis = float((y.T @ S_inv @ y)[0, 0])
            
            # Reject anomalous spikes
            if d_mahalanobis > 45.0:
                return False

            # Kalman Gain K = P * H^T * S_inv
            K = self.P @ self.H.T @ S_inv

            # State Update: x = x + K * y
            self.x = self.x + K @ y
            self.x[4, 0] = self.x[4, 0] % (2 * math.pi)

            # Joseph-form Covariance Update for numerical stability: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
            I_KH = np.eye(5) - K @ self.H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_scaled @ K.T
            return True
        except np.linalg.LinAlgError:
            return False

    def get_nav_state(self) -> Tuple[float, float, float, float, float, float]:
        """Return (est_lat, est_lon, est_east, est_north, est_speed, est_heading_deg)."""
        e = float(self.x[0, 0])
        n = float(self.x[1, 0])
        ve = float(self.x[2, 0])
        vn = float(self.x[3, 0])
        psi = float(self.x[4, 0])

        speed = math.sqrt(ve * ve + vn * vn)
        heading_deg = math.degrees(psi) % 360.0
        lat, lon, _ = enu_to_wgs84(e, n, self.ref_lat, self.ref_lon, self.ref_alt)

        return lat, lon, e, n, speed, heading_deg
