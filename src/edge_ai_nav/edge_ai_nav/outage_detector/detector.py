"""GNSS Outage Detection Engine for Edge AI Navigation."""

import time
import math
from typing import Tuple, Optional
from ..sensor_layer.data_types import SensorFrame, GNSSData


class GNSSOutageDetector:
    """Monitors GNSS signal integrity across multiple criteria and triggers mode transitions."""

    # Operating Modes
    MODE_GNSS_FIX = "GNSS_FIX"
    MODE_GNSS_DEGRADED = "GNSS_DEGRADED"
    MODE_IDR_ACTIVE = "IDR_ACTIVE"
    MODE_FUSION_CONVERGING = "FUSION_CONVERGING"

    def __init__(self,
                 timeout_threshold_sec: float = 1.2,
                 min_satellites: int = 4,
                 max_hdop_outage: float = 3.5,
                 max_hdop_degraded: float = 2.2,
                 recovery_confirmation_count: int = 3):
        self.timeout_threshold_sec = timeout_threshold_sec
        self.min_satellites = min_satellites
        self.max_hdop_outage = max_hdop_outage
        self.max_hdop_degraded = max_hdop_degraded
        self.recovery_confirmation_count = recovery_confirmation_count

        # Internal State Machine
        self.current_mode = self.MODE_GNSS_FIX
        self.last_valid_gnss_time: float = time.time()
        self.last_valid_lat: Optional[float] = None
        self.last_valid_lon: Optional[float] = None

        # Statistics & Timers
        self.outage_start_time: Optional[float] = None
        self.outage_duration: float = 0.0
        self.total_outages_count: int = 0
        self.recovery_counter: int = 0
        self.transition_reason: str = "System Initialized"

    def update(self, frame: SensorFrame, ai_condition: str = "OPEN_SKY") -> Tuple[str, str]:
        """Evaluate sensor frame and return (new_mode, transition_reason)."""
        gnss = frame.gnss
        now = frame.timestamp
        dt_since_last_gnss = now - self.last_valid_gnss_time

        # Outage trigger tests
        is_hardware_invalid = (not gnss.valid) or (gnss.fix_type == 0)
        is_sat_low = gnss.num_sats < self.min_satellites
        is_hdop_bad = gnss.hdop >= self.max_hdop_outage
        # A currently valid receiver fix must clear a startup heartbeat timeout.
        # Freshness is represented by frame.timestamp in this MVP; latching a
        # timeout while valid data is present would make recovery impossible.
        is_timed_out = (dt_since_last_gnss > self.timeout_threshold_sec) and not gnss.valid
        is_forced = frame.is_simulated_outage
        is_ai_tunnel = (ai_condition == "TUNNEL_OUTAGE")

        is_outage = is_hardware_invalid or is_sat_low or is_hdop_bad or is_timed_out or is_forced or is_ai_tunnel

        # State Machine Transitions
        if is_outage:
            self.recovery_counter = 0
            if self.current_mode in (self.MODE_GNSS_FIX, self.MODE_GNSS_DEGRADED):
                # Transition GNSS -> IDR
                self.current_mode = self.MODE_IDR_ACTIVE
                self.outage_start_time = now
                self.total_outages_count += 1
                
                # Determine primary trigger reason
                if is_forced:
                    self.transition_reason = "Outage Injected / Simulated"
                elif is_ai_tunnel:
                    self.transition_reason = "AI Alert: Tunnel Environment Detected"
                elif is_hardware_invalid:
                    self.transition_reason = "GNSS Fix Invalid Flag (0 sats)"
                elif is_sat_low:
                    self.transition_reason = f"Satellites Dropped Below Threshold ({gnss.num_sats} < {self.min_satellites})"
                elif is_hdop_bad:
                    self.transition_reason = f"HDOP Degradation ({gnss.hdop:.1f} >= {self.max_hdop_outage})"
                else:
                    self.transition_reason = f"GNSS Heartbeat Timeout ({dt_since_last_gnss:.2f}s)"

            if self.outage_start_time is not None:
                self.outage_duration = now - self.outage_start_time

        else:
            # GNSS is currently valid!
            self.last_valid_gnss_time = now
            self.last_valid_lat = gnss.lat
            self.last_valid_lon = gnss.lon

            if self.current_mode == self.MODE_IDR_ACTIVE:
                # GNSS has just returned after an outage!
                self.recovery_counter += 1
                if self.recovery_counter >= self.recovery_confirmation_count:
                    self.current_mode = self.MODE_FUSION_CONVERGING
                    self.transition_reason = f"GNSS Signal Restored ({gnss.num_sats} sats, HDOP {gnss.hdop:.1f}) - Fusion Engaged"
                    self.outage_start_time = None
                    self.outage_duration = 0.0
            elif self.current_mode == self.MODE_FUSION_CONVERGING:
                # Once convergence stabilizes, return to standard GNSS fix
                self.recovery_counter += 1
                if self.recovery_counter >= (self.recovery_confirmation_count + 10):
                    self.current_mode = self.MODE_GNSS_FIX
                    self.transition_reason = "Position Stabilized in Nominal GNSS Fix"
                    self.recovery_counter = 0
            else:
                # Nominal tracking: check if degraded
                if gnss.hdop > self.max_hdop_degraded or gnss.num_sats < (self.min_satellites + 2) or ai_condition == "URBAN_CANYON":
                    if self.current_mode != self.MODE_GNSS_DEGRADED:
                        self.current_mode = self.MODE_GNSS_DEGRADED
                        self.transition_reason = f"Sub-optimal GNSS: HDOP {gnss.hdop:.1f}, Sats {gnss.num_sats}"
                else:
                    self.current_mode = self.MODE_GNSS_FIX
                    self.transition_reason = "Nominal Open-Sky Fix"

        return self.current_mode, self.transition_reason
