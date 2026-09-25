"""Interactive Terminal User Interface (TUI) with real-time HUD and keyboard hotkeys."""

import os
import sys
import time
import select
import termios
import tty
from typing import Optional
from ..sensor_layer.data_types import SensorFrame, NavigationOutput


class NonBlockingKeyboard:
    """Captures keypresses without waiting for ENTER in Linux terminal."""

    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def get_key(self) -> Optional[str]:
        if not self.is_tty:
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return None

    def cleanup(self):
        if self.is_tty:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


class TerminalHUD:
    """Renders high-visibility ANSI HUD in terminal."""

    # ANSI Colors
    RESET = "\033[0m"
    BOLD = "\033[1m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    CYAN = "\033[96m"
    MAGENTA = "\033[95m"
    BLUE = "\033[94m"
    CLEAR_SCREEN = "\033[2J\033[H"

    def __init__(self):
        self.last_render_time = 0.0
        self.kb = NonBlockingKeyboard()

    def render(self, frame: SensorFrame, out: NavigationOutput) -> None:
        """Draw updated HUD to stdout."""
        now = time.time()
        # Limit render rate to 10 Hz to prevent terminal flicker
        if now - self.last_render_time < 0.1:
            return
        self.last_render_time = now

        # Format Mode Badge
        if out.mode == "GNSS_FIX":
            mode_badge = f"{self.GREEN}{self.BOLD}[ ● GNSS FIX ACTIVE ]{self.RESET}"
        elif out.mode == "GNSS_DEGRADED":
            mode_badge = f"{self.YELLOW}{self.BOLD}[ ⚠ GNSS DEGRADED (URBAN CANYON) ]{self.RESET}"
        elif out.mode == "IDR_ACTIVE":
            mode_badge = f"{self.RED}{self.BOLD}[ ⚡ IDR DEAD RECKONING ACTIVE - OUTAGE DETECTED ]{self.RESET}"
        elif out.mode == "FUSION_CONVERGING":
            mode_badge = f"{self.CYAN}{self.BOLD}[ 🔄 FUSION ENGAGED - CONVERGING DRIFT ]{self.RESET}"
        else:
            mode_badge = f"{self.YELLOW}[ {out.mode} ]{self.RESET}"

        # AI Condition Badge
        if out.ai_condition == "OPEN_SKY_NORMAL":
            ai_badge = f"{self.GREEN}{out.ai_condition}{self.RESET} ({out.ai_confidence*100:.1f}%)"
        elif out.ai_condition == "URBAN_CANYON_WEAK":
            ai_badge = f"{self.YELLOW}{out.ai_condition}{self.RESET} ({out.ai_confidence*100:.1f}%)"
        elif out.ai_condition == "TUNNEL_OUTAGE":
            ai_badge = f"{self.RED}{self.BOLD}{out.ai_condition}{self.RESET} ({out.ai_confidence*100:.1f}%)"
        else:
            ai_badge = f"{self.MAGENTA}{out.ai_condition}{self.RESET} ({out.ai_confidence*100:.1f}%)"

        # Format GNSS line
        if frame.gnss.valid:
            gnss_str = f"{self.GREEN}VALID{self.RESET} | Sats: {frame.gnss.num_sats:2d} | HDOP: {frame.gnss.hdop:4.1f} | Speed: {frame.gnss.speed:4.1f} m/s"
        else:
            gnss_str = f"{self.RED}OUTAGE / OCCLUDED{self.RESET} | Sats: 0 | HDOP: 99.9 | Speed: 0.0 m/s"

        lines = [
            self.CLEAR_SCREEN,
            f"{self.CYAN}{self.BOLD}========================================================================={self.RESET}",
            f"{self.CYAN}{self.BOLD}      🛰️  EDGE AI + GNSS / INTELLIGENT DEAD RECKONING (IDR) SYSTEM       {self.RESET}",
            f"{self.CYAN}{self.BOLD}========================================================================={self.RESET}",
            f" SYSTEM STATUS  : {mode_badge}",
            f" EDGE AI DECISION: {ai_badge} | Latency: {out.latency_ms:.2f} ms",
            f" SENSOR SOURCE  : {frame.source.upper()}",
            f"-------------------------------------------------------------------------",
            f" {self.BOLD}ESTIMATED FUSED POSITION (EKF){self.RESET}:",
            f"   Latitude : {out.est_lat:11.7f}°     Longitude : {out.est_lon:11.7f}°",
            f"   Speed    : {out.est_speed:5.1f} m/s ({out.est_speed * 3.6:5.1f} km/h)  Heading : {out.est_heading:5.1f}°",
            f"   Local ENU: East {out.est_east:7.2f} m | North {out.est_north:7.2f} m",
            f"-------------------------------------------------------------------------",
            f" {self.BOLD}GNSS HEALTH & TELEMETRY{self.RESET}:",
            f"   Signal   : {gnss_str}",
            f"   Raw Fix  : Lat {out.raw_lat or 0.0:10.6f} | Lon {out.raw_lon or 0.0:10.6f}",
            f"-------------------------------------------------------------------------",
            f" {self.BOLD}INTELLIGENT DEAD RECKONING (IDR) SUBSYSTEM{self.RESET}:",
            f"   Outage Timer     : {out.outage_duration:5.1f} s",
            f"   Est. Drift Error : {out.accumulated_drift:5.2f} meters",
            f"   ZUPT Standstill  : {self.GREEN if out.zupt_active else self.YELLOW}{'YES (Zero-Velocity Hold)' if out.zupt_active else 'NO (Vehicle Moving)'}{self.RESET}",
            f"   IMU Accelerometer: Ax: {frame.imu.ax:+6.2f} | Ay: {frame.imu.ay:+6.2f} | Az: {frame.imu.az:+6.2f} m/s²",
            f"   IMU Gyroscope    : Gx: {frame.imu.gx:+6.3f} | Gy: {frame.imu.gy:+6.3f} | Gz: {frame.imu.gz:+6.3f} rad/s",
            f"=========================================================================",
            f" {self.BOLD}INTERACTIVE CONTROLS{self.RESET}:",
            f"   [{self.BOLD}O{self.RESET}] Trigger/Toggle GNSS Outage (Simulate Tunnel Entrance)",
            f"   [{self.BOLD}R{self.RESET}] Restore GNSS Signal (Simulate Tunnel Exit & Fusion)",
            f"   [{self.BOLD}M{self.RESET}] Show Mobile Phone Connect Link",
            f"   [{self.BOLD}W{self.RESET}] Web Dashboard: http://localhost:8080",
            f"   [{self.BOLD}Q{self.RESET}] Exit Prototype",
            f"=========================================================================",
        ]
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def check_key(self) -> Optional[str]:
        return self.kb.get_key()

    def close(self):
        self.kb.cleanup()
