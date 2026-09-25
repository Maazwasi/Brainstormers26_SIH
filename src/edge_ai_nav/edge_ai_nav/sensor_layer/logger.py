"""Telemetry and sensor data logger and replayer."""

import os
import csv
import json
import time
from typing import Optional, List, Generator
from .data_types import SensorFrame, GNSSData, IMUData, NavigationOutput


class TelemetryLogger:
    """Thread-safe CSV and JSONL logger for offline analysis and playback."""

    def __init__(self, log_dir: str = "/home/maaz-wasi/amr_ws/logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"navigation_log_{timestamp_str}.csv")
        self.jsonl_path = os.path.join(self.log_dir, f"navigation_log_{timestamp_str}.jsonl")
        
        self.csv_file = open(self.csv_path, "w", newline="")
        self.jsonl_file = open(self.jsonl_path, "w")
        
        self.fieldnames = [
            "timestamp", "frame_id", "mode", "ai_condition", "ai_confidence",
            "raw_lat", "raw_lon", "raw_speed", "raw_heading", "gnss_hdop", "gnss_sats",
            "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz",
            "est_lat", "est_lon", "est_east", "est_north", "est_speed", "est_heading",
            "idr_lat", "idr_lon", "idr_east", "idr_north",
            "outage_duration", "accumulated_drift", "zupt_active"
        ]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def log(self, frame: SensorFrame, out: NavigationOutput) -> None:
        """Log a synchronized sensor frame and the navigation pipeline output."""
        row = {
            "timestamp": frame.timestamp,
            "frame_id": frame.frame_id,
            "mode": out.mode,
            "ai_condition": out.ai_condition,
            "ai_confidence": round(out.ai_confidence, 3),
            "raw_lat": frame.gnss.lat if frame.gnss.valid else "",
            "raw_lon": frame.gnss.lon if frame.gnss.valid else "",
            "raw_speed": round(frame.gnss.speed, 3) if frame.gnss.valid else "",
            "raw_heading": round(frame.gnss.heading, 2) if frame.gnss.valid else "",
            "gnss_hdop": round(frame.gnss.hdop, 2),
            "gnss_sats": frame.gnss.num_sats,
            "imu_ax": round(frame.imu.ax, 4),
            "imu_ay": round(frame.imu.ay, 4),
            "imu_az": round(frame.imu.az, 4),
            "imu_gx": round(frame.imu.gx, 4),
            "imu_gy": round(frame.imu.gy, 4),
            "imu_gz": round(frame.imu.gz, 4),
            "est_lat": round(out.est_lat, 7),
            "est_lon": round(out.est_lon, 7),
            "est_east": round(out.est_east, 3),
            "est_north": round(out.est_north, 3),
            "est_speed": round(out.est_speed, 3),
            "est_heading": round(out.est_heading, 2),
            "idr_lat": round(out.idr_lat, 7),
            "idr_lon": round(out.idr_lon, 7),
            "idr_east": round(out.idr_east, 3),
            "idr_north": round(out.idr_north, 3),
            "outage_duration": round(out.outage_duration, 2),
            "accumulated_drift": round(out.accumulated_drift, 3),
            "zupt_active": int(out.zupt_active),
        }
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # Write to JSONL
        self.jsonl_file.write(json.dumps({"frame": frame.to_dict(), "output": out.to_dict()}) + "\n")
        self.jsonl_file.flush()

    def close(self) -> None:
        """Close log file handles safely."""
        try:
            self.csv_file.close()
            self.jsonl_file.close()
        except Exception:
            pass
