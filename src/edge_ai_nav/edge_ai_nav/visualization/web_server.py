"""Lightweight HTTP server serving real-time Leaflet map and mobile sensor streamer."""

import os
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Optional, Dict, Any, Deque
from collections import deque

from ..sensor_layer.data_types import SensorFrame, NavigationOutput


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class WebDashboardServer:
    """Manages web dashboard and mobile telemetry endpoints."""

    def __init__(self, pipeline, port: int = 8080, static_dir: Optional[str] = None):
        self.pipeline = pipeline
        self.port = port
        if static_dir is None:
            self.static_dir = os.path.join(os.path.dirname(__file__), "static")
        else:
            self.static_dir = static_dir

        self.latest_frame: Optional[SensorFrame] = None
        self.latest_output: Optional[NavigationOutput] = None
        
        # Recent trail buffers for map drawing (last 200 points)
        self.fused_trail: Deque[Dict[str, float]] = deque(maxlen=200)
        self.raw_trail: Deque[Dict[str, float]] = deque(maxlen=200)
        self.idr_trail: Deque[Dict[str, float]] = deque(maxlen=200)

        # Wire pipeline subscriber
        self.pipeline.add_subscriber(self.on_nav_update)

        self.server: Optional[ThreadedHTTPServer] = None
        self.server_thread: Optional[threading.Thread] = None

    def on_nav_update(self, frame: SensorFrame, out: NavigationOutput) -> None:
        self.latest_frame = frame
        self.latest_output = out

        # Append trail points
        if out.est_lat != 0.0:
            self.fused_trail.append({"lat": out.est_lat, "lon": out.est_lon, "mode": out.mode})
        if out.raw_lat and frame.gnss.valid:
            self.raw_trail.append({"lat": out.raw_lat, "lon": out.raw_lon})
        if out.idr_lat != 0.0:
            self.idr_trail.append({"lat": out.idr_lat, "lon": out.idr_lon})

    def start(self) -> None:
        handler_class = self._create_handler()
        self.server = ThreadedHTTPServer(("0.0.0.0", self.port), handler_class)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

    def stop(self) -> None:
        if self.server:
            self.server.shutdown()
            self.server.server_close()

    def _create_handler(self):
        dashboard = self

        class NavRequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # Suppress console logging to keep terminal HUD clean

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._serve_file(os.path.join(dashboard.static_dir, "index.html"), "text/html")
                elif self.path in ("/mobile", "/mobile.html"):
                    self._serve_file(os.path.join(dashboard.static_dir, "mobile.html"), "text/html")
                elif self.path == "/api/telemetry":
                    self._serve_telemetry()
                elif self.path.startswith("/static/") or self.path.endswith((".css", ".js", ".png", ".svg")):
                    rel_path = self.path.lstrip("/").replace("static/", "")
                    file_path = os.path.join(dashboard.static_dir, rel_path)
                    content_type = "text/javascript" if file_path.endswith(".js") else "text/css"
                    self._serve_file(file_path, content_type)
                else:
                    self.send_error(404, "Not Found")

            def do_POST(self):
                content_len = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_len).decode("utf-8")
                try:
                    payload = json.loads(body) if body else {}
                except Exception:
                    payload = {}

                if self.path == "/api/control":
                    action = payload.get("action")
                    if action == "toggle_outage":
                        is_outage = dashboard.pipeline.toggle_outage()
                        self._send_json({"success": True, "outage_active": is_outage})
                    elif action == "cut_gnss":
                        dashboard.pipeline.toggle_outage(force=True)
                        self._send_json({"success": True, "outage_active": True})
                    elif action == "restore_gnss":
                        dashboard.pipeline.toggle_outage(force=False)
                        self._send_json({"success": True, "outage_active": False})
                    else:
                        self.send_error(400, "Unknown action")

                elif self.path == "/api/sensor_stream":
                    dashboard.pipeline.ingest_mobile_payload(payload)
                    self._send_json({"status": "received", "frames": dashboard.pipeline.mobile_receiver.frame_counter})

                else:
                    self.send_error(404, "Unknown API endpoint")

            def _serve_telemetry(self):
                out = dashboard.latest_output
                frame = dashboard.latest_frame
                data = {
                    "output": out.to_dict() if out else {},
                    "frame": frame.to_dict() if frame else {},
                    "trails": {
                        "fused": list(dashboard.fused_trail),
                        "raw": list(dashboard.raw_trail),
                        "idr": list(dashboard.idr_trail),
                    }
                }
                self._send_json(data)

            def _send_json(self, data: Dict[str, Any]):
                payload = json.dumps(data).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(payload)

            def _serve_file(self, filepath: str, content_type: str):
                if not os.path.exists(filepath):
                    self.send_error(404, f"File {os.path.basename(filepath)} Not Found")
                    return
                with open(filepath, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

        return NavRequestHandler
