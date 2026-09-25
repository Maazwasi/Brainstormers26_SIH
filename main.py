#!/usr/bin/env python3
"""Main Entrypoint for Edge AI + GNSS / Intelligent Dead Reckoning Prototype.

Run from any Ubuntu terminal:
    cd ~/amr_ws
    python3 main.py
"""

import sys
import os
import time
import signal

# Ensure src/edge_ai_nav is in Python module search path
workspace_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.join(workspace_dir, "src", "edge_ai_nav")
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

from edge_ai_nav.pipeline import NavigationPipeline
from edge_ai_nav.visualization.terminal_ui import TerminalHUD
from edge_ai_nav.visualization.web_server import WebDashboardServer


def main():
    print("\033[96mInitializing Edge AI + GNSS/IDR Navigation Prototype...\033[0m")
    
    # 1. Initialize Pipeline
    pipeline = NavigationPipeline(
        ref_lat=28.6139,
        ref_lon=77.2090,
        ref_alt=216.0,
        enable_logging=True
    )

    # 2. Start Web Dashboard Server (localhost:8080)
    web_server = WebDashboardServer(pipeline, port=8080)
    web_server.start()

    # 3. Initialize Interactive Terminal HUD
    hud = TerminalHUD()

    running = True

    def sig_handler(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        # 50 Hz loop
        dt = 0.02
        while running:
            t_start = time.time()

            # Execute pipeline step
            output = pipeline.step()
            frame = pipeline.last_frame

            # Render Terminal HUD
            if frame is not None:
                hud.render(frame, output)

            # Check keyboard hotkeys
            key = hud.check_key()
            if key:
                k = key.lower()
                if k == 'q':
                    break
                elif k == 'o':
                    # Toggle / Force Outage
                    is_out = pipeline.toggle_outage()
                elif k == 'r':
                    # Restore GNSS
                    pipeline.toggle_outage(force=False)
                elif k == 'm':
                    # Print Mobile Connect info
                    print("\n\033[92m📱 Open on your phone (connected to same Wi-Fi):\033[0m")
                    print("   \033[1mhttp://192.168.1.6:8080/mobile\033[0m\n")
                    time.sleep(2.0)

            # Sleep to maintain ~50 Hz
            elapsed = time.time() - t_start
            sleep_time = max(0.001, dt - elapsed)
            time.sleep(sleep_time)

    finally:
        hud.close()
        web_server.stop()
        if pipeline.logger:
            pipeline.logger.close()
        print("\n\033[92m[✓] Navigation prototype shutdown cleanly.\033[0m")


if __name__ == "__main__":
    main()
