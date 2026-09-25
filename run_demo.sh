#!/usr/bin/env bash
# ==============================================================================
# Edge AI + GNSS / Intelligent Dead Reckoning (IDR) Demo Launcher
# SIH Evaluation Startup Script
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo -e "\033[1;36m=================================================================\033[0m"
echo -e "\033[1;36m   🛰️  EDGE AI + GNSS / INTELLIGENT DEAD RECKONING PROTOTYPE     \033[0m"
echo -e "\033[1;36m=================================================================\033[0m"

# Check Python environment
if ! command -v python3 &> /dev/null; then
    echo -e "\033[1;31m[ERROR] python3 could not be found.\033[0m"
    exit 1
fi

echo -e "\033[1;32m[✓] Python Environment: $(python3 --version)\033[0m"
echo -e "\033[1;32m[✓] Workspace: $WORKSPACE_DIR\033[0m"
echo -e "\033[1;32m[✓] Local LAN IP for Mobile Streaming: 192.168.1.6\033[0m"
echo -e "\033[1;32m[✓] Live Map Dashboard will be served at: http://localhost:8080\033[0m"
echo -e "\033[1;32m[✓] Phone Sensor Stream URL: http://192.168.1.6:8080/mobile\033[0m"
echo -e ""
echo -e "\033[1;33mStarting Interactive Terminal Prototype...\033[0m"
sleep 1

# Launch Python prototype
exec python3 "$WORKSPACE_DIR/main.py" "$@"
