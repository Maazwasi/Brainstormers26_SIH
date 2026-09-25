#!/bin/bash
# ============================================================
#  Edge AI + GNSS/IDR  — AMR Warehouse Simulation Demo
#  Single-command launcher. Run from the amr_ws directory.
# ============================================================
set -e

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_SETUP="/opt/ros/lyrical/setup.bash"
NAV2_SETUP="/home/maaz-wasi/nav2_ws/install/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

# ── Colours ──────────────────────────────────────────────────
GRN="\033[1;32m"
YLW="\033[1;33m"
CYN="\033[1;36m"
RED="\033[1;31m"
NC="\033[0m"

banner() {
    echo -e "${CYN}\n╔══════════════════════════════════════════════════════╗"
    printf "${CYN}║  %-52s║\n${NC}" "$1"
    echo -e "${CYN}╚══════════════════════════════════════════════════════╝${NC}"
}

# ── Validate environment ─────────────────────────────────────
banner "Checking environment..."

if [[ ! -f "$ROS_SETUP" ]]; then
    echo -e "${RED}[ERROR]${NC} ROS 2 not found at $ROS_SETUP. Please install ROS 2 Lyrical first."
    exit 1
fi

source "$ROS_SETUP"
echo -e "${GRN}[OK]${NC} ROS 2 Lyrical sourced."

# ── Source Nav2 workspace ────────────────────────────────────
if [[ ! -f "$NAV2_SETUP" ]]; then
    echo -e "${RED}[ERROR]${NC} Nav2 workspace not found at $NAV2_SETUP"
    exit 1
fi

source "$NAV2_SETUP"
echo -e "${GRN}[OK]${NC} Nav2 workspace sourced."

# ── Build workspace ──────────────────────────────────────────
banner "Building workspace..."

cd "$WS_DIR"

colcon build --symlink-install \
    --packages-select amr_simulation edge_ai_nav 2>&1 | tail -5

source "$WS_SETUP"

echo -e "${GRN}[OK]${NC} Workspace built and sourced."

# ── Parse arguments ──────────────────────────────────────────
USE_NAV2="true"
USE_RVIZ="true"
HEADLESS="false"
AUTO_NAV="false"

for arg in "$@"; do
    case $arg in
        --headless)
            HEADLESS="true"
            ;;
        --no-nav2)
            USE_NAV2="false"
            ;;
        --no-rviz)
            USE_RVIZ="false"
            ;;
        --auto-nav)
            AUTO_NAV="true"
            ;;
        --help|-h)
            echo ""
            echo "  Usage: ./start_demo.sh [OPTIONS]"
            echo ""
            echo "  Options:"
            echo "    --headless     Run Gazebo in server-only mode (no GUI)"
            echo "    --no-nav2      Skip Nav2 bringup"
            echo "    --no-rviz      Skip RViz2 visualization"
            echo "    --auto-nav     Auto-launch AMR waypoint mission navigator"
            echo "    --help         Show this help"
            echo ""
            exit 0
            ;;
    esac
done

# ── Launch main simulation ───────────────────────────────────
banner "Launching Warehouse AMR + Edge AI Demo"

echo -e "${YLW}► use_sim_time=true  |  use_nav2=$USE_NAV2  |  headless=$HEADLESS${NC}"

echo ""
echo -e "  ${GRN}Demo Scenario:${NC}"
echo "   1. AMR spawns at Outdoor Loading Bay (GNSS ON ✓)"
echo "   2. Robot drives toward warehouse entrance"
echo "   3. GNSS signal degrades under canopy (Edge AI detects)"
echo "   4. AMR enters warehouse → GNSS OUTAGE → IDR activates"
echo "   5. AMR navigates under pure Dead Reckoning (IMU/Odom)"
echo "   6. AMR exits warehouse → GNSS restores → EKF Fusion corrects"

echo ""
echo -e "  ${CYN}RViz2 Topics to visualize:${NC}"
echo "   /amr/path_fused      — EKF fused path"
echo "   /amr/path_idr        — IDR-only path"
echo "   /amr/nav_status      — JSON status (mode, AI condition, drift)"
echo "   /amr/status_marker   — Floating status banner above robot"
echo "   /gps/zone_status     — Physical zone (Outdoor/Canopy/Indoor)"
echo ""

# ── Launch integrated warehouse simulation ───────────────────
ros2 launch edge_ai_nav warehouse_edge_ai_nav.launch.py \
    use_sim_time:=true \
    use_nav2:=$USE_NAV2 \
    use_rviz:=$USE_RVIZ \
    headless:=$HEADLESS &

MAIN_PID=$!

# ── Optionally launch auto waypoint navigator ─────────────────
if [[ "$AUTO_NAV" == "true" ]]; then
    echo -e "\n${YLW}[AUTO-NAV] Waiting 15s for simulation to initialize...${NC}"

    sleep 15

    echo -e "${GRN}[AUTO-NAV] Starting AMR Warehouse Mission Navigator...${NC}"

    ros2 run edge_ai_nav amr_warehouse_navigator &

    NAV_PID=$!
fi

# ── Trap clean shutdown ──────────────────────────────────────
cleanup() {
    echo -e "\n${RED}[SHUTDOWN] Stopping simulation...${NC}"

    kill $MAIN_PID 2>/dev/null || true

    [[ -n "$NAV_PID" ]] && kill $NAV_PID 2>/dev/null || true

    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "rviz2"  2>/dev/null || true

    echo -e "${GRN}[DONE] All processes stopped.${NC}"
}

trap cleanup EXIT INT TERM

echo -e "\n${GRN}[RUNNING]${NC} Press Ctrl+C to stop the demo.\n"

wait $MAIN_PID
