# SWARMX — Decentralized Five-AMR Warehouse Demo

SWARMX is a ROS 2 and Gazebo demonstration of five autonomous warehouse AMRs
with local navigation, LiDAR obstacle response, peer-to-peer coordination,
task assignment, charging bays, and a live fleet dashboard.

Repository: <https://github.com/Maazwasi/amr_ws>

## System requirements

- Ubuntu with a graphical desktop
- ROS 2 Lyrical installed at `/opt/ros/lyrical`
- Gazebo Sim with the matching `ros_gz_sim` and `ros_gz_bridge` packages
- `colcon`, `rosdep`, Python 3, Git, and an Ubuntu terminal emulator
- A computer capable of running five simulated robots; a dedicated GPU helps

Start the demo from normal **Ubuntu Terminal** windows, not the VS Code
terminal.

## Download the repository

```bash
cd ~
git clone https://github.com/Maazwasi/amr_ws.git
cd amr_ws
git checkout sih-five-amr-mvp
```

If this branch is already the repository default, the checkout command is not
required.

## Install ROS package dependencies

```bash
source /opt/ros/lyrical/setup.bash
cd ~/amr_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`sudo rosdep init` normally needs to be run only once per computer.

## Build the workspace

```bash
cd ~/amr_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

The launch scripts source the workspace automatically after it is built. If a
separate Nav2 workspace exists, it is detected at `~/nav2_ws`. Supply another
location with `NAV2_WS=/path/to/nav2_ws`.

If executable permissions were not preserved by the checkout, run:

```bash
chmod +x start_swarmx_demo.sh run_fleet_demo_ros.sh \
  run_fleet_dashboard.sh run_swarmx_control_dashboard.sh \
  run_swarmx_v2_large.sh run_stage6_negotiation.sh
```

## Legacy demo — start everything automatically

Open an Ubuntu Terminal and run:

```bash
cd ~/amr_ws
./start_swarmx_demo.sh
```

This opens two visible Ubuntu Terminal windows:

1. **SWARMX Gazebo + ROS** — warehouse, five AMRs, controllers and RViz.
2. **SWARMX Dashboard** — dashboard server on port `8090`.

Wait for Gazebo and all five AMRs to load, then open:

<http://localhost:8090/fleet>

## Legacy demo — manual startup

Use this method if automatic terminal launching is unavailable.

### Ubuntu Terminal 1 — Gazebo, ROS and RViz

```bash
cd ~/amr_ws
./run_fleet_demo_ros.sh
```

### Ubuntu Terminal 2 — dashboard

```bash
cd ~/amr_ws
./run_fleet_dashboard.sh
```

Open <http://localhost:8090/fleet> in a browser on the same computer.

## SWARMX V2 50 × 40 m dashboard-controlled demo

V2 uses a persistent control dashboard on port **8091**. Start this lightweight
service once in Ubuntu Terminal; it remains available when the simulation is
stopped:

```bash
cd ~/amr_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select amr_simulation edge_ai_nav --symlink-install
./run_swarmx_control_dashboard.sh
```

Open <http://localhost:8091/fleet>, then use **SIMULATION CONTROL**:

- **START SIMULATION** starts Gazebo, fleet RViz, SLAM, and all five AMRs.
- **STOP SIMULATION** gracefully stops only the process tree owned by the dashboard.
- **RESTART** finishes STOP before starting one clean replacement stack.
- Status reports `STOPPED`, `STARTING`, `RUNNING`, `STOPPING`, `ERROR`, or
  `EXTERNAL`.

The dashboard uses this fixed, demo-safe profile:

```text
use_nav2:=false enabled:=false use_rviz:=true use_dashboard:=false
```

The child launch does not start a second dashboard, and the browser cannot
provide arbitrary shell commands. Do not run legacy and V2 Gazebo stacks at
the same time.

The control API used by the buttons is intentionally limited to:

```text
GET  /api/simulation/status
POST /api/simulation/start
POST /api/simulation/stop
POST /api/simulation/restart
```

The managed stack runs in its own process group. STOP sends SIGINT first,
then SIGTERM and SIGKILL only if the owned process group does not exit. A
dashboard restart never takes ownership of an already-running external stack.

The dashboard's **AUTO** assignment publishes a high-level pickup/drop task;
each available AMR bids locally and the winner executes its A* route. **MANUAL**
assignment selects an AMR while retaining its local navigation and safety
controllers. The map and robot positions come from live V2 telemetry; battery
percentage is simulated by the robot controller. Physical validation uses
sampled odometry separation and static-clearance gates, not contact sensors;
do not describe the dashboard's unmeasured collision field as a measured zero.

See [the V2 checkpoint](SWARMX_V2_CHECKPOINT.md) for measured results and open
gates and [the implementation report](SWARMX_V2_IMPLEMENTATION_REPORT.md) for
parameters and recording steps. The V2 physical occupancy brake can hold even
the negotiated winner when another robot is too close. Yield-pocket recovery
and alternate-aisle rerouting have focused live evidence. Baseline fleet
delivery/charging, charging departures, staged 0.50/0.60 m/s motion/fleet runs,
and final-speed priority/recovery/rerouting have passed. Final defaults are
**0.60 m/s linear and 0.90 rad/s angular**. Both packages build and 141 tests
pass. RViz TF/labels agree with physical odometry. Retained result summaries
are in [validation evidence](SWARMX_V2_VALIDATION_EVIDENCE.json).

V2 uses port **8091**, including access from another computer on the same LAN:
`http://SIMULATION_COMPUTER_IP:8091/fleet`. The legacy instructions below use
port 8090. Both dashboards are demonstration interfaces without authentication.

For supervised physical validation, source `install/setup.bash` in an Ubuntu
Terminal and run `python3 validate_v2_fleet.py fleet`. The validator launches a
fresh stack, assigns five AUTO tasks, monitors actual Gazebo odometry and stops
its own stack on completion or a failed gate. `--visible` shows Gazebo for the
same scenario; `priority`, `equal`, `recovery`, `reroute`, `later` and
`charging_gap` select focused checks. `motion` exercises straight, 30°, 45°,
90° and successive opposing turns in the free central corridor.
`--speed 0.5` and `--speed 0.6` select staged speed checks; they do not alter
the accepted launch defaults. Keep only one simulation running.

### V2 recording sequence

Start the persistent dashboard in **Ubuntu Terminal**, not the VS Code terminal:

```bash
cd ~/amr_ws
./run_swarmx_control_dashboard.sh
```

Open <http://localhost:8091/fleet>, press **START SIMULATION**, and wait for
`RUNNING`. Keep Gazebo, fleet RViz, and the dashboard visible. Fleet RViz opens
with all five LiDARs enabled, ALPHA 3 highlighted, and ALPHA 3 as the live SLAM
source.

1. Show all five named/color-coded AMRs and EDGE AI / DECENTRALIZED banners.
2. Select AUTO, Warehouse Transfer and HIGH (numeric priority 3).
3. Create these tasks about three seconds apart, matching the accepted fleet
   runs: PICKUP_03→DROP_07, PICKUP_01→DROP_01, PICKUP_04→DROP_06,
   PICKUP_02→DROP_02, PICKUP_10→DROP_03.
4. Show live motion, decisions, task completion and automatic return/charging.
5. For reproducible close-up negotiation/reroute clips, press **STOP
   SIMULATION**, then run one focused scenario at a time in Ubuntu Terminal:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 validate_v2_fleet.py priority --speed 0.6 --visible
python3 validate_v2_fleet.py reroute --speed 0.6 --visible
python3 validate_v2_fleet.py equal --speed 0.4 --visible
python3 validate_v2_fleet.py later --speed 0.4 --visible
```

Each command stages initial poses/tasks, then uses actual local controllers
and peer decisions. It stops its own stack automatically at the gate. Winners
and states are not faked. Do not change frozen geometry, friction, footprint,
prediction, physical guard or speed parameters before recording.

To view the V2 dashboard from another computer on the same local network, use:

```text
http://SIMULATION_COMPUTER_IP:8091/fleet
```

Port `8091` must be allowed by the simulation computer's firewall. Do not
expose this unauthenticated demonstration dashboard directly to the internet.

## Recommended demonstration sequence

1. Keep Gazebo and the fleet dashboard visible side by side.
2. Wait until all five AMR cards show live data.
3. Select and submit a task from the dashboard.
4. Observe the selected AMR physically follow its route in Gazebo.
5. Show live task progress, localization, coordination state and peer count.
6. Assign another task to demonstrate decentralized allocation.
7. Assign charging and show slot reservation plus charging status.

## Stop the demo

For V2, press **STOP SIMULATION** and wait for `STOPPED`. Gazebo, RViz, SLAM,
bridges, and fleet nodes stop while the dashboard remains available. Press
`Ctrl+C` in the control-dashboard terminal only when the web control service
itself is no longer needed.

For the legacy two-terminal demo, press `Ctrl+C` in both terminals.

Do not start a second simulation while one is running. The launch script uses
a lock to prevent duplicate Gazebo stacks.

## Rebuild after source changes

Stop the simulation and dashboard, then run:

```bash
cd ~/amr_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

Restart the simulation and dashboard after the build finishes.

## Troubleshooting

### Dashboard does not open

Confirm the dashboard terminal is still running, then check:

```bash
curl http://localhost:8091/fleet
curl http://localhost:8091/api/simulation/status
```

### Dashboard opens but shows no live robots

For V2, check **SIMULATION CONTROL**. Press **START SIMULATION** when it shows
`STOPPED`; wait when it shows `STARTING`; or correct the short error message
and press **RETRY START** when it shows `ERROR`.

### Workspace setup file is missing

The workspace has not built successfully. Repeat the build instructions and
confirm that `install/setup.bash` exists.

### A simulation is already running

`EXTERNAL` means V2 was started outside the control dashboard. Return to its
Ubuntu Terminal and stop it with `Ctrl+C`. The dashboard deliberately refuses
to kill or duplicate an unowned stack. Wait for `STOPPED`, then press **START
SIMULATION**. Avoid killing unrelated ROS processes.

### Gazebo is slow

Close unnecessary applications and RViz panels. A real-time factor below 1.0
is acceptable if physical movement remains stable enough for the demo.

## Main launch scripts

### Dashboard-controlled V2 simulation

Start the persistent control dashboard once from Ubuntu Terminal:

```bash
cd /home/maaz-wasi/amr_ws
./run_swarmx_control_dashboard.sh
```

Open <http://localhost:8091/fleet>. The **SIMULATION CONTROL** panel starts,
stops, or restarts the V2 Gazebo/RViz stack. The managed demo profile is fixed
to `use_nav2:=false enabled:=false use_rviz:=true`; arbitrary commands are not
accepted. The dashboard remains available after STOP. Stop any manually
launched V2 stack before using dashboard process control.

The manual workflow remains available for diagnostics:

```bash
cd /home/maaz-wasi/amr_ws
./run_swarmx_v2_large.sh use_nav2:=false enabled:=false use_rviz:=true use_dashboard:=false
```

A manually launched stack appears as `EXTERNAL` and must be stopped from its
owning Ubuntu Terminal.

### Experimental 2× midpoint AMR / 3× speed upgrade

The operator revised the size from 3× to 2× original: chassis
1.80 × 1.40 × 0.40 m, wheel-inclusive footprint 1.80 × 1.52 m.
The warehouse is unchanged from the enlarged variant. View all five parked
robots with Gazebo, one RViz SLAM/LiDAR view, and the dashboard in Ubuntu Terminal:

```bash
cd /home/maaz-wasi/amr_ws
bash run_swarmx_v2_large.sh enabled:=false use_rviz:=true use_dashboard:=false max_linear_speed:=3.80 max_angular_speed:=7.50
```

Open <http://localhost:8091/fleet>, assign a task, and watch the fleet's live
routes and LiDAR returns in RViz. ALPHA 3 is the highlighted SLAM/LiDAR demo
robot. The requested speed values are
caps; obstacle avoidance and the controller can command less. This 3.80 m/s,
7.50 rad/s profile is not physically validated. For the conservative tested
profile, override with `max_linear_speed:=0.60 max_angular_speed:=0.50`.
See [SLAM recording instructions](SLAM_RECORDING.md).

Existing 3× physical reports do not validate this revised 2× platform.

This is a separate, **not-yet-accepted** variant; it does not replace the demo.
See [the baseline audit](SWARMX_SCALE_SPEED_AUDIT.md) and
[the live validation report](SWARMX_SCALE_SPEED_REPORT.md). In Ubuntu Terminal,
after stopping any existing simulation:

```bash
cd /home/maaz-wasi/amr_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select amr_simulation edge_ai_nav --symlink-install
source install/setup.bash
python3 validate_v2_large_stages.py --visible
```

The runner advances through 0.60 → 0.90 → 1.20 → 1.50 → 1.80 m/s
only after each stage's gates pass. It stops on failure. Detailed
attempts are saved as `SWARMX_LARGE_*.json`; current completed gates are in
`SWARMX_LARGE_STAGE_PROGRESS.json`. Viewing the enlarged fleet at the conservative
initial cap: `bash run_swarmx_v2_large.sh enabled:=false use_rviz:=true use_dashboard:=false max_linear_speed:=0.60 max_angular_speed:=0.50`.
Dashboard: <http://localhost:8091/fleet>. Do not describe 1.80 m/s as verified
until the final physical and concurrent-fleet gates have passed.

- `start_swarmx_demo.sh` — opens both demo terminals.
- `run_fleet_demo_ros.sh` — starts the five-AMR ROS/Gazebo stack.
- `run_fleet_dashboard.sh` — starts the dashboard server.
- `run_swarmx_control_dashboard.sh` — persistent V2 START/STOP/RESTART dashboard.
- `run_swarmx_v2_large.sh` — manual V2 Gazebo/RViz/fleet launch wrapper.
- `run_stage6_negotiation.sh` — underlying decentralized coordination launch.
