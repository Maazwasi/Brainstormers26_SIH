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
  run_fleet_dashboard.sh run_stage6_negotiation.sh
```

## Start everything automatically

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

## Manual startup

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

## SWARMX V2 50 × 40 m preview (validation in progress)

The V2 warehouse uses a separate launch mode and dashboard port. It starts the
five ALPHA 1–5 robots, local controllers, peer coordination, task allocation,
charging, and dashboard together in **one Ubuntu Terminal**:

```bash
cd ~/amr_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select amr_simulation edge_ai_nav --symlink-install
./run_swarmx_v2_demo.sh
```

Open <http://localhost:8091/fleet>. Use `Ctrl+C` in the same Ubuntu Terminal to
stop the V2 stack. For a lower-load, non-graphical check, launch with
`./run_swarmx_v2_demo.sh headless:=true use_rviz:=false enabled:=false`.
Do not run the legacy and V2 Gazebo stacks at the same time.

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

Run in **Ubuntu Terminal**, not the VS Code terminal:

```bash
cd ~/amr_ws
./run_swarmx_v2_demo.sh enabled:=false use_rviz:=true
```

Open <http://localhost:8091/fleet>. Keep Gazebo and the dashboard side by side;
RViz is optional (omit `use_rviz:=true` to reduce rendering load).

1. Show all five named/color-coded AMRs and EDGE AI / DECENTRALIZED banners.
2. Select AUTO, Warehouse Transfer and HIGH (numeric priority 3).
3. Create these tasks about three seconds apart, matching the accepted fleet
   runs: PICKUP_03→DROP_07, PICKUP_01→DROP_01, PICKUP_04→DROP_06,
   PICKUP_02→DROP_02, PICKUP_10→DROP_03.
4. Show live motion, decisions, task completion and automatic return/charging.
5. For reproducible close-up negotiation/reroute clips, stop the fleet first
   with Ctrl+C, then run one focused scenario at a time in Ubuntu Terminal:

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

To view the dashboard from another computer on the same local network, replace
`localhost` with the simulation computer's IP address:

```text
http://SIMULATION_COMPUTER_IP:8090/fleet
```

Port `8090` must be allowed by the simulation computer's firewall. Do not
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

Press `Ctrl+C` in both Ubuntu Terminal windows. With the automatic launcher,
interrupt or close both terminal windows it opened.

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
curl http://localhost:8090/fleet
```

### Dashboard opens but shows no live robots

Start `run_fleet_demo_ros.sh` first, wait for the ROS nodes to launch, and then
restart `run_fleet_dashboard.sh`.

### Workspace setup file is missing

The workspace has not built successfully. Repeat the build instructions and
confirm that `install/setup.bash` exists.

### A simulation is already running

Return to the existing Gazebo Ubuntu Terminal and stop it with `Ctrl+C`. Wait a
few seconds and launch again. Avoid killing unrelated ROS processes.

### Gazebo is slow

Close unnecessary applications and RViz panels. A real-time factor below 1.0
is acceptable if physical movement remains stable enough for the demo.

## Main launch scripts

- `start_swarmx_demo.sh` — opens both demo terminals.
- `run_fleet_demo_ros.sh` — starts the five-AMR ROS/Gazebo stack.
- `run_fleet_dashboard.sh` — starts the dashboard server.
- `run_stage6_negotiation.sh` — underlying decentralized coordination launch.
