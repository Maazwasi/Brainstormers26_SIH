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
