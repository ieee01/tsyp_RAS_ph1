# LivingMap — installation and jury run instructions

This guide prepares a fresh computer and runs the complete mine simulation:
Writer exploration, SLAM, beacon memories, fire suppression and victim assistance.
Run the commands in **Bash**. Examples use `~/IEEE_TSYP`; change that path if you
extract the project elsewhere.

## 1. Prepare the computer

| Item | Use |
|---|---|
| Operating system | **Ubuntu 24.04 LTS Desktop**, preferably x86_64 |
| ROS distribution | **ROS 2 Jazzy** |
| Simulator | **Gazebo Harmonic**, installed through `ros-jazzy-ros-gz` |
| Navigation and mapping | **Nav2** and **SLAM Toolbox** |
| Suggested resources | 8 or more CPU cores, 16 GB RAM, 20 GB free disk space |
| Graphics | A desktop session with working OpenGL; Gazebo renders the robots' LiDAR sensors |
| Installation access | Internet connection and a user account with `sudo` access |

Ubuntu 24.04, Jazzy and Harmonic are the supported combination for this project.
Gazebo documents this pairing and supplies Harmonic through the ROS vendor
packages in its [installation guide](https://gazebosim.org/docs/harmonic/ros_installation/).

For a virtual machine, allocate the suggested resources and enable 3D
acceleration. Run the graphical demo inside the Ubuntu desktop session.

Check the operating system:

```bash
cat /etc/os-release
uname -m
```

The OS should report `ubuntu` and `24.04`. The project's automatic installer
checks this before installing dependencies.

## 2. Get the project

Install basic tools:

```bash
sudo apt update
sudo apt install -y git curl ca-certificates build-essential python3
```

Clone the implementation branch:

```bash
cd ~
git clone --branch complete-implementation https://github.com/aminebensaid66/IEEE_TSYP.git
cd ~/IEEE_TSYP
```

If the team supplies a ZIP or project folder, extract it to `~/IEEE_TSYP` and
start with `cd ~/IEEE_TSYP`. Use the submitted revision for the evaluation.

If an extracted archive has lost executable permissions:

```bash
chmod +x scripts/*.sh
```

## 3. Install the environment

### Automated installation

Run this once from the project root:

```bash
cd ~/IEEE_TSYP
./scripts/bootstrap.sh
```

Enter your Ubuntu password when `sudo` requests it. The script installs:

| Dependency | Purpose |
|---|---|
| `ros-jazzy-desktop`, `ros-dev-tools` | ROS 2, RViz and development tools |
| `ros-jazzy-ros-gz` | Gazebo Harmonic and the ROS/Gazebo bridge |
| `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | Robot navigation |
| `ros-jazzy-slam-toolbox` | LiDAR mapping |
| `ros-jazzy-rviz2`, `ros-jazzy-rqt-graph` | Visualization and ROS graph inspection |
| `python3-colcon-common-extensions`, `python3-rosdep` | Building and dependency resolution |
| `.venv` and `requirements-dev.txt` | Dashboard, Python utilities and development dependencies |

`rosdep` also installs the dependencies declared in each ROS package's
`package.xml`, including the navigation controller and planner plugins.

Continue when the script prints `Bootstrap complete` and exits successfully.
For individual installation commands, use [Appendix A](#appendix-a-manual-environment-installation)
instead of this script, then continue with step 4.

## 4. Build the project

```bash
cd ~/IEEE_TSYP
./scripts/build.sh
```

This compiles the ROS packages and installs the worlds, robot models,
configuration files and dashboard assets. The default build recreates the
generated `build/`, `install/` and `log/` directories. Build on the jury's
computer even if a supplied archive contains those directories.

The project scripts source ROS, activate `.venv` and load the workspace. For
direct `ros2` commands in a new terminal, load the same environment:

```bash
cd ~/IEEE_TSYP
source scripts/_env.sh
```

Optional automated checks:

```bash
./scripts/test.sh
```

## 5. Launch Gazebo, RViz and the application

In **terminal 1**:

```bash
cd ~/IEEE_TSYP
mkdir -p runtime
./scripts/run_demo.sh 2>&1 | tee runtime/demo.log
```

Keep this terminal open. The command launches the mine, all three robots,
SLAM Toolbox, the three Nav2 stacks, the beacon mesh, gateway, simulated
long distance link and browser application.

In **terminal 2**, after the windows appear:

```bash
cd ~/IEEE_TSYP
./scripts/health_check.sh
xdg-open http://localhost:8081
```

Allow the ROS nodes and navigation servers to finish starting. If the first
health check reports missing nodes, wait and run it again. It checks the
Writer and Executor navigation stacks; the fire robot can also be inspected:

```bash
source scripts/_env.sh
ros2 lifecycle get /firebot/controller_server
```

The controller should report `active`. Before starting a mission, the Writer
is normally **PAUSED** and the two rescue robots are **READY**.

| Page | Address |
|---|---|
| Command post and mission controls | [http://localhost:8081](http://localhost:8081) |
| Scenario placement, fault injection and replay settings | [http://localhost:8081/harness](http://localhost:8081/harness) |
| Current telemetry as JSON | [http://localhost:8081/api/state](http://localhost:8081/api/state) |

## 6. Run the demonstration

1. Open the command post and select the Writer outcome: **Destroyed**,
   **Stuck** or **Returns**.
2. Click **Run full demo**.
3. Observe the Writer explore and build the map. In the current default layout,
   the automatic destroyed/stuck sequence waits for **both victims, both gas
   hazards and the fire** before injecting the chosen outcome.
4. Observe the preserved beacon memories reach the command post after the
   Writer's sortie ends.
5. The fire robot receives its brief, extinguishes the fire, marks its beacon
   `RESOLVED` and returns to the entrance.
6. The rescue robot receives its brief, assists the victims in priority order
   and writes `RESOLVED` into their beacons.
7. Wait for **MISSION COMPLETE** and the rescue robot's detail to say
   **back at the entrance**. Objective completion can appear before its return
   journey finishes.

Gas hazards remain active and are avoided; the fleet has no gas neutralization
robot. The blocked path is another event that can be recorded during exploration.

The supplied default event positions are in `config/events.yaml`. Use the
harness to move victims, gases or the fire and restart with matching sensor
positions and Gazebo visuals. Keep **Autonomous** navigation and **Normal**
speed for the initial evaluation.

## 7. Stop and replay

Press **Ctrl+C** in terminal 1. For a clean replay:

```bash
cd ~/IEEE_TSYP
./scripts/reset_demo.sh
./scripts/run_demo.sh 2>&1 | tee runtime/demo.log
```

`reset_demo.sh` stops the demo processes and clears the previous run's persistent
beacon and dashboard state. It retains the saved event layout in `runtime/scenario/`.
The harness also provides **Restart simulation**.

To use the submitted default layout after experimenting, stop the demo, move
the active layout aside, and relaunch:

```bash
./scripts/reset_demo.sh
if [ -d runtime/scenario ]; then
  mv runtime/scenario "runtime/scenario-backup-$(date +%Y%m%d-%H%M%S)"
fi
./scripts/run_demo.sh
```

## 8. Troubleshooting

### `ros2`, `gz` or a project package is not found

Load the environment and inspect package discovery:

```bash
cd ~/IEEE_TSYP
source scripts/_env.sh
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
ros2 pkg prefix living_map_bringup
command -v gz
```

If the project package is missing, run `./scripts/build.sh`. If a ROS dependency
is missing, complete installation in step 3 or Appendix A. `gz` is supplied by
the Gazebo vendor packages and becomes available when ROS is sourced.

### Installation reports missing packages or dependency conflicts

Check that Ubuntu Universe and the ROS apt source are enabled using Appendix A.
For `ros-dev-tools` dependency conflicts on a fresh Ubuntu installation, the
official ROS guide also calls for `noble-updates` and `noble-backports` in the
Ubuntu apt sources. [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### Gazebo is blank, the map is absent or the robots stay frozen

Inspect the current log and simulation clock:

```bash
cd ~/IEEE_TSYP
source scripts/_env.sh
tail -n 80 runtime/demo.log
timeout 10 ros2 topic echo /clock --once
```

Gazebo uses GPU LiDAR rendering, so check the desktop's graphics driver and a
VM's 3D acceleration. The launch includes a guard that retries a Gazebo startup
with no clock. If startup still fails, stop the demo and use the clean replay
commands above.

To reduce CPU use while keeping Gazebo and the dashboard:

```bash
./scripts/reset_demo.sh
LIVING_MAP_RVIZ=false ./scripts/run_demo.sh 2>&1 | tee runtime/demo.log
```

### The browser cannot connect

Keep terminal 1 running and use port **8081**. Check the API and log:

```bash
curl -fsS http://localhost:8081/api/state
tail -n 80 runtime/demo.log
```

### Python reports a missing dashboard module

Restore the project Python environment:

```bash
cd ~/IEEE_TSYP
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
```

Then stop and relaunch the demo. Python dependencies are installed into `.venv`.

## 9. Demonstration without ROS

To demonstrate the information flow on a computer without ROS or a graphical
simulator:

```bash
cd ~/IEEE_TSYP
python3 tools/headless_demo.py
```

This exercises beacon messages, the radio mesh, gateway translation and an
inherited robot plan. It uses its own fixed example and does not run the
Gazebo world or validate physical navigation.

## Appendix A. Manual environment installation

Use these steps as an alternative to `./scripts/bootstrap.sh`. Start in Bash
on Ubuntu 24.04, with the project available at `~/IEEE_TSYP`.

### A1. Base packages and UTF-8 locale

The ROS installation guide requires a UTF-8 locale; see the
[official locale setup](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Ubuntu-Set-Locale.rst) for details.

```bash
sudo apt update
sudo apt install -y git curl ca-certificates software-properties-common \
  locales build-essential python3 python3-venv
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo apt update
sudo apt upgrade
```

### A2. Add the official ROS repository and install Jazzy

Install the official `ros2-apt-source` package, which supplies the repository
configuration and signing keys, following the
[official apt repository setup](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).

```bash
livingmap_ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
curl -fL -o /tmp/livingmap-ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${livingmap_ros_apt_version}/ros2-apt-source_${livingmap_ros_apt_version}.noble_all.deb"
sudo dpkg -i /tmp/livingmap-ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
source /opt/ros/jazzy/setup.bash
```

### A3. Install Gazebo, navigation, SLAM and application dependencies

`ros-jazzy-ros-gz` installs the Harmonic integration from the ROS repository.
The navigation package names follow the [Nav2 installation guide](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/);
mapping uses the apt package published by [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/jazzy#install).

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rqt-graph \
  python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-fastapi python3-uvicorn python3-websockets python3-pydantic python3-yaml
```

### A4. Prepare Python and resolve package dependencies

```bash
cd ~/IEEE_TSYP
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

Return to **step 4** to build, then **step 5** to launch the application.
