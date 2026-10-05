#!/usr/bin/env bash
set -euo pipefail
if [[ ! -r /etc/os-release ]]; then echo "Cannot identify OS"; exit 2; fi
. /etc/os-release
if [[ "$ID:$VERSION_ID" != "ubuntu:24.04" ]]; then
  echo "ERROR: official target is Ubuntu 24.04 (Noble); detected $ID:$VERSION_ID"
  echo "Use .devcontainer/ on non-target development hosts."
  exit 2
fi

sudo apt update
sudo apt install -y curl software-properties-common ca-certificates
sudo add-apt-repository -y universe
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Installing the official ROS 2 apt source and ROS 2 Jazzy Desktop..."
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
  sudo apt install -y ros-jazzy-desktop ros-dev-tools
fi

# ROS setup may reference variables that are unset under `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
set -u

sudo apt install -y \
  python3-venv python3-rosdep python3-colcon-common-extensions \
  python3-fastapi python3-uvicorn python3-websockets python3-pydantic \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-rqt-graph

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements-dev.txt
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

echo "Bootstrap complete. Next: ./scripts/build.sh"
