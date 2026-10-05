FROM osrf/ros:jazzy-desktop-full
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-rqt-graph \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspaces/IEEE_TSYP
