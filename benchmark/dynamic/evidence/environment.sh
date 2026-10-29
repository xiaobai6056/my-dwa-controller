#!/usr/bin/env bash
set -e

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
uname -a
lsb_release -ds
nproc
gazebo --version || true
dpkg-query -W ros-humble-nav2-bringup ros-humble-nav2-controller ros-humble-dwb-core ros-humble-gazebo-ros ros-humble-turtlebot3-gazebo
cd "${REPO_ROOT}"
git rev-parse HEAD
git branch --show-current
sha256sum config/nav2_params.yaml config/nav2_params_dwb.yaml ${MYDWA_WS}/install/my_dwa_controller/lib/libmy_dwa_controller.so