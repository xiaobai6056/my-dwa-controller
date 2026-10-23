#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
uname -a
lsb_release -ds
nproc
gazebo --version || true
dpkg-query -W ros-humble-nav2-bringup ros-humble-nav2-controller ros-humble-dwb-core ros-humble-gazebo-ros ros-humble-turtlebot3-gazebo
cd /home/user/ros2_ws/src/my_dwa_controller
git rev-parse HEAD
git branch --show-current
sha256sum config/nav2_params.yaml config/nav2_params_dwb.yaml /home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so