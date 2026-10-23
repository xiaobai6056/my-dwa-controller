#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws
colcon build --packages-select my_dwa_controller --symlink-install
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --verbose
