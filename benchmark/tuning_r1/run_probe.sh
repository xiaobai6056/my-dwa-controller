#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=97
python3 /home/user/ros2_ws/src/my_dwa_controller/benchmark/tuning_r1/evidence/build_probe.py
