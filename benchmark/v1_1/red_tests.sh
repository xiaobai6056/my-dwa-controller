#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/build/my_dwa_controller
cmake --build . --target test_controller -j2
ROS_DOMAIN_ID=97 ./test_controller \
  --gtest_filter='ControllerTest.Terminal*:ControllerTest.PropagatesInvalidVelocityAndPathContractFailures' \
  --gtest_output=xml:/home/user/ros2_ws/src/my_dwa_controller/benchmark/v1_1/evidence/red_tests.xml
