#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${MYDWA_WS}"/build/my_dwa_controller
cmake --build . --target test_controller -j2
ROS_DOMAIN_ID=97 ./test_controller \
  --gtest_filter='ControllerTest.Terminal*:ControllerTest.PropagatesInvalidVelocityAndPathContractFailures' \
  --gtest_output=xml:${REPO_ROOT}/benchmark/v1_1/evidence/red_tests.xml
