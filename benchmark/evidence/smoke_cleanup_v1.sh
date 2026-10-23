#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
python3 run_suite.py --controllers mydwa --scenes S01 --repeats 1 \
  --run-id "${1:-cleanup_smoke_20261019_002}"
