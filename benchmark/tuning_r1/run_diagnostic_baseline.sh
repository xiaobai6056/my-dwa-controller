#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
sha256sum -c evidence/unchanged.sha256
python3 run_suite.py --controllers mydwa --scenes S04 --repeats 3 \
  --run-id tuning_r1_diagnostic_baseline_20261019
