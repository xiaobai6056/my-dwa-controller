#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
sha256sum ../src/controller.cpp ../test/test_controller.cpp ../config/nav2_params.yaml ../config/nav2_params_dwb.yaml /home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so > v1_1/evidence/tested_artifacts.sha256
cp /home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so v1_1/evidence/libmy_dwa_controller.so.tested
for attempt in 001 002 003 004 005; do
  python3 run_suite.py --controllers mydwa --scenes S04 --repeats 1 --run-id v1_1_S04_${attempt}
  sha256sum -c v1_1/evidence/tested_artifacts.sha256
  python3 v1_1/evidence/check_s04.py results/v1/v1_1_S04_${attempt}
done
