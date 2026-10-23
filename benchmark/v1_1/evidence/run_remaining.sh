#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller/benchmark
for attempt in 001 002 003 004 005; do
  python3 v1_1/evidence/check_s04.py results/v1/v1_1_S04_${attempt}
done
sha256sum -c v1_1/evidence/tested_artifacts.sha256
python3 run_suite.py --controllers mydwa --scenes S06 --repeats 3 --run-id v1_1_S06
sha256sum -c v1_1/evidence/tested_artifacts.sha256
python3 run_suite.py --controllers mydwa --scenes S01 S03 S05 --repeats 2 --run-id v1_1_tracking
sha256sum -c v1_1/evidence/tested_artifacts.sha256
