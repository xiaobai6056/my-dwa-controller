#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller
sha256sum -c benchmark/evidence/unchanged.sha256
bash -n benchmark/tuning_r1/run_diagnostic_baseline.sh
bash -n benchmark/tuning_r1/run_probe.sh
ament_flake8
ament_pep257 --exclude benchmark/results benchmark/evidence benchmark/tuning_r1/evidence
cd /home/user/ros2_ws
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
