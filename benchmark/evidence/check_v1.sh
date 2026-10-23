#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
python3 -m unittest discover -s /home/user/ros2_ws/scripts/tests -p 'test_*.py' -v
ament_flake8
ament_pep257 --exclude benchmark/results benchmark/evidence
sha256sum -c benchmark/evidence/unchanged.sha256
bash -n benchmark/start_scene.sh
bash -n /home/user/ros2_ws/scripts/start_headless_baseline.sh
bash -n /home/user/ros2_ws/scripts/start_ui_baseline.sh
python3 benchmark/evidence/validate_v1.py benchmark/results/v1/static_v1_20261019
