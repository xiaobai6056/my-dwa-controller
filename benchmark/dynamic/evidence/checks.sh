#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
cd /home/user/ros2_ws/src/my_dwa_controller
python3 -m compileall -q benchmark/dynamic benchmark/test_dynamic.py
bash -n benchmark/dynamic/start_scene.sh benchmark/dynamic/run.sh
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
python3 -m unittest discover -s /home/user/ros2_ws/scripts/tests -v