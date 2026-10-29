#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
root=/home/user/ros2_experiments/locked_reference_20261029
cmake -S "$root/source/experiment/rolling" -B "$root/build" -DCMAKE_BUILD_TYPE=Release > "$root/logs/configure.log" 2>&1
cmake --build "$root/build" -j3 > "$root/logs/build.log" 2>&1
ctest --test-dir "$root/build" --output-on-failure
