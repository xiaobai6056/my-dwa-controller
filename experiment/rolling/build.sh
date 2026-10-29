#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_experiments/simplified_bypass_20261029/baseline_install/setup.bash
root=/home/user/ros2_experiments/rolling_bypass_20261029
mkdir -p "$root/source/experiment/rolling"
cp /path/to/local-experiment-inputs/rolling_bypass_20261029/{rolling_model.hpp,rolling_model.cpp,model_test.cpp,rolling.cpp,audit_selected.cpp,CMakeLists.txt} "$root/source/experiment/rolling/"
cmake -S "$root/source/experiment/rolling" -B "$root/build" -DCMAKE_BUILD_TYPE=Release > "$root/logs/configure.log" 2>&1
cmake --build "$root/build" -j3 > "$root/logs/build.log" 2>&1
"$root/build/model_test" --gtest_output=xml:"$root/logs/model.xml"
