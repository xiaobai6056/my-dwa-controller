#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
root=/home/user/ros2_experiments/simplified_bypass_20261029
source "$root/baseline_install/setup.bash"
mkdir -p "$root/source/experiment" "$root/results"
cp /path/to/local-experiment-inputs/simplified_bypass_20261029/{probe.cpp,probe_geometry.hpp,geometry_test.cpp,CMakeLists.txt,export.py,setup.py,build_probe.sh} "$root/source/experiment/"
cp "$root/source/test/migration/common.hpp" "$root/source/experiment/"
cmake -S "$root/source/experiment" -B "$root/probe_build" -DCMAKE_BUILD_TYPE=Release > "$root/logs/probe_configure.log" 2>&1
cmake --build "$root/probe_build" -j3 > "$root/logs/probe_build.log" 2>&1
ctest --test-dir "$root/probe_build" --output-on-failure | tee "$root/logs/probe_test.log"
"$root/probe_build/probe" "$root/inputs.json" "$root/results" > "$root/logs/probe.log" 2>&1
cat "$root/results/summary.json"
