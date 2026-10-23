#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$here/perf_build"
touch "$here/perf_build/AMENT_IGNORE"
includes=(-I/home/user/ros2_ws/src/my_dwa_controller/include -I/usr/include/eigen3)
while IFS= read -r directory; do includes+=(-I"$directory"); done < <(find /opt/ros/humble/include -mindepth 1 -maxdepth 1 -type d | sort)
g++ -std=c++17 -O2 -shared -fPIC -pthread "${includes[@]}" "$here/perf_probe.cpp" -ldl -o "$here/perf_build/libnavperf.so"
printf '%s\n' "$here/perf_build/libnavperf.so"