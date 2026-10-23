#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$here/probe_build"
touch "$here/probe_build/AMENT_IGNORE"
includes=(-I/home/user/ros2_ws/src/my_dwa_controller/include -I/usr/include/eigen3)
while IFS= read -r directory; do includes+=(-I"$directory"); done < <(find /opt/ros/humble/include -mindepth 1 -maxdepth 1 -type d | sort)
g++ -std=c++17 -O2 -shared -fPIC -pthread "${includes[@]}" "$here/decision_probe.cpp" -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib -lnav2_costmap_2d_core -ldl -o "$here/probe_build/libdecision.so"
sha256sum "$here/probe_build/libdecision.so"