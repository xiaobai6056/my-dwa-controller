#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$here/probe_build"
touch "$here/probe_build/AMENT_IGNORE"
includes=(-I"${REPO_ROOT}/include" -I/usr/include/eigen3)
while IFS= read -r directory; do includes+=(-I"$directory"); done < <(find /opt/ros/humble/include -mindepth 1 -maxdepth 1 -type d | sort)
g++ -std=c++17 -O2 -shared -fPIC -pthread "${includes[@]}" "$here/decision_probe.cpp" -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib -lnav2_costmap_2d_core -ldl -o "$here/probe_build/libdecision.so"
sha256sum "$here/probe_build/libdecision.so"