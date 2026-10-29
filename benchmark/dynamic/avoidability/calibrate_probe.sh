#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
release="${MYDWA_WS}/install_compile_comparison_20261020/release/lib"
export LD_LIBRARY_PATH="$release:$LD_LIBRARY_PATH"
includes=(-I"${REPO_ROOT}/include" -I/usr/include/eigen3)
while IFS= read -r directory; do includes+=(-I"$directory"); done < <(find /opt/ros/humble/include -mindepth 1 -maxdepth 1 -type d | sort)
g++ -std=c++17 -O2 -shared -fPIC -pthread "${includes[@]}" "$here/probe_fixture.cpp" -L"$release" -L/opt/ros/humble/lib -lmy_dwa_controller -lnav2_costmap_2d_core -lrclcpp -o "$here/probe_build/libfixture.so"
g++ -std=c++17 -O2 "${includes[@]}" "$here/probe_fixture_main.cpp" -L"$here/probe_build" -Wl,-rpath,"$here/probe_build" -lfixture -o "$here/probe_build/fixture"
"$here/probe_build/fixture"
mkdir -p "$here/evidence/calibration"
DECISION_DIR="$here/evidence/calibration" NAVPERF_MYDWA_LIB="$here/probe_build/libfixture.so" LD_PRELOAD="$here/probe_build/libdecision.so" "$here/probe_build/fixture"