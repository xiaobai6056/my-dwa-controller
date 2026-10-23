#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/ros2_ws/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$here/run_diagnostic.py" --controller mydwa --headless --preload "$here/probe_build/libdecision.so" "$@"