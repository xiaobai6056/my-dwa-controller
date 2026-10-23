#!/usr/bin/env bash
# From a clean WSL terminal: bash benchmark/dynamic/run.sh [suite arguments].
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 "$here/scenario.py" generate
exec python3 "$here/run_suite.py" "$@"