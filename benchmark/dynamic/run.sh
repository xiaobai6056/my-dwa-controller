#!/usr/bin/env bash
# From a clean WSL terminal: bash benchmark/dynamic/run.sh [suite arguments].

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
export TURTLEBOT3_MODEL=waffle
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 "$here/scenario.py" generate
exec python3 "$here/run_suite.py" "$@"