#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"
python3 -m compileall -q benchmark/dynamic benchmark/test_dynamic.py
bash -n benchmark/dynamic/start_scene.sh benchmark/dynamic/run.sh
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
python3 -m unittest discover -s ${MYDWA_WS}/scripts/tests -v