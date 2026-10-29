#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"
python3 -m unittest discover -s benchmark -p 'test_*.py' -v
python3 -m unittest discover -s ${MYDWA_WS}/scripts/tests -p 'test_*.py' -v
ament_flake8
ament_pep257 --exclude benchmark/results benchmark/evidence
sha256sum -c benchmark/evidence/unchanged.sha256
bash -n benchmark/start_scene.sh
bash -n ${MYDWA_WS}/scripts/start_headless_baseline.sh
bash -n ${MYDWA_WS}/scripts/start_ui_baseline.sh
python3 benchmark/evidence/validate_v1.py benchmark/results/v1/static_v1_20261019
