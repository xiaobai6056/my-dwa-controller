#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"/benchmark
sha256sum -c evidence/unchanged.sha256
python3 run_suite.py --controllers mydwa --scenes S04 --repeats 3 \
  --run-id tuning_r1_diagnostic_baseline_20261019
