#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"
sha256sum -c benchmark/evidence/unchanged.sha256
bash -n benchmark/tuning_r1/run_diagnostic_baseline.sh
bash -n benchmark/tuning_r1/run_probe.sh
ament_flake8
ament_pep257 --exclude benchmark/results benchmark/evidence benchmark/tuning_r1/evidence
cd "${MYDWA_WS}"
ROS_DOMAIN_ID=97 colcon test --packages-select my_dwa_controller --event-handlers console_direct+
colcon test-result --test-result-base build/my_dwa_controller/test_results --verbose
