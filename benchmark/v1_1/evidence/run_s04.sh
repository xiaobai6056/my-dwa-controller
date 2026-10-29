#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"/benchmark
sha256sum ../src/controller.cpp ../test/test_controller.cpp ../config/nav2_params.yaml ../config/nav2_params_dwb.yaml ${MYDWA_WS}/install/my_dwa_controller/lib/libmy_dwa_controller.so > v1_1/evidence/tested_artifacts.sha256
cp ${MYDWA_WS}/install/my_dwa_controller/lib/libmy_dwa_controller.so v1_1/evidence/libmy_dwa_controller.so.tested
for attempt in 001 002 003 004 005; do
  python3 run_suite.py --controllers mydwa --scenes S04 --repeats 1 --run-id v1_1_S04_${attempt}
  sha256sum -c v1_1/evidence/tested_artifacts.sha256
  python3 v1_1/evidence/check_s04.py results/v1/v1_1_S04_${attempt}
done
