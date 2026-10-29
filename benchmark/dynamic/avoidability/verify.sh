#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
source "${MYDWA_WS}/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash"
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=${REPO_ROOT}
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v > "$here/evidence/all_python_tests.log" 2>&1
ctest --test-dir ${MYDWA_WS}/build_compile_comparison_20261020/release -R '^test_' --output-on-failure --no-tests=error > "$here/evidence/release_ctest.log" 2>&1
python3 -m compileall -q "$here"
for script in "$here"/*.sh; do bash -n "$script"; done
git -C "$root" diff --check > "$here/evidence/diff_check.log"
tail -5 "$here/evidence/all_python_tests.log"
tail -5 "$here/evidence/release_ctest.log"