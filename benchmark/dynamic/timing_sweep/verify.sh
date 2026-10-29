#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=${REPO_ROOT}
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v >"$here/evidence/python_tests.log" 2>&1
tail -5 "$here/evidence/python_tests.log"
source "${MYDWA_WS}/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash"
ctest --test-dir ${MYDWA_WS}/build_compile_comparison_20261020/release -R '^test_' --output-on-failure --no-tests=error >"$here/evidence/release_functional_tests.log" 2>&1
tail -8 "$here/evidence/release_functional_tests.log"
python3 -m compileall -q "$here"
bash -n "$here/run.sh" "$here/launch_scene.sh"