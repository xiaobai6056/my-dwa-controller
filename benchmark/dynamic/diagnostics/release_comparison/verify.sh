#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=${REPO_ROOT}
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v >"$here/evidence/python_tests_final.log" 2>&1
tail -5 "$here/evidence/python_tests_final.log"
for variant in unoptimized release; do
  (
    source "${MYDWA_WS}/install_compile_comparison_20261020/$variant/share/my_dwa_controller/local_setup.bash"
    ros2 pkg prefix my_dwa_controller
    ctest --test-dir "${MYDWA_WS}/build_compile_comparison_20261020/$variant" -R '^test_' --output-on-failure --no-tests=error
  ) >"$here/evidence/${variant}_functional_tests_final.log" 2>&1
  tail -8 "$here/evidence/${variant}_functional_tests_final.log"
done
python3 -m compileall -q "$here"
bash -n "$here/build.sh" "$here/launch_scene.sh"
python3 "$here/audit.py"
git -C "$root" diff --check