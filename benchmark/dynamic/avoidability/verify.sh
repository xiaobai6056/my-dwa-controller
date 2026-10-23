#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/ros2_ws/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=/home/user/ros2_ws/src/my_dwa_controller
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v > "$here/evidence/all_python_tests.log" 2>&1
ctest --test-dir /home/user/ros2_ws/build_compile_comparison_20261020/release -R '^test_' --output-on-failure --no-tests=error > "$here/evidence/release_ctest.log" 2>&1
python3 -m compileall -q "$here"
for script in "$here"/*.sh; do bash -n "$script"; done
git -C "$root" diff --check > "$here/evidence/diff_check.log"
tail -5 "$here/evidence/all_python_tests.log"
tail -5 "$here/evidence/release_ctest.log"