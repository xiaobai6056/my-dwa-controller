#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=/home/user/ros2_ws/src/my_dwa_controller
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v >"$here/evidence/python_tests.log" 2>&1
tail -5 "$here/evidence/python_tests.log"
source /home/user/ros2_ws/install_compile_comparison_20261020/release/share/my_dwa_controller/local_setup.bash
ctest --test-dir /home/user/ros2_ws/build_compile_comparison_20261020/release -R '^test_' --output-on-failure --no-tests=error >"$here/evidence/release_functional_tests.log" 2>&1
tail -8 "$here/evidence/release_functional_tests.log"
python3 -m compileall -q "$here"
bash -n "$here/run.sh" "$here/launch_scene.sh"