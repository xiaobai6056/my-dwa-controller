#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=/home/user/ros2_ws/src/my_dwa_controller
python3 -m unittest discover -s "$root/benchmark" -p 'test_*.py' -v >"$here/evidence/python_tests_final.log" 2>&1
tail -5 "$here/evidence/python_tests_final.log"
for variant in unoptimized release; do
  (
    source "/home/user/ros2_ws/install_compile_comparison_20261020/$variant/share/my_dwa_controller/local_setup.bash"
    ros2 pkg prefix my_dwa_controller
    ctest --test-dir "/home/user/ros2_ws/build_compile_comparison_20261020/$variant" -R '^test_' --output-on-failure --no-tests=error
  ) >"$here/evidence/${variant}_functional_tests_final.log" 2>&1
  tail -8 "$here/evidence/${variant}_functional_tests_final.log"
done
python3 -m compileall -q "$here"
bash -n "$here/build.sh" "$here/launch_scene.sh"
python3 "$here/audit.py"
git -C "$root" diff --check