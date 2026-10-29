#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=${REPO_ROOT}
work=${MYDWA_WS}
mkdir -p "$work/build_compile_comparison_20261020" "$work/install_compile_comparison_20261020"
touch "$work/build_compile_comparison_20261020/COLCON_IGNORE" "$work/install_compile_comparison_20261020/COLCON_IGNORE"
variants=("$@")
if [[ ${#variants[@]} -eq 0 ]]; then variants=(unoptimized release); fi
for variant in "${variants[@]}"; do
  build="$work/build_compile_comparison_20261020/$variant"
  install="$work/install_compile_comparison_20261020/$variant"
  [[ ! -e "$build" && ! -e "$install" ]] || { echo "Refusing to overwrite $variant" >&2; exit 1; }
  if [[ "$variant" == release ]]; then kind=Release; else kind=''; fi
  cmake -S "$root" -B "$build" -DCMAKE_INSTALL_PREFIX="$install" \
    -DCMAKE_BUILD_TYPE="$kind" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON \
    >"$here/evidence/${variant}_configure.log" 2>&1
  cmake --build "$build" --parallel 2 --verbose >"$here/evidence/${variant}_build.log" 2>&1
  cmake --install "$build" >"$here/evidence/${variant}_install.log" 2>&1
  cp "$build/compile_commands.json" "$here/evidence/${variant}_compile_commands.json"
  cp "$build/CMakeCache.txt" "$here/evidence/${variant}_CMakeCache.txt"
  (
    source "$install/share/my_dwa_controller/local_setup.bash"
    ctest --test-dir "$build" -R '^test_' --output-on-failure
  ) >"$here/evidence/${variant}_functional_tests.log" 2>&1
  echo "$variant build, install, functional tests passed"
done