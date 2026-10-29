#!/bin/bash
# Source ROS2 and a compatible my_dwa_controller installation before invoking.
set -eo pipefail
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: $0 EXTERNAL_BUILD_DIRECTORY [rolling|probe]" >&2; exit 2
fi
repo=$(cd "$(dirname "$0")/.." && pwd)
build=$(realpath -m "$1")
case "$build/" in "$repo/"*) echo "Build directory must be outside the repository" >&2; exit 2;; esac
case "${2:-rolling}" in
  rolling) source_dir="$repo/experiment/rolling";;
  probe) source_dir="$repo/experiment";;
  *) echo "Expected rolling or probe" >&2; exit 2;;
esac
cmake -S "$source_dir" -B "$build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build" -j2
ctest --test-dir "$build" --output-on-failure
