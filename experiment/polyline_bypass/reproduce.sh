#!/bin/bash
set -eo pipefail
if [ "$#" -ne 1 ]; then echo "Usage: $0 NEW_OUTPUT_DIRECTORY (activate ROS2 and controller install prefix first)" >&2; exit 2; fi
repo=$(cd "$(dirname "$0")/../.." && pwd)
out=$(realpath -m "$1")
case "$out/" in "$repo/"*) echo "Output directory must be outside the repository" >&2; exit 2;; esac
if [ -e "$out" ]; then echo "Output directory already exists: $out" >&2; exit 2; fi
mkdir -p "$out"
bash "$repo/experiment/build.sh" "$out/build" rolling
for scenario in A0 A1 A2; do
  mkdir "$out/$scenario"
  mode=B
  if [ "$scenario" = A0 ]; then mode=A; fi
  "$out/build/rolling" "$repo/experiment/polyline_bypass/inputs/$scenario.json" "$out/$scenario" "$mode" 4
done
