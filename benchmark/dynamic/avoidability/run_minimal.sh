#!/usr/bin/env bash
set -uo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tag=${1:?Usage: run_minimal.sh NEW_UNIQUE_TAG}
[[ "$tag" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
for suffix in t2_early t4_early t4_late diagnostic_t2 diagnostic_t4; do
  [[ ! -e "$here/runs/${tag}_${suffix}" && ! -e "$here/evidence/${tag}_${suffix}.log" ]] || { echo 'Tag already used; choose a new tag.' >&2; exit 2; }
done
failed=0
for spec in '2 4.0 t2_early' '4 4.6 t4_early' '4 6.0 t4_late'; do
  read -r start brake label <<< "$spec"
  bash "$here/validate.sh" --start "$start" --brake "$brake" --run-id "${tag}_${label}" > "$here/evidence/${tag}_${label}.log" 2>&1
  status=$?; (( status==0 )) || failed=1
  printf '%s exit=%s\n' "$label" "$status"
done
for start in 2 4; do
  bash "$here/diagnostic.sh" --start-time "$start" --run-id "${tag}_diagnostic_t${start}" > "$here/evidence/${tag}_diagnostic_t${start}.log" 2>&1
  status=$?; (( status==0 )) || failed=1
  printf 'diagnostic_t%s exit=%s\n' "$start" "$status"
done
exit "$failed"
