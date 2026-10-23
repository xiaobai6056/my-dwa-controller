#!/usr/bin/env bash
set -uo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Every attempt has its own folder. Failure never overwrites or deletes a previous attempt.
bash "$here/validate.sh" --start 4 --brake 4.6 --run-id validation_t4_early > "$here/evidence/validation_t4_early.log" 2>&1
printf 'validation_t4_early exit=%s\n' "$?"
bash "$here/validate.sh" --start 4 --brake 6.0 --run-id validation_t4_late > "$here/evidence/validation_t4_late.log" 2>&1
printf 'validation_t4_late exit=%s\n' "$?"
bash "$here/diagnostic.sh" --start-time 2 --run-id diagnostic_t2 > "$here/evidence/diagnostic_t2.log" 2>&1
printf 'diagnostic_t2 exit=%s\n' "$?"
bash "$here/diagnostic.sh" --start-time 4 --run-id diagnostic_t4 > "$here/evidence/diagnostic_t4.log" 2>&1
printf 'diagnostic_t4 exit=%s\n' "$?"