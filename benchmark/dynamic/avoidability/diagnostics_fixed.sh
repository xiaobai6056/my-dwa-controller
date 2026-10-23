#!/usr/bin/env bash
set -uo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash "$here/diagnostic.sh" --start-time 2 --run-id diagnostic_t2_linked > "$here/evidence/diagnostic_t2_linked.log" 2>&1
printf 'diagnostic_t2_linked exit=%s\n' "$?"
bash "$here/diagnostic.sh" --start-time 4 --run-id diagnostic_t4_linked > "$here/evidence/diagnostic_t4_linked.log" 2>&1
printf 'diagnostic_t4_linked exit=%s\n' "$?"