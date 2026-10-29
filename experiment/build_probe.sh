#!/bin/bash
set -eo pipefail
if [ "$#" -ne 1 ]; then echo "Usage: $0 EXTERNAL_BUILD_DIRECTORY" >&2; exit 2; fi
exec bash "$(dirname "$0")/build.sh" "$1" probe
