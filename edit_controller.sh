#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FILE="$ROOT/ros2_ws/src/krac_control/src/rescue_controller_team.py"
if command -v code >/dev/null 2>&1; then exec code "$FILE"; fi
if command -v nano >/dev/null 2>&1; then exec nano "$FILE"; fi
echo "$FILE"
