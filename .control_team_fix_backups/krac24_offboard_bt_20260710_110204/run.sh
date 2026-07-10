#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros2_ws"

[[ -f "$WS/install/setup.bash" ]] || {
  echo "ERROR: 먼저 ./setup.sh를 실행하세요."
  exit 1
}

pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f "krac_bt_runner" 2>/dev/null || true
pkill -9 -f "rescue_controller_team.py" 2>/dev/null || true
pkill -9 -f "mavros_node" 2>/dev/null || true
sleep 1

export KRAC_ROOT="$ROOT"
exec "$ROOT/scripts/run_control_team_cycle.sh"
