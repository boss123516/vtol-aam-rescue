#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble not found: /opt/ros/humble/setup.bash"
  exit 1
fi

if [[ ! -d "$WS/src/krac_control" ]]; then
  echo "ERROR: krac_control source missing: $WS/src/krac_control"
  exit 2
fi

if [[ ! -d "$WS/src/krac_interfaces" ]]; then
  echo "ERROR: krac_interfaces source missing: $WS/src/krac_interfaces"
  echo "This package is required by krac_control."
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WS"

echo "[1/5] Removing stale package build/install state..."
rm -rf \
  build/krac_control \
  build/krac_interfaces \
  install/krac_control \
  install/krac_interfaces

echo "[2/5] Checking package discovery..."
colcon list | grep -E '^(krac_interfaces|krac_control)[[:space:]]'

echo "[3/5] Building krac_control and all workspace dependencies..."
colcon build \
  --symlink-install \
  --packages-up-to krac_control \
  --event-handlers console_direct+

set +u
source "$WS/install/setup.bash"
set -u

echo "[4/5] Checking installed packages..."
ros2 pkg prefix krac_interfaces
ros2 pkg prefix krac_control

echo "[5/5] Running project verification..."
cd "$ROOT"
./verify.sh

echo
echo "[SETUP] READY"
echo
echo "Run:"
echo "  cd '$ROOT'"
echo "  ./run.sh"
