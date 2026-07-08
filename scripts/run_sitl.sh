#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

START_PX4="${START_PX4:-true}"
START_MAVROS="${START_MAVROS:-true}"
START_FSM="${START_FSM:-true}"
START_MISSION_LOADER="${START_MISSION_LOADER:-true}"
START_LOGGER="${START_LOGGER:-true}"
VEHICLE_MODEL="${VEHICLE_MODEL:-gz_standard_vtol}"

echo "[run_sitl] vehicle_model=${VEHICLE_MODEL}"
echo "[run_sitl] start_px4=${START_PX4} start_mavros=${START_MAVROS} start_fsm=${START_FSM} start_mission_loader=${START_MISSION_LOADER} start_logger=${START_LOGGER}"

ros2 launch krac_mission sitl_vtol.launch.py \
  vehicle_model:="${VEHICLE_MODEL}" \
  start_px4:="${START_PX4}" \
  start_mavros:="${START_MAVROS}" \
  start_fsm:="${START_FSM}" \
  start_mission_loader:="${START_MISSION_LOADER}" \
  start_logger:="${START_LOGGER}"
