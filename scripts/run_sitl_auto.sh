#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

echo "[0/8] Starting QGroundControl if available..."
if [ -f "$HOME/QGroundControl.AppImage" ]; then
    chmod +x "$HOME/QGroundControl.AppImage"
    "$HOME/QGroundControl.AppImage" >/tmp/qgc.log 2>&1 &
    echo "[QGC] Started in background."
else
    echo "[WARN] QGroundControl.AppImage not found at: $HOME/QGroundControl.AppImage"
fi

echo "[1/8] Starting PX4/Gazebo/MAVROS/Mission launch..."
cd "$REPO_DIR/ros2_ws"
ros2 launch krac_mission sitl_vtol.launch.py &
LAUNCH_PID=$!

cleanup() {
    echo ""
    echo "[CLEANUP] Stopping launch..."
    kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[2/8] Waiting for MAVROS connection..."
for i in {1..60}; do
    STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
    if echo "$STATE" | grep -q "connected: true"; then
        echo "[OK] MAVROS connected."
        break
    fi

    if [ "$i" -eq 60 ]; then
        echo "[ERROR] MAVROS did not connect."
        wait "$LAUNCH_PID"
        exit 1
    fi

    sleep 1
done

echo "[3/8] Waiting for mission upload..."
MISSION_OK=0
for i in {1..60}; do
    WPS="$(ros2 topic echo /mavros/mission/waypoints --once 2>/dev/null || true)"

    if echo "$WPS" | grep -q "waypoints:"; then
        COUNT="$(echo "$WPS" | grep -c "frame:" || true)"
        if [ "$COUNT" -gt 0 ]; then
            echo "[OK] Mission waypoints detected: $COUNT"
            MISSION_OK=1
            break
        fi
    fi

    echo "  waiting mission upload... $i/60"
    sleep 1
done

if [ "$MISSION_OK" -ne 1 ]; then
    echo "[WARN] Mission waypoints not detected."
    echo "       mission_loader may have failed or /mavros/mission/waypoints is not publishing."
    echo "       Continuing anyway..."
fi

echo "[4/8] Waiting for QGC / PX4 readiness..."
echo "      Watch PX4 log for: Ready for takeoff!"
sleep 8

echo "[5/8] Current MAVROS state:"
ros2 topic echo /mavros/state --once || true

echo "[6/8] Arming vehicle..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" || true

# 너무 오래 기다리면 auto preflight disarm이 걸릴 수 있으므로 짧게 대기
sleep 0.5

echo "[7/8] Sending AUTO.TAKEOFF immediately..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.TAKEOFF'}" || true

sleep 2

echo "[8/8] Final state:"
ros2 topic echo /mavros/state --once || true
ros2 topic echo /mavros/local_position/pose --once || true

echo ""
echo "[INFO] If the vehicle is still not armed:"
echo "  1. Make sure QGC is connected."
echo "  2. In PX4 shell, use:"
echo "     commander arm -f"
echo "     commander takeoff"
echo ""
echo "[INFO] Launch is still running. Press Ctrl+C to stop."

wait "$LAUNCH_PID"
