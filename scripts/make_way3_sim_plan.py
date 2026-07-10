#!/usr/bin/env python3
import json
import math
import re
import subprocess
from pathlib import Path

PLAN_PATH = Path.home() / "vtol-aam-rescue" / "ros2_ws" / "src" / "krac_control" / "src" / "way3.plan"
BACKUP_PATH = PLAN_PATH.with_suffix(".plan.before_sim_local")

ALT = 10.0

def get_current_gps():
    cmd = ["ros2", "topic", "echo", "/mavros/global_position/global", "--once"]
    out = subprocess.check_output(cmd, text=True)

    lat_m = re.search(r"latitude:\s*([-0-9.]+)", out)
    lon_m = re.search(r"longitude:\s*([-0-9.]+)", out)

    if not lat_m or not lon_m:
        raise RuntimeError("Failed to parse /mavros/global_position/global")

    return float(lat_m.group(1)), float(lon_m.group(1))

def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    earth_radius = 6378137.0
    lat_rad = math.radians(lat_deg)

    dlat = north_m / earth_radius
    dlon = east_m / (earth_radius * math.cos(lat_rad))

    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)

def set_item_latlon_alt(item, lat, lon, alt):
    # QGC .plan format: params[4]=lat, params[5]=lon, params[6]=alt
    if "params" in item and len(item["params"]) >= 7:
        item["params"][4] = lat
        item["params"][5] = lon
        item["params"][6] = alt

    if "coordinate" in item and isinstance(item["coordinate"], list) and len(item["coordinate"]) >= 3:
        item["coordinate"][0] = lat
        item["coordinate"][1] = lon
        item["coordinate"][2] = alt

def main():
    if not PLAN_PATH.exists():
        raise FileNotFoundError(f"Cannot find {PLAN_PATH}")

    home_lat, home_lon = get_current_gps()

    if not BACKUP_PATH.exists():
        BACKUP_PATH.write_text(PLAN_PATH.read_text())

    plan = json.loads(PLAN_PATH.read_text())
    mission = plan.get("mission", {})
    items = mission.get("items", [])

    if not items:
        raise RuntimeError("No mission items found")

    # current home 주변 20m 사각형 테스트 미션
    offsets = [
        (0.0, 0.0),      # VTOL takeoff
        (20.0, 0.0),     # 북쪽 20m
        (20.0, 20.0),    # 북동쪽
        (0.0, 20.0),     # 동쪽
        (0.0, 0.0),      # home 근처
        (10.0, -10.0),   # 추가 테스트점
    ]

    wp_i = 0

    for item in items:
        cmd = item.get("command")

        # 84 = MAV_CMD_NAV_VTOL_TAKEOFF
        # 16 = MAV_CMD_NAV_WAYPOINT
        if cmd in [84, 16]:
            north, east = offsets[min(wp_i, len(offsets) - 1)]
            lat, lon = offset_latlon(home_lat, home_lon, north, east)
            set_item_latlon_alt(item, lat, lon, ALT)
            wp_i += 1

        # 3000 = MAV_CMD_DO_VTOL_TRANSITION
        # 좌표 없는 command라 그대로 둠

    if "plannedHomePosition" in mission:
        mission["plannedHomePosition"] = [home_lat, home_lon, 0.0]

    PLAN_PATH.write_text(json.dumps(plan, indent=4))

    print("[OK] way3.plan updated")
    print(f"     home lat/lon = {home_lat}, {home_lon}")
    print(f"     altitude     = {ALT} m")
    print(f"     waypoints    = {wp_i}")
    print(f"     backup       = {BACKUP_PATH}")

if __name__ == "__main__":
    main()
