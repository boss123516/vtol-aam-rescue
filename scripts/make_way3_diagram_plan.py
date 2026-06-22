#!/usr/bin/env python3
import copy
import json
import math
import re
import subprocess
from pathlib import Path

PLAN_PATH = Path.home() / "vtol-aam-rescue" / "ros2_ws" / "src" / "krac_control" / "src" / "way3.plan"
BACKUP_PATH = PLAN_PATH.with_suffix(".plan.before_wp1_wp5_rep_route")

ALT = 10.0

# 전체 경로 크기 조절
# 1.0이면 아래 meter 값 그대로
# 너무 크면 0.5로 줄여도 됨
SCALE = 1.0


def get_current_gps():
    cmd = ["ros2", "topic", "echo", "/mavros/global_position/global", "--once"]
    out = subprocess.check_output(cmd, text=True)

    lat_m = re.search(r"latitude:\s*([-0-9.]+)", out)
    lon_m = re.search(r"longitude:\s*([-0-9.]+)", out)

    if not lat_m or not lon_m:
        raise RuntimeError("Failed to parse /mavros/global_position/global. Is MAVROS running?")

    return float(lat_m.group(1)), float(lon_m.group(1))


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    earth_radius = 6378137.0
    lat_rad = math.radians(lat_deg)

    north_m *= SCALE
    east_m *= SCALE

    dlat = north_m / earth_radius
    dlon = east_m / (earth_radius * math.cos(lat_rad))

    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def set_item_latlon_alt(item, lat, lon, alt):
    # QGC .plan:
    # params[4] = latitude
    # params[5] = longitude
    # params[6] = altitude
    if "params" in item and isinstance(item["params"], list) and len(item["params"]) >= 7:
        item["params"][4] = lat
        item["params"][5] = lon
        item["params"][6] = alt

    if "coordinate" in item and isinstance(item["coordinate"], list) and len(item["coordinate"]) >= 3:
        item["coordinate"][0] = lat
        item["coordinate"][1] = lon
        item["coordinate"][2] = alt

    if "Altitude" in item:
        item["Altitude"] = alt


def set_common_fields(item, seq_id, command):
    item["command"] = command

    # 원래 기존 mission이 frame=3이었으므로 GLOBAL_RELATIVE_ALT로 통일
    # frame 6은 QGC/PX4 해석이 헷갈릴 수 있어서 테스트 단계에서는 피함
    item["frame"] = 3

    item["autoContinue"] = True

    if "doJumpId" in item:
        item["doJumpId"] = seq_id

    if "type" in item:
        item["type"] = "SimpleItem"


def main():
    if not PLAN_PATH.exists():
        raise FileNotFoundError(f"Cannot find plan file: {PLAN_PATH}")

    home_lat, home_lon = get_current_gps()

    if not BACKUP_PATH.exists():
        BACKUP_PATH.write_text(PLAN_PATH.read_text())

    plan = json.loads(PLAN_PATH.read_text())
    mission = plan.get("mission", {})
    old_items = mission.get("items", [])

    if not old_items:
        raise RuntimeError("No mission items found in current way3.plan")

    takeoff_template = None
    waypoint_template = None

    for item in old_items:
        if item.get("command") == 84 and takeoff_template is None:
            takeoff_template = copy.deepcopy(item)
        if item.get("command") == 16 and waypoint_template is None:
            waypoint_template = copy.deepcopy(item)

    if takeoff_template is None:
        raise RuntimeError("No VTOL TAKEOFF item(command 84) found in current plan")

    if waypoint_template is None:
        raise RuntimeError("No NAV WAYPOINT item(command 16) found in current plan")

    # 그림 기준 route
    # N = north meter, E = east meter
    #
    # HOME → WP1 → WP2 → WP3 → WP4 → WP5 → REP
    #      → WP5 → WP4 → WP3 → WP2 → WP1 → HOME
    #
    # 좌표는 그림 배치 느낌으로 잡은 테스트용 local 좌표임.
    # 실제 대회장 절대좌표가 아니라 Gazebo home 주변 상대좌표.
    route = [
        ("TAKEOFF_HOME", 0.0,   0.0,   84),

        ("WP1",          25.0,   0.0,   16),
        ("WP2",          85.0,  70.0,   16),
        ("WP3",         145.0, 210.0,   16),
        ("WP4",         145.0, -20.0,   16),
        ("WP5",          95.0,  45.0,   16),
        ("REP",          35.0, 130.0,   16),

        ("WP5_BACK",     95.0,  45.0,   16),
        ("WP4_BACK",    145.0, -20.0,   16),
        ("WP3_BACK",    145.0, 210.0,   16),
        ("WP2_BACK",     85.0,  70.0,   16),
        ("WP1_BACK",     25.0,   0.0,   16),
        ("HOME_RETURN",   0.0,   0.0,   16),
    ]

    new_items = []

    for idx, (name, north, east, command) in enumerate(route, start=1):
        item = copy.deepcopy(takeoff_template if command == 84 else waypoint_template)

        lat, lon = offset_latlon(home_lat, home_lon, north, east)

        set_common_fields(item, seq_id=idx, command=command)
        set_item_latlon_alt(item, lat, lon, ALT)

        item["amsr_name"] = name
        item["amsr_local_offset_ne_m"] = [north * SCALE, east * SCALE]

        new_items.append(item)

    mission["items"] = new_items
    mission["plannedHomePosition"] = [home_lat, home_lon, 0.0]

    plan["mission"] = mission
    PLAN_PATH.write_text(json.dumps(plan, indent=4))

    print("[OK] Rebuilt way3.plan from WP1-WP5-REP route")
    print(f"     plan file: {PLAN_PATH}")
    print(f"     backup   : {BACKUP_PATH}")
    print(f"     home GPS : lat={home_lat}, lon={home_lon}")
    print(f"     altitude : {ALT} m")
    print(f"     scale    : {SCALE}")
    print("")
    print("Route:")
    for i, (name, north, east, command) in enumerate(route):
        print(f"  {i:02d} {name:14s} cmd={command} N={north*SCALE:7.1f}m E={east*SCALE:7.1f}m")


if __name__ == "__main__":
    main()
