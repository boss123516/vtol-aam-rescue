#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/vtol-aam-rescue"
PLAN="$ROOT/ros2_ws/src/krac_control/src/way3.plan"
BACKUP="$PLAN.backup_before_direct_rescue_$(date +%Y%m%d_%H%M%S)"

cd "$ROOT"

if [ ! -f "$PLAN" ]; then
  echo "[ERROR] plan file not found: $PLAN"
  exit 1
fi

cp "$PLAN" "$BACKUP"
echo "[OK] backup saved: $BACKUP"

python3 - <<'PY'
import json
from pathlib import Path

root = Path.home() / "vtol-aam-rescue"
plan_path = root / "ros2_ws/src/krac_control/src/way3.plan"

data = json.loads(plan_path.read_text())
items = data["mission"]["items"]
by_name = {it.get("amsr_name", ""): it for it in items}

required = ["TAKEOFF_HOME", "REP", "WP5_BACK", "WP4_BACK", "WP3_BACK", "WP2_BACK", "WP1_BACK", "HOME_RETURN"]
missing = [n for n in required if n not in by_name]
if missing:
    raise SystemExit(f"[ERROR] missing waypoints in plan: {missing}")

def clone_item(src_name, new_name=None, alt=None, hold=None):
    item = json.loads(json.dumps(by_name[src_name]))
    if new_name is not None:
        item["amsr_name"] = new_name
    if alt is not None:
        item["Altitude"] = float(alt)
        if "AMSLAltAboveTerrain" in item:
            item["AMSLAltAboveTerrain"] = float(alt)
        if len(item.get("params", [])) >= 7:
            item["params"][6] = float(alt)
    if hold is not None and len(item.get("params", [])) >= 1:
        # MAV_CMD_NAV_WAYPOINT param1 = hold time in seconds.
        # If the custom mission parser ignores this, true 3s wait must be added in FSM code.
        item["params"][0] = float(hold)
    item["autoContinue"] = True
    return item

rep_alt = by_name["REP"].get("Altitude", 10.0)

new_items = []
new_items.append(clone_item("TAKEOFF_HOME", "TAKEOFF_HOME_DIRECT_RESCUE", alt=rep_alt))
new_items.append(clone_item("REP", "REP_DIRECT", alt=rep_alt, hold=2.0))
new_items.append(clone_item("REP", "REP_DESCEND_LOW", alt=1.0, hold=2.0))
new_items.append(clone_item("REP", "REP_HOLD_3S_LOW", alt=1.0, hold=3.0))
new_items.append(clone_item("REP", "REP_TAKEOFF_AGAIN", alt=rep_alt, hold=2.0))
for name in ["WP5_BACK", "WP4_BACK", "WP3_BACK", "WP2_BACK", "WP1_BACK", "HOME_RETURN"]:
    new_items.append(clone_item(name, name))

for i, item in enumerate(new_items, start=1):
    item["doJumpId"] = i

# keep plannedHomePosition etc. only replace items
ndata = data
ndata["mission"]["items"] = new_items
ndata["mission"]["plannedHomePosition"] = data["mission"].get("plannedHomePosition", [47.3979712, 8.5461687, 10.0])
plan_path.write_text(json.dumps(ndata, indent=4, ensure_ascii=False))

print("[OK] wrote direct rescue plan:", plan_path)
for it in new_items:
    print(f"  {it['doJumpId']:02d} {it.get('amsr_name')} cmd={it.get('command')} alt={it.get('Altitude')} params={it.get('params')}")
PY

# Make sure auto_spawn uses repo models, not old ~/.gz/fuel/models.
python3 - <<'PY'
from pathlib import Path
p = Path.home() / "vtol-aam-rescue/scripts/auto_spawn.sh"
if p.exists():
    s = p.read_text()
    s2 = s.replace('MODEL_ROOT="$HOME/.gz/fuel/models"', 'MODEL_ROOT="$HOME/vtol-aam-rescue/px4_assets/gz/models"')
    if s2 != s:
        p.write_text(s2)
        print("[OK] fixed MODEL_ROOT in scripts/auto_spawn.sh")
    else:
        print("[OK] MODEL_ROOT already checked")
else:
    print("[WARN] scripts/auto_spawn.sh not found")
PY

chmod +x "$ROOT/scripts/auto_spawn.sh" 2>/dev/null || true

echo "[NEXT] launching original mission script..."
echo "[NOTE] If objects do not appear, open another terminal after Gazebo loads and run:"
echo "       cd ~/vtol-aam-rescue && ./scripts/auto_spawn.sh"

exec "$ROOT/scripts/run_mission_from_wp1.sh"
