#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$ROOT/ros2_ws/src/krac_control"
WS="$ROOT/ros2_ws"

FILES=(
  "$PKG/bt/krac_control_team_cycle_bt.xml"
  "$PKG/config/krac_bt_params_control_team.yaml"
  "$PKG/src/control_team_outbound.plan"
  "$PKG/src/control_team_return.plan"
  "$PKG/src/rescue_controller_team.py"
  "$ROOT/scripts/run_control_team_cycle.sh"
)
for f in "${FILES[@]}"; do
  [[ -f "$f" ]] || { echo "ERROR: missing $f"; exit 1; }
done

python3 -m py_compile "$PKG/src/rescue_controller_team.py"
bash -n "$ROOT/scripts/run_control_team_cycle.sh"
bash -n "$ROOT/setup.sh"
bash -n "$ROOT/run.sh"

python3 - "$PKG/bt/krac_control_team_cycle_bt.xml" "$PKG/src/control_team_outbound.plan" "$PKG/src/control_team_return.plan" <<'PY'
import json, sys, xml.etree.ElementTree as ET
from pathlib import Path
bt, out, ret = map(Path, sys.argv[1:4])
root = ET.parse(bt).getroot()
assert any(e.tag == 'BehaviorTree' and e.get('ID') == 'CONTROL_TEAM_CYCLE_BT' for e in root)
module = next(e for e in root if e.tag == 'BehaviorTree' and e.get('ID') == 'RescuePickupModule')
assert len(list(module.iter('ExecuteExternalRescueModule'))) == 1
op=json.loads(out.read_text()); rp=json.loads(ret.read_text())
assert [x['command'] for x in op['mission']['items']] == [84,16]
assert [x['command'] for x in rp['mission']['items']] == [16,85]
assert op['mission']['items'][1]['autoContinue'] is False
print('BT/XML/mission validation OK')
PY

if [[ -f "$WS/install/setup.bash" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  set -u
  PREFIX="$(ros2 pkg prefix krac_control)"
  [[ "$PREFIX" == "$WS/install/krac_control" ]] || { echo "ERROR: wrong overlay $PREFIX"; exit 2; }
fi

echo "[VERIFY] OK"
