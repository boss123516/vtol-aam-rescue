#!/usr/bin/env bash
set -eo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -d "$PX4_DIR" ]; then
  echo "ERROR: PX4_DIR not found: $PX4_DIR"
  echo "Set PX4_DIR=/path/to/PX4-Autopilot and retry."
  exit 1
fi

mkdir -p "$PX4_DIR/Tools/simulation/gz/models"
mkdir -p "$PX4_DIR/Tools/simulation/gz/worlds"
mkdir -p "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"

cp -R "$REPO_DIR/px4_assets/gz/models/amsr_vtol" "$PX4_DIR/Tools/simulation/gz/models/"
cp "$REPO_DIR/px4_assets/gz/worlds/my_world.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/"
cp "$REPO_DIR/px4_assets/airframes/1984_gz_amsr_vtol" "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/"

# PX4's stock standard_vtol has no image camera topic. The BT/Yolo SITL path
# uses standard_vtol for stable FW transition, so apply a minimal camera
# payload patch that merges model://mono_cam onto base_link.
if [ -f "$REPO_DIR/px4_assets/model_patches/standard_vtol/model.sdf" ]; then
  cp "$REPO_DIR/px4_assets/model_patches/standard_vtol/model.sdf" \
    "$PX4_DIR/Tools/simulation/gz/models/standard_vtol/model.sdf"
fi

# Optional marker/environment models for Gazebo spawn service.
# GZ_SIM_RESOURCE_PATH (see ~/.bashrc) only includes
# $PX4_DIR/Tools/simulation/gz/models, so model:// mesh URIs inside these
# models only resolve if copied there. ~/.gz/fuel/models is NOT on that path;
# copying only there (the previous behavior) left model:// mesh references
# unresolved for any model not also placed under $PX4_DIR manually.
mkdir -p "$HOME/.gz/fuel/models"
for m in v_marker land_marker box victim survivor_tray; do
  if [ -d "$REPO_DIR/px4_assets/gz/models/$m" ]; then
    cp -R "$REPO_DIR/px4_assets/gz/models/$m" "$PX4_DIR/Tools/simulation/gz/models/"
    cp -R "$REPO_DIR/px4_assets/gz/models/$m" "$HOME/.gz/fuel/models/"
  fi
done

CMAKE="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
if [ -f "$CMAKE" ] && ! grep -q "1984_gz_amsr_vtol" "$CMAKE"; then
  cp "$CMAKE" "$CMAKE.bak.$(date +%Y%m%d_%H%M%S)"
  python3 - <<PY
from pathlib import Path
p=Path('$CMAKE')
s=p.read_text()
needle='4004_gz_standard_vtol'
if needle in s:
    s=s.replace(needle, needle+'\n\t1984_gz_amsr_vtol', 1)
else:
    s=s.replace('px4_add_romfs_files(\n', 'px4_add_romfs_files(\n\t1984_gz_amsr_vtol\n', 1)
p.write_text(s)
PY
fi

echo "PX4 assets applied to: $PX4_DIR"
echo "Next: cd $PX4_DIR && make px4_sitl gz_amsr_vtol"
