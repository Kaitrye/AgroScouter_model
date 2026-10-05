#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ -n "${BLENDER_BIN:-}" ]]; then
  blender_bin="$BLENDER_BIN"
elif command -v blender >/dev/null 2>&1; then
  blender_bin="$(command -v blender)"
else
  blender_bin=""
  for candidate in "$HOME"/Загрузки/blender-*-linux-x64/blender "$HOME"/Downloads/blender-*-linux-x64/blender; do
    if [[ -x "$candidate" ]]; then
      blender_bin="$candidate"
      break
    fi
  done
fi
if [[ -z "$blender_bin" ]]; then
  echo 'Blender not found. Set BLENDER_BIN to the Blender executable.' >&2
  exit 1
fi

"$blender_bin" -b camera_mount.blend -noaudio --python tools/export_camera_mount.py
python3 tools/generate_model.py
"$blender_bin" -b robot_visual_edit.blend -noaudio --python tools/sync_camera_mount_blend.py
python3 tools/prepare_phobos_import.py
python3 tools/validate_assets.py
