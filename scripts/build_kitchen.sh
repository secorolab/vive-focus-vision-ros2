#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

set -euo pipefail

usage() {
    cat <<'EOF'
Builds the RoboCasa kitchen and exports it for the headset.

  scripts/build_kitchen.sh                 build, then export
  scripts/build_kitchen.sh --force         rebuild the MJCF even if it exists
  scripts/build_kitchen.sh --env PnPCounterToCab --layout 3 --style 2

Needs robocasa importable by python3 and a sourced workspace. The MJCF and the export are
each done once.
EOF
}

cache="${VIVE_VR_CACHE:-$HOME/.cache/vive_vr_ros2}"
worlds="$cache/worlds"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

env_name=NavigateKitchen
layout=1
style=1
force=0
out=""
while [ $# -gt 0 ]; do
    case "$1" in
        --env)    env_name="$2"; shift 2 ;;
        --layout) layout="$2"; shift 2 ;;
        --style)  style="$2"; shift 2 ;;
        -o|--out) out="$2"; shift 2 ;;
        --force)  force=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

name="kitchen_${env_name}_l${layout}_s${style}"
mjcf="$worlds/$name.xml"
with_objects="$worlds/${name}_objects.xml"
out="${out:-$cache/scenes/$name}"

if ! python3 -c "import robocasa" 2>/dev/null; then
    echo "robocasa is not importable; activate the environment that has it" >&2
    exit 1
fi
if ! command -v ros2 >/dev/null; then
    echo "ros2 is not on PATH; source the workspace" >&2
    exit 1
fi

rc="$(python3 -c 'import os, robocasa; print(os.path.dirname(robocasa.__file__))')"
objects="$rc/models/assets/objects/lightwheel"

if [ ! -d "$objects" ] || [ -z "$(ls -A "$objects" 2>/dev/null)" ]; then
    echo "RoboCasa assets are missing. Download the kitchen packs once:" >&2
    echo "  python3 -m robocasa.scripts.download_kitchen_assets --type tex fixtures_lw objs_lw" >&2
    exit 1
fi

mkdir -p "$worlds"

if [ "$force" -eq 1 ] || [ ! -s "$mjcf" ]; then
    echo "==> building $env_name layout $layout style $style"
    python3 "$here/scripts/make_kitchen.py" "$mjcf" \
        --env "$env_name" --layout "$layout" --style "$style"
    # Every RoboCasa fixture is welded, so without this there is nothing to pick up.
    python3 "$here/scripts/add_kitchen_objects.py" "$objects" "$mjcf" "$with_objects"
else
    echo "==> $mjcf exists, skipping (--force to rebuild)"
fi

echo "==> exporting"
ros2 run vive_vr_ros2 scene_export "$with_objects" -o "$out"

cat <<EOF

kitchen ready:
  ros2 launch vive_vr_ros2 sim.launch.py model:=$with_objects scene_dir:=$out
EOF
