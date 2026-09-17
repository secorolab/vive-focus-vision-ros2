#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

set -euo pipefail

usage() {
    cat <<'EOF'
Builds the RoboCasa kitchen and exports it for the headset.

  scripts/build_kitchen.sh                 build, then export to the cache
  scripts/build_kitchen.sh --force         rebuild the MJCF even if it exists
  scripts/build_kitchen.sh --env PnPCounterToCab --layout 3 --style 2

Idempotent: the venv, the ~4 GB asset download and the MJCF are each done once. Nothing lands
in /tmp, because rebuilding any of it needs RoboCasa and a network.
EOF
}

cache="${VIVE_VR_CACHE:-$HOME/.cache/vive_vr_ros2}"
venv="$cache/robocasa-venv"
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
out="${out:-$cache/exports/$name}"

mkdir -p "$worlds"

if [ ! -x "$venv/bin/python" ]; then
    echo "==> creating $venv"
    python3 -m venv "$venv"
fi

if ! "$venv/bin/python" -c "import robocasa" 2>/dev/null; then
    echo "==> installing RoboCasa"
    "$venv/bin/pip" install -q "git+https://github.com/robocasa/robocasa.git"
    # RoboCasa pins an older robosuite than its own code needs.
    "$venv/bin/pip" install -q --force-reinstall --no-deps \
        "git+https://github.com/ARISE-Initiative/robosuite.git@master"
fi

rc="$("$venv/bin/python" -c 'import os, robocasa; print(os.path.dirname(robocasa.__file__))')"
objects="$rc/models/assets/objects/lightwheel"

if [ ! -d "$objects" ] || [ -z "$(ls -A "$objects" 2>/dev/null)" ]; then
    echo "==> downloading kitchen assets (several GB, once)"
    "$venv/bin/python" -m robocasa.scripts.download_kitchen_assets
fi

if [ "$force" -eq 1 ] || [ ! -s "$mjcf" ]; then
    echo "==> building $env_name layout $layout style $style"
    "$venv/bin/python" "$here/scripts/make_kitchen.py" "$mjcf" \
        --env "$env_name" --layout "$layout" --style "$style"
    # Every RoboCasa fixture is welded, so without this there is nothing to pick up.
    python3 "$here/scripts/add_kitchen_objects.py" "$objects" "$mjcf" "$with_objects"
else
    echo "==> $mjcf exists, skipping (--force to rebuild)"
fi

if ! command -v ros2 >/dev/null; then
    echo "ros2 not on PATH; source the workspace to export the .glb" >&2
    echo "then: ros2 run vive_vr_ros2 scene_export $with_objects -o $out" >&2
    exit 1
fi

echo "==> exporting"
ros2 run vive_vr_ros2 scene_export "$with_objects" -o "$out"

cat <<EOF

kitchen ready:
  ros2 launch vive_vr_ros2 sim.launch.py model:=$with_objects scene_dir:=$out
EOF
