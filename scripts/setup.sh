#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

set -euo pipefail

usage() {
    cat <<'EOF'
Prepares a fresh workspace, once. Create and activate a virtualenv first:

  python3 -m venv --system-site-packages venv && source venv/bin/activate
  ./src/vive-vr-ros2/scripts/setup.sh [WORKSPACE]

WORKSPACE is the directory holding src/vive-vr-ros2, and defaults to the current one.

  --no-scenes    skip RoboCasa and its assets (no kitchen)
  --no-unity     skip the VIVE plugin (no APK builds)

It clones the workspace dependencies, installs the scene tooling into the active virtualenv and
fetches the VIVE plugin. Then source your ROS distro and colcon build.
EOF
}

scenes=1
unity=1
workspace="$PWD"
for arg in "$@"; do
    case "$arg" in
        --no-scenes) scenes=0 ;;
        --no-unity)  unity=0 ;;
        -h|--help)   usage; exit 0 ;;
        -*) echo "unknown option: $arg" >&2; usage >&2; exit 2 ;;
        *) workspace="$arg" ;;
    esac
done

[ -d "$workspace" ] || { echo "no such directory: $workspace" >&2; exit 1; }
workspace="$(cd "$workspace" && pwd)"
repo="$workspace/src/vive-vr-ros2"
[ -f "$repo/package.xml" ] || {
    echo "$workspace does not hold src/vive-vr-ros2; pass the workspace path" >&2
    exit 1
}

if [ ! -d "$workspace/src/mj_kdl_wrapper" ]; then
    command -v vcs >/dev/null || { echo "vcs not found; apt install python3-vcstool" >&2; exit 1; }
    echo "==> cloning workspace dependencies"
    vcs import src < "$repo/dependencies.repos"
fi

if [ "$scenes" -eq 1 ]; then
    [ -n "${VIRTUAL_ENV:-}" ] || {
        echo "no virtualenv active; the scene tooling would land in the system python." >&2
        echo "  python3 -m venv --system-site-packages venv && source venv/bin/activate" >&2
        echo "--system-site-packages, or the venv hides rclpy and launch." >&2
        exit 1
    }

    if ! python3 -c "import robocasa" 2>/dev/null; then
        echo "==> installing the scene tooling into $VIRTUAL_ENV"
        pip install -q "git+https://github.com/robocasa/robocasa.git"
        # RoboCasa pins an older robosuite than its own code needs.
        pip install -q --force-reinstall --no-deps \
            "git+https://github.com/ARISE-Initiative/robosuite.git@master"
    fi

    rc="$(python3 -c 'import os, robocasa; print(os.path.dirname(robocasa.__file__))')"
    if [ ! -d "$rc/models/assets/objects/lightwheel" ]; then
        # "all" is ~10 GB of Objaverse and AI-generated sets that nothing here loads.
        echo "==> downloading the kitchen asset packs"
        yes | python3 -m robocasa.scripts.download_kitchen_assets \
            --type tex fixtures_lw objs_lw
    fi
fi

if [ "$unity" -eq 1 ] && ! ls "$repo"/unity/VrRos/vendor/*.tgz >/dev/null 2>&1; then
    echo "==> fetching the VIVE OpenXR plugin"
    "$repo/scripts/fetch_vive_plugin.py"
fi

cat <<EOF

ready. next:
  source /opt/ros/jazzy/setup.bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SCENES=ON
EOF
