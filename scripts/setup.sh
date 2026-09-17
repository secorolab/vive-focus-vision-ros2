#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

set -euo pipefail

usage() {
    cat <<'EOF'
Prepares a fresh workspace. Run it from the workspace root, once:

  cd ~/work/p/vrws
  ./src/vive-vr-ros2/scripts/setup.sh

  --no-scenes    skip RoboCasa and its assets (no kitchen)
  --no-unity     skip the VIVE plugin (no APK builds)

It clones the workspace dependencies, makes venv/ with --system-site-packages, installs the
scene tooling into it and fetches the VIVE plugin. Then: source venv/bin/activate, source your
ROS distro, colcon build.
EOF
}

scenes=1
unity=1
for arg in "$@"; do
    case "$arg" in
        --no-scenes) scenes=0 ;;
        --no-unity)  unity=0 ;;
        -h|--help)   usage; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

workspace="$PWD"
repo="$workspace/src/vive-vr-ros2"
[ -f "$repo/package.xml" ] || {
    echo "run this from the workspace root, the directory holding src/vive-vr-ros2" >&2
    exit 1
}

if [ ! -d "$workspace/src/mj_kdl_wrapper" ]; then
    command -v vcs >/dev/null || { echo "vcs not found; apt install python3-vcstool" >&2; exit 1; }
    echo "==> cloning workspace dependencies"
    vcs import src < "$repo/dependencies.repos"
fi

venv="$workspace/venv"
if [ ! -x "$venv/bin/python" ]; then
    # Without system site packages the venv hides rclpy and launch, and nothing here runs in it.
    echo "==> creating $venv"
    python3 -m venv --system-site-packages "$venv"
fi

if [ "$scenes" -eq 1 ]; then
    if ! "$venv/bin/python" -c "import robocasa" 2>/dev/null; then
        echo "==> installing the scene tooling"
        "$venv/bin/pip" install -q "git+https://github.com/robocasa/robocasa.git"
        # RoboCasa pins an older robosuite than its own code needs.
        "$venv/bin/pip" install -q --force-reinstall --no-deps \
            "git+https://github.com/ARISE-Initiative/robosuite.git@master"
    fi

    rc="$("$venv/bin/python" -c 'import os, robocasa; print(os.path.dirname(robocasa.__file__))')"
    if [ ! -d "$rc/models/assets/objects/lightwheel" ]; then
        # "all" is ~10 GB of Objaverse and AI-generated sets that nothing here loads.
        echo "==> downloading the kitchen asset packs"
        yes | "$venv/bin/python" -m robocasa.scripts.download_kitchen_assets \
            --type tex fixtures_lw objs_lw
    fi
fi

if [ "$unity" -eq 1 ] && ! ls "$repo"/unity/VrRos/vendor/*.tgz >/dev/null 2>&1; then
    echo "==> fetching the VIVE OpenXR plugin"
    "$repo/scripts/fetch_vive_plugin.sh"
fi

cat <<EOF

ready. next:
  source $venv/bin/activate
  source /opt/ros/jazzy/setup.bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SCENES=ON
EOF
