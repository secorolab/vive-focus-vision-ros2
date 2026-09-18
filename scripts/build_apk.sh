#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

set -euo pipefail

usage() {
    cat <<'EOF'
Source to headset in one command.

  scripts/build_apk.sh                             build the APK
  scripts/build_apk.sh --setup                     rewrite the scene and player settings first
  scripts/build_apk.sh --install                   also install and launch on the headset
  scripts/build_apk.sh --setup --install --logcat  and tail the client's log

UNITY overrides the editor path. VR_HOST seeds the address baked into the APK, which is
optional: the client discovers the PC when the configured address fails.
EOF
}

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project="$here/unity/VrRos"
apk="$project/Build/VrRos.apk"
package="de.uni_bremen.secoro.vrros"
old_package="sh.vamsi.vrros"

UNITY="${UNITY:-$HOME/Unity/Hub/Editor/6000.0.83f1/Editor/Unity}"

setup=0
install=0
logcat=0
for arg in "$@"; do
    case "$arg" in
        --setup)   setup=1 ;;
        --install) install=1 ;;
        --logcat)  install=1; logcat=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

[ -x "$UNITY" ] || { echo "no Unity editor at $UNITY; set UNITY=<path>" >&2; exit 1; }

# The plugin is a 361 MB tarball kept out of git, and Unity cannot open the project without it.
if ! ls "$project"/vendor/*.tgz >/dev/null 2>&1; then
    echo "VIVE plugin missing; run scripts/fetch_vive_plugin.py" >&2
    exit 1
fi

lockfile="$project/Temp/UnityLockfile"
if [ -e "$lockfile" ] && fuser "$lockfile" >/dev/null 2>&1; then
    echo "the Unity editor has this project open; close it first" >&2
    exit 1
fi

run_unity() {
    local method="$1" log
    log="$(mktemp -t vrros-"${method##*.}"-XXXX.log)"
    echo "==> $method"
    if ! "$UNITY" -batchmode -quit -nographics -projectPath "$project" \
                  -buildTarget Android -executeMethod "$method" -logFile "$log"; then
        echo "$method failed; last lines of $log:" >&2
        tail -30 "$log" >&2
        exit 1
    fi
    # The trailing space matters: without it this also catches Unity's stack-trace frames.
    grep -E "VrRosSetup: |error CS" "$log" || true
}

# Before the build rather than after it: the build is minutes long, and finding out at the end
# that there is nothing to install to throws all of it away.
require_device() {
    command -v adb >/dev/null || {
        echo "adb not found; apt install android-tools-adb" >&2
        exit 1
    }
    if [ -z "$(adb devices | awk 'NR>1 && $2=="device"')" ]; then
        echo "no device: plug into the headset's RIGHT-side USB-C port and accept the prompt" >&2
        echo "wireless: adb tcpip 5555 over USB once, then adb connect <headset-ip>:5555" >&2
        exit 1
    fi
}

[ "$install" -eq 1 ] && require_device

[ "$setup" -eq 1 ] && run_unity VrRosSetup.SetupAll
run_unity VrRosSetup.BuildApk
ls -lh "$apk"

[ "$install" -eq 1 ] || exit 0

# The transport can drop during a build that takes minutes, so confirm it is still there.
require_device

# A different application id is a different application: the old one is joined, not replaced.
if adb shell pm list packages | tr -d '\r' | grep -qx "package:$old_package"; then
    echo "warning: $old_package is still installed and will not be replaced by this build." >&2
    echo "         remove it with: adb uninstall $old_package" >&2
fi

echo "==> installing"
adb install -r "$apk"
adb shell monkey -p "$package" -c android.intent.category.LAUNCHER 1 >/dev/null
echo "launched $package"

[ "$logcat" -eq 1 ] && exec adb logcat -s Unity
exit 0
