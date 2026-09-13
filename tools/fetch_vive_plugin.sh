#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.
#
# Fetches the VIVE OpenXR UPM tarball that Packages/manifest.json references by relative path.
# It is 361 MB and stays out of git, so a fresh clone needs this before Unity can open the
# project. Idempotent: an existing file with the right checksum is left alone.

set -euo pipefail

VERSION="${VIVE_OPENXR_VERSION:-2.5.1}"
SHA256="250c55f8cfd65c28c29f52ccb8b8864ebaa64f47665c74c50186ed5d388ccf80" # 2.5.1

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_dir="$repo_root/unity/VrRos/vendor"
tarball="$vendor_dir/com.htc.upm.vive.openxr-$VERSION.tgz"
url="https://github.com/ViveSoftware/VIVE-OpenXR-Unity/releases/download/versions%2F$VERSION/com.htc.upm.vive.openxr-$VERSION.tgz"

if [ -f "$tarball" ] && echo "$SHA256  $tarball" | sha256sum --check --status 2>/dev/null; then
    echo "VIVE OpenXR $VERSION already present: $tarball"
    exit 0
fi

mkdir -p "$vendor_dir"
echo "downloading VIVE OpenXR $VERSION (361 MB)..."
curl -fL --progress-bar -o "$tarball" "$url"

# The checksum is pinned for 2.5.1 only; a different version is expected not to match.
if [ "$VERSION" = "2.5.1" ]; then
    echo "$SHA256  $tarball" | sha256sum --check -
else
    echo "note: no pinned checksum for $VERSION; verify it yourself"
    sha256sum "$tarball"
fi

# UPM resolves by the path in manifest.json, so a non-default version needs that updated too.
if [ "$VERSION" != "2.5.1" ]; then
    echo "now point unity/VrRos/Packages/manifest.json at com.htc.upm.vive.openxr-$VERSION.tgz"
fi

echo "done: $tarball"
