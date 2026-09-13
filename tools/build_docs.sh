#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.
#
# Generates the HTML docs without configuring the ROS package, which CI would otherwise need
# ament, rclcpp, MuJoCo and the wrapper for. Same Doxyfile.in, so the two paths cannot drift.
#
# Usage: tools/build_docs.sh [OUTPUT_DIR]     (default: build/docs)

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_root="$repo_root/src/vr"
out_dir="${1:-$repo_root/build/docs}"
template="$package_root/docs/doxygen/Doxyfile.in"

# Below 1.16, doxygen's own div.fragment rule outranks the theme's .fragment and the pages
# render subtly wrong rather than failing.
DOXYGEN_MINIMUM="1.16.0"
DOXYGEN="${DOXYGEN:-doxygen}"

if ! command -v "$DOXYGEN" >/dev/null; then
    echo "doxygen not found. This project needs $DOXYGEN_MINIMUM or newer; the distro package" >&2
    echo "is too old. Download the official binary:" >&2
    echo "  https://github.com/doxygen/doxygen/releases  (doxygen-<version>.linux.bin.tar.gz)" >&2
    echo "then re-run with DOXYGEN=/path/to/bin/doxygen" >&2
    exit 1
fi

doxygen_version="$("$DOXYGEN" --version | awk '{print $1}')"
if [ "$(printf '%s\n%s\n' "$DOXYGEN_MINIMUM" "$doxygen_version" | sort -V | head -1)" \
     != "$DOXYGEN_MINIMUM" ]; then
    echo "doxygen $doxygen_version is too old; the theme needs $DOXYGEN_MINIMUM or newer." >&2
    echo "It would still generate, but with unpadded code blocks and a broken navigation tree." >&2
    echo "Download the official binary from https://github.com/doxygen/doxygen/releases and" >&2
    echo "re-run with DOXYGEN=/path/to/bin/doxygen" >&2
    exit 1
fi

version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$package_root/package.xml" | head -1)"

mkdir -p "$out_dir"
doxyfile="$out_dir/Doxyfile"

sed -e "s|@PROJECT_VERSION@|$version|g" \
    -e "s|@CMAKE_CURRENT_BINARY_DIR@|$out_dir|g" \
    -e "s|@VR_DOXYGEN_SOURCE_ROOT@|$package_root|g" \
    -e "s|@VR_DOXYGEN_DOCS_ROOT@|$repo_root|g" \
    "$template" > "$doxyfile"

# A broken ref or a page missing from INPUT is otherwise a silent hole in the published site.
warnings="$out_dir/doxygen-warnings.txt"
sed -i "s|^QUIET .*|QUIET = YES|; s|^WARN_LOGFILE .*||" "$doxyfile"
echo "WARN_LOGFILE = $warnings" >> "$doxyfile"

"$DOXYGEN" "$doxyfile"

if [ -s "$warnings" ]; then
    echo "doxygen reported warnings:" >&2
    cat "$warnings" >&2
    exit 1
fi

echo "docs written to $out_dir/docs/html/index.html (version $version)"
