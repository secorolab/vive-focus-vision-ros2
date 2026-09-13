#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.
#
# Generates the HTML documentation without configuring the ROS package.
#
# The CMake `docs` target needs the whole build configured first -- ament, rclcpp, MuJoCo, the
# wrapper -- which is the right thing on a developer machine and far too much for CI that only
# wants the docs. This substitutes the same Doxyfile.in by hand and runs doxygen, so the two
# paths cannot drift: there is still only one Doxyfile template.
#
# Usage: tools/build_docs.sh [OUTPUT_DIR]     (default: build/docs)

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_root="$repo_root/src/vr"
out_dir="${1:-$repo_root/build/docs}"
template="$package_root/docs/doxygen/Doxyfile.in"

if ! command -v doxygen >/dev/null; then
    echo "doxygen is not installed: sudo apt install doxygen" >&2
    exit 1
fi

# The version is declared once, in package.xml; nothing else should carry a copy of it.
version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$package_root/package.xml" | head -1)"

mkdir -p "$out_dir"
doxyfile="$out_dir/Doxyfile"

sed -e "s|@PROJECT_VERSION@|$version|g" \
    -e "s|@CMAKE_CURRENT_BINARY_DIR@|$out_dir|g" \
    -e "s|@VR_DOXYGEN_SOURCE_ROOT@|$package_root|g" \
    -e "s|@VR_DOXYGEN_DOCS_ROOT@|$repo_root|g" \
    "$template" > "$doxyfile"

# Warnings are worth failing on: a broken @ref or a page missing from INPUT is a silent hole in
# the published site otherwise.
warnings="$out_dir/doxygen-warnings.txt"
sed -i "s|^QUIET .*|QUIET = YES|; s|^WARN_LOGFILE .*||" "$doxyfile"
echo "WARN_LOGFILE = $warnings" >> "$doxyfile"

doxygen "$doxyfile"

if [ -s "$warnings" ]; then
    echo "doxygen reported warnings:" >&2
    cat "$warnings" >&2
    exit 1
fi

echo "docs written to $out_dir/docs/html/index.html (version $version)"
