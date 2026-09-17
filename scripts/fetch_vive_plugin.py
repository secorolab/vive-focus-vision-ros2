#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Fetches the VIVE OpenXR UPM tarball that Packages/manifest.json references by relative path.

361 MB, kept out of git, so a fresh clone needs this before Unity can open the project. An
existing file with the right checksum is left alone.
"""

import argparse
import hashlib
import pathlib
import sys
import urllib.error
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
VENDOR = ROOT / "unity/VrRos/vendor"

CHECKSUMS = {
    "2.5.1": "250c55f8cfd65c28c29f52ccb8b8864ebaa64f47665c74c50186ed5d388ccf80",
}
DEFAULT_VERSION = "2.5.1"

URL = ("https://github.com/ViveSoftware/VIVE-OpenXR-Unity/releases/download/"
       "versions%2F{v}/com.htc.upm.vive.openxr-{v}.tgz")


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url: str, out: pathlib.Path) -> None:
    with urllib.request.urlopen(url) as response, out.open("wb") as f:
        total = int(response.headers.get("content-length", 0))
        done = 0
        while block := response.read(1 << 20):
            f.write(block)
            done += len(block)
            if total and sys.stderr.isatty():
                print(f"\r  {done / 1e6:.0f} / {total / 1e6:.0f} MB", end="", file=sys.stderr)
    if total and sys.stderr.isatty():
        print(file=sys.stderr)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--version", default=DEFAULT_VERSION)
    args = p.parse_args()

    expected = CHECKSUMS.get(args.version)
    tarball = VENDOR / f"com.htc.upm.vive.openxr-{args.version}.tgz"

    if tarball.exists() and expected and sha256(tarball) == expected:
        print(f"VIVE OpenXR {args.version} already present: {tarball}")
        return 0

    VENDOR.mkdir(parents=True, exist_ok=True)
    print(f"downloading VIVE OpenXR {args.version} (361 MB)...")
    try:
        download(URL.format(v=args.version), tarball)
    except urllib.error.URLError as exc:
        reason = getattr(exc, "code", None) or exc.reason
        print(f"download failed ({reason}); is {args.version} a released version?",
              file=sys.stderr)
        tarball.unlink(missing_ok=True)
        return 1

    actual = sha256(tarball)
    if expected is None:
        print(f"no pinned checksum for {args.version}; verify it yourself:\n  {actual}")
        print(f"and point unity/VrRos/Packages/manifest.json at {tarball.name}")
    elif actual != expected:
        print(f"checksum mismatch:\n  expected {expected}\n  got      {actual}", file=sys.stderr)
        tarball.unlink()
        return 1

    print(f"done: {tarball}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
