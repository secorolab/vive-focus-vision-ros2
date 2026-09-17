#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu.
# See LICENSE for details.

"""Builds the scenes declared in scenes/*.yaml into the export cache.

Each scene is either a `model` — an MJCF handed to scene_export — or a `generator`, a script
that produces and exports its own world. Adding one is a new yaml file and nothing else.
"""

import argparse
import os
import pathlib
import subprocess
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent
CACHE = pathlib.Path(os.environ.get("VIVE_VR_CACHE", pathlib.Path.home() / ".cache/vive_vr_ros2"))


def export_dir(name: str) -> pathlib.Path:
    return CACHE / "scenes" / name


def build(scene: dict, scene_export: str, force: bool) -> bool:
    name = scene["name"]
    out = export_dir(name)

    if out.joinpath("scene.glb").exists() and not force:
        print(f"{name}: already in {out}")
        return True

    if "generator" in scene:
        # -o so a generator lands under its scene name too, rather than one of its own choosing.
        cmd = [str(ROOT / scene["generator"]), *map(str, scene.get("args", [])), "-o", str(out)]
        if force:
            cmd.append("--force")
    else:
        model = pathlib.Path(scene["model"]).expanduser()
        if not model.exists():
            print(f"{name}: no model at {model}", file=sys.stderr)
            return False
        cmd = [scene_export, str(model), "-o", str(out)]

    print(f"{name}: {' '.join(cmd)}")
    return subprocess.run(cmd, check=False).returncode == 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene-export", default="scene_export",
                   help="the scene_export binary; the CMake target passes the built one")
    p.add_argument("--only", action="append", help="build just this scene, repeatable")
    p.add_argument("--force", action="store_true")
    p.add_argument("--skip-heavy", action="store_true",
                   help="skip scenes marked heavy, which download gigabytes on first build")
    p.add_argument("--list", action="store_true")
    args = p.parse_args()

    scenes = [yaml.safe_load(f.read_text()) for f in sorted((ROOT / "scenes").glob("*.yaml"))]

    if args.list:
        for s in scenes:
            built = "built" if export_dir(s["name"]).joinpath("scene.glb").exists() else "-"
            print(f"{s['name']:10} {built:6} {' '.join(s['description'].split())}")
        return 0

    failed = []
    for s in scenes:
        if args.only and s["name"] not in args.only:
            continue
        if args.skip_heavy and s.get("heavy"):
            print(f"{s['name']}: skipped, heavy")
            continue
        if not build(s, args.scene_export, args.force):
            failed.append(s["name"])

    if failed:
        print(f"failed: {', '.join(failed)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
