#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu.
# See LICENSE for details.

"""Builds the scenes declared in scenes/*.yaml into the export cache.

Each scene is either a `model` — an MJCF handed to scene_export — or a `generator`, a script
that produces and exports its own world. Adding one is a new yaml file and nothing else.
"""

import argparse
import json
import os
import pathlib
import subprocess
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent
CACHE = pathlib.Path(os.environ.get("VIVE_VR_CACHE", pathlib.Path.home() / ".cache/vive_vr_ros2"))


def export_dir(name: str) -> pathlib.Path:
    return CACHE / "scenes" / name


def model_for(scene: dict, force: bool) -> pathlib.Path | None:
    """The MJCF to export: declared, or produced by a generator that prints its path."""
    if "model" in scene:
        model = pathlib.Path(scene["model"]).expanduser()
        return model if model.exists() else None

    cmd = [str(ROOT / scene["generator"]), *map(str, scene.get("args", []))]
    if force:
        cmd.append("--force")
    print(f"  {' '.join(cmd)}")
    run = subprocess.run(cmd, check=False, text=True, capture_output=True)
    sys.stdout.write(run.stdout)
    sys.stderr.write(run.stderr)
    if run.returncode != 0:
        return None
    return pathlib.Path(run.stdout.strip().splitlines()[-1])


def build(scene: dict, scene_export: str, force: bool) -> bool:
    name = scene["name"]
    out = export_dir(name)

    if out.joinpath("scene.glb").exists() and not force:
        print(f"{name}: already in {out}")
        return True

    print(f"{name}:")
    model = model_for(scene, force)
    if model is None or not model.exists():
        print(f"{name}: no model to export", file=sys.stderr)
        return False

    cmd = [scene_export, str(model), "-o", str(out)]
    if scene.get("groups"):
        cmd += ["--groups", str(scene["groups"])]
    if subprocess.run(cmd, check=False).returncode != 0:
        return False

    write_spawn(scene, out)
    return True


def write_spawn(scene: dict, out: pathlib.Path) -> None:
    """Puts the scene's spawn point into its manifest, which the scene message carries verbatim.

    Where to stand is a property of the world, not of the headset: the client's own
    spawnPosition cannot be right for two worlds at once, and is the fallback when a scene
    declares none.
    """
    spawn = scene.get("spawn")
    if not spawn:
        return

    manifest = out / "manifest.json"
    data = json.loads(manifest.read_text())
    data["spawn"] = {"xyz": [float(v) for v in spawn["xyz"]],
                     "yaw_deg": float(spawn.get("yaw_deg", 0.0))}
    manifest.write_text(json.dumps(data, indent=2))
    print(f"  spawn {data['spawn']['xyz']} yaw {data['spawn']['yaw_deg']} written to manifest")


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
