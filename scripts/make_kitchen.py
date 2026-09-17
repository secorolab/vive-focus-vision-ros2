#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Builds a RoboCasa kitchen as one MJCF, with objects that can be picked up.

Two steps. The arena alone is walls and floor: fixtures are merged in when an environment is
constructed, which is also where the asset renaming happens that lets dozens of models coexist,
so a whole environment is built and its model written out. Then objects are added, because every
fixture in a RoboCasa kitchen is welded and a world with nothing graspable is no use here.

Prints the MJCF path on the last line; build_scenes.py exports it.
"""

import argparse
import os
import pathlib
import shutil
import sys
import xml.etree.ElementTree as ET

# Category/instance, and where it stands on the counter's near edge.
OBJECTS = [
    ("glass_cup/GlassCup002", "glass_cup", 2.05, -0.55),
    ("jar/Jar012", "jar", 2.35, -0.55),
    ("tupperware/Tupperware001", "tupperware", 2.65, -0.55),
    ("measuring_cup/MeasuringCup001", "measuring_cup", 2.95, -0.55),
    ("pot/Pot052", "pot", 3.25, -0.55),
    ("fruit_bowl/FruitBowl001", "fruit_bowl", 3.55, -0.55),
]


def dump_environment(env_name: str, layout: int, style: int, out: pathlib.Path) -> None:
    from robocasa.utils.env_utils import create_env

    env = create_env(env_name=env_name, layout_ids=[layout], style_ids=[style],
                     render_onscreen=False, seed=0)
    env.reset()
    out.write_text(env.sim.model.get_xml())
    print(f"{out.name}: {env.sim.model.nbody} bodies, {env.sim.model.ngeom} geoms, "
          f"{env.sim.model.nmesh} meshes, {env.sim.model.ntex} textures")


def add_objects(assets: pathlib.Path, kitchen: pathlib.Path, out: pathlib.Path) -> int:
    root = ET.Element("mujoco", {"model": "kitchen with objects"})
    ET.SubElement(root, "include", {"file": kitchen.name})
    asset_out = ET.SubElement(root, "asset")
    world_out = ET.SubElement(root, "worldbody")

    added = 0
    for rel, name, x, y in OBJECTS:
        d = assets / rel
        model = d / "model.xml"
        if not model.exists():
            print(f"  missing, skipped: {rel}")
            continue

        src = ET.parse(model).getroot()

        # Paths are relative to the object's own directory, and the kitchen sets meshdir.
        for el in src.find("asset"):
            if "file" in el.attrib:
                el.attrib["file"] = str((d / el.attrib["file"]).resolve())
            asset_out.append(el)

        # Each object ships with its body called "object", so the names would collide.
        body = ET.SubElement(world_out, "body", {"name": name, "pos": f"{x} {y} 1.05"})
        ET.SubElement(body, "freejoint")
        for geom in src.find(".//body[@name='object']").findall("geom"):
            # The classes belong to the object's own defaults, which are not carried over.
            geom.attrib.pop("class", None)
            # Collision meshes have no material: keep them out of sight but still colliding.
            geom.attrib["group"] = "1" if "material" in geom.attrib else "3"
            body.append(geom)
        added += 1

    ET.indent(root, "  ")
    out.write_text(ET.tostring(root, encoding="unicode"))
    return added


def vendor_assets(mjcf: pathlib.Path, assets_dir: pathlib.Path) -> int:
    """Copies every file an MJCF references next to it, and rewrites the paths to match.

    RoboCasa's meshes live in site-packages, so a generated world stops loading the moment the
    package is upgraded or removed. Copying them makes the world stand on its own.
    """
    tree = ET.parse(mjcf)
    root = tree.getroot()

    compiler = root.find("compiler")
    search = [mjcf.parent]
    for attr in ("meshdir", "texturedir", "assetdir"):
        if compiler is not None and compiler.get(attr):
            # Relative dirs are relative to the MJCF, not to wherever this is run from.
            search.insert(0, (mjcf.parent / compiler.get(attr)).expanduser())
            del compiler.attrib[attr]

    copied = 0
    for el in root.iter():
        src_attr = el.get("file")
        if not src_attr:
            continue
        candidates = [pathlib.Path(src_attr)] if pathlib.Path(src_attr).is_absolute() else [
            d / src_attr for d in search
        ]
        src = next((c for c in candidates if c.is_file()), None)
        if src is None:
            print(f"  missing asset, left as-is: {src_attr}", file=sys.stderr)
            continue

        # The same basename appears under several object directories, so the parent is kept.
        dest = assets_dir / src.parent.name / src.name
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
            copied += 1
        el.set("file", str(dest.relative_to(mjcf.parent)))

    tree.write(mjcf, encoding="unicode")
    return copied


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--env", default="NavigateKitchen", help="RoboCasa environment name")
    p.add_argument("--layout", type=int, default=1)
    p.add_argument("--style", type=int, default=1)
    p.add_argument("--worlds", type=pathlib.Path,
                   default=pathlib.Path(os.environ.get(
                       "VIVE_VR_CACHE", pathlib.Path.home() / ".cache/vive_vr_ros2")) / "worlds")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()

    try:
        import robocasa
    except ImportError:
        print("robocasa is not importable; activate the environment that has it", file=sys.stderr)
        return 1

    assets = pathlib.Path(robocasa.__file__).parent / "models/assets/objects/lightwheel"
    if not assets.is_dir() or not any(assets.iterdir()):
        print("RoboCasa assets are missing. Download the kitchen packs once:", file=sys.stderr)
        print("  python3 -m robocasa.scripts.download_kitchen_assets "
              "--type tex fixtures_lw objs_lw", file=sys.stderr)
        return 1

    world = args.worlds / f"kitchen_{args.env}_l{args.layout}_s{args.style}"
    bare = world / "kitchen.xml"
    full = world / "scene.xml"

    if args.force or not full.exists():
        world.mkdir(parents=True, exist_ok=True)
        dump_environment(args.env, args.layout, args.style, bare)
        print(f"  added {add_objects(assets, bare, full)} objects")
        # Both halves: the fixtures come from the dump, the objects from add_objects.
        copied = vendor_assets(bare, world / "assets") + vendor_assets(full, world / "assets")
        print(f"  vendored {copied} asset files into {world / 'assets'}")

    print(full)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
