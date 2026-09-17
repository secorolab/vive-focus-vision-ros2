#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Adds graspable RoboCasa objects to a generated kitchen.

Every fixture in a RoboCasa kitchen is welded, so nothing in one can be picked up. Each object
ships as its own MJCF with a body called "object", so they cannot simply be included - the names
would collide. Their asset names are already namespaced, so only the body needs renaming, and a
freejoint is what makes it graspable.
"""

import argparse
import pathlib
import xml.etree.ElementTree as ET

# Category/instance, and where it stands on the counter's near edge.
WANTED = [
    ("glass_cup/GlassCup002", "glass_cup", 2.05, -0.55),
    ("jar/Jar012", "jar", 2.35, -0.55),
    ("tupperware/Tupperware001", "tupperware", 2.65, -0.55),
    ("measuring_cup/MeasuringCup001", "measuring_cup", 2.95, -0.55),
    ("pot/Pot052", "pot", 3.25, -0.55),
    ("fruit_bowl/FruitBowl001", "fruit_bowl", 3.55, -0.55),
]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=pathlib.Path,
                   help=".../robocasa/models/assets/objects/lightwheel")
    p.add_argument("kitchen", type=pathlib.Path, help="MJCF from make_kitchen.py")
    p.add_argument("out", type=pathlib.Path)
    args = p.parse_args()

    root = ET.Element("mujoco", {"model": "kitchen with real objects"})
    ET.SubElement(root, "include", {"file": args.kitchen.name})
    asset_out = ET.SubElement(root, "asset")
    world_out = ET.SubElement(root, "worldbody")

    added = 0
    for rel, name, x, y in WANTED:
        d = args.assets / rel
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

        body = ET.SubElement(world_out, "body", {"name": name, "pos": f"{x} {y} 1.05"})
        ET.SubElement(body, "freejoint")
        for geom in src.find(".//body[@name='object']").findall("geom"):
            # The classes belong to the object's own defaults, which are not carried over.
            geom.attrib.pop("class", None)
            # Collision meshes have no material: keep them out of sight but still colliding.
            geom.attrib["group"] = "1" if "material" in geom.attrib else "3"
            body.append(geom)
        added += 1
        print(f"  added {name} from {rel}")

    ET.indent(root, "  ")
    args.out.write_text(ET.tostring(root, encoding="unicode"))
    print(f"wrote {args.out} with {added} objects")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
