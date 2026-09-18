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

# Category/instance. Where each one stands is found on the counter, not fixed here: the runs are
# broken by the stove and the sink, and RoboCasa already stands appliances on them.
OBJECTS = [
    ("glass_cup/GlassCup002", "glass_cup"),
    ("jar/Jar012", "jar"),
    ("tupperware/Tupperware028", "tupperware"),
    ("measuring_cup/MeasuringCup001", "measuring_cup"),
    ("pot/Pot052", "pot"),
    ("fruit_bowl/FruitBowl001", "fruit_bowl"),
]

# Clear of the counter edge, of the appliances, and of each other; searched on a grid.
EDGE_MARGIN = 0.03
CLEARANCE = 0.03
GRID = 0.02


def dump_environment(env_name: str, layout: int, style: int, out: pathlib.Path) -> None:
    from robocasa.utils.env_utils import create_env

    env = create_env(env_name=env_name, layout_ids=[layout], style_ids=[style],
                     render_onscreen=False, seed=0)
    env.reset()

    # create_env always builds a robot into the arena; this world is the kitchen on its own, and
    # the robot it parks at (10, 10, 0) would be visible from inside the room.
    root = ET.fromstring(env.sim.model.get_xml())
    world = root.find("worldbody")
    robot = [b for b in world.findall("body")
             if (b.get("name") or "").startswith("robot0_")
             or (b.get("name") or "").endswith("_eef_target")]
    for b in robot:
        world.remove(b)

    # Actuators, sensors and the rest still drive the joints that went with those bodies, and
    # MuJoCo refuses a model whose actuator names a joint it cannot find. Matched against what
    # survives rather than against a name prefix: the robot contributes several of those
    # (robot0_, mobilebase0_, gripper0_), and missing one fails the load the same way.
    alive = {el.get("name") for el in root.iter() if el.get("name")}
    refs = ("joint", "joint1", "joint2", "jointinparent", "site", "site1", "site2",
            "body", "body1", "body2", "geom", "geom1", "geom2", "tendon", "objname")

    dangling = 0
    for section in ("actuator", "sensor", "tendon", "equality", "contact"):
        el = root.find(section)
        if el is None:
            continue
        for child in list(el):
            if any(child.get(r) and child.get(r) not in alive for r in refs):
                el.remove(child)
                dangling += 1
        if len(el) == 0:
            root.remove(el)

    # A keyframe stores one qpos per joint, so it no longer matches a model that lost some.
    for key in root.findall("keyframe"):
        root.remove(key)

    out.write_text(ET.tostring(root, encoding="unicode"))
    print(f"{out.name}: {env.sim.model.nbody} bodies, {env.sim.model.ngeom} geoms, "
          f"{env.sim.model.nmesh} meshes, {env.sim.model.ntex} textures")
    print(f"  stripped {len(robot)} robot bodies and {dangling} references to them")


def counter_slots(kitchen: pathlib.Path, footprints: list[tuple[float, float, float]]) -> list:
    """Where each footprint (half x, half y, height) stands: (x, y, surface z, turned) or None.

    Front row first and clear of what RoboCasa already stood there; `turned` is a quarter turn.
    """
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(kitchen))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # World-aligned box around every geom; what gets placed is appended so the next one avoids it.
    lo, hi = [], []
    for g in range(model.ngeom):
        rot = data.geom_xmat[g].reshape(3, 3)
        centre = data.geom_xpos[g] + rot @ model.geom_aabb[g, :3]
        half = np.abs(rot) @ model.geom_aabb[g, 3:]
        lo.append(centre - half)
        hi.append(centre + half)

    def blocked(box_lo, box_hi) -> bool:
        return bool(np.any(np.all(np.array(lo) < box_hi, axis=1)
                           & np.all(np.array(hi) > box_lo, axis=1)))

    runs = []
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, model.geom_matid[g]) or ""
        # The thin strips are the trim around a sink cutout, not somewhere a pot can stand.
        if "counter_top" in name and model.geom_size[g][1] >= 0.2:
            runs.append(g)
    runs.sort(key=lambda g: -(hi[g] - lo[g])[:2].max())

    def front_edge(g: int) -> tuple[int, float, float]:
        """The edge with nothing standing beyond it, as (axis, value, outward sign)."""
        top = hi[g][2]
        edges = []
        for axis in (0, 1):
            for edge, outward in ((lo[g][axis], -1.0), (hi[g][axis], 1.0)):
                probe_lo, probe_hi = lo[g].copy(), hi[g].copy()
                probe_lo[2], probe_hi[2] = top + 0.01, top + 0.3
                probe_lo[axis], probe_hi[axis] = sorted((edge, edge + outward * 0.3))
                length = (hi[g] - lo[g])[1 - axis]
                edges.append((blocked(probe_lo, probe_hi), -length, axis, edge, outward))
        _, _, axis, edge, outward = min(edges)
        return axis, edge, outward

    slots = []
    for hx, hy, height in footprints:
        slot = None
        for g in runs:
            top = hi[g][2]
            across, front, outward = front_edge(g)
            along = 1 - across
            for turned in (False, True):
                half = np.array([hy, hx, 0.0] if turned else [hx, hy, 0.0])
                depths = np.arange(EDGE_MARGIN + half[across],
                                   hi[g][across] - lo[g][across] - EDGE_MARGIN - half[across],
                                   GRID)
                spans = np.arange(lo[g][along] + EDGE_MARGIN + half[along],
                                  hi[g][along] - EDGE_MARGIN - half[along], GRID)
                for depth in depths:
                    for span in spans:
                        centre = np.zeros(3)
                        centre[along], centre[across] = span, front - outward * depth
                        box_lo, box_hi = centre - half, centre + half
                        box_lo[2], box_hi[2] = top + 0.005, top + height
                        if not blocked(box_lo, box_hi):
                            slot = (float(centre[0]), float(centre[1]), float(top), turned)
                            lo.append(box_lo)
                            hi.append(box_hi)
                            break
                    if slot:
                        break
                if slot:
                    break
            if slot:
                break
        slots.append(slot)
    return slots


def flatten_classes(src: ET.Element) -> None:
    """Writes each geom's default class onto it: pasted into another world, the class is lost."""
    classes: dict[str, dict[str, str]] = {}

    def walk(default: ET.Element, inherited: dict[str, str]) -> None:
        geom = default.find("geom")
        attrs = {**inherited, **(geom.attrib if geom is not None else {})}
        classes[default.get("class") or "main"] = attrs
        for child in default.findall("default"):
            walk(child, attrs)

    top = src.find("default")
    if top is not None:
        walk(top, {})
    for geom in src.iter("geom"):
        for key, value in classes.get(geom.attrib.pop("class", None) or "main", {}).items():
            geom.attrib.setdefault(key, value)


def floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def add_objects(assets: pathlib.Path, kitchen: pathlib.Path, out: pathlib.Path) -> int:
    root = ET.Element("mujoco", {"model": "kitchen with objects"})
    ET.SubElement(root, "include", {"file": kitchen.name})
    asset_out = ET.SubElement(root, "asset")
    world_out = ET.SubElement(root, "worldbody")

    bodies = []
    for rel, name in OBJECTS:
        d = assets / rel
        model = d / "model.xml"
        if not model.exists():
            print(f"  missing, skipped: {rel}")
            continue

        src = ET.parse(model).getroot()
        flatten_classes(src)

        # Paths are relative to the object's own directory, and the kitchen sets meshdir.
        for el in src.find("asset"):
            if "file" in el.attrib:
                el.attrib["file"] = str((d / el.attrib["file"]).resolve())
            asset_out.append(el)

        # Each object ships with its body called "object", so the names would collide.
        body = ET.Element("body", {"name": name})
        ET.SubElement(body, "freejoint")
        for geom in src.find(".//body[@name='object']").findall("geom"):
            # Geoms share one namespace across the model, and every object names one reg_bbox.
            if "name" in geom.attrib:
                geom.attrib["name"] = f"{name}_{geom.attrib['name']}"
            body.append(geom)

        # Every object carries a `reg_bbox` box around itself: its underside is the object's.
        box = body.find(f"geom[@name='{name}_reg_bbox']")
        if box is None:
            print(f"  no bounding box, so no way to stand it on anything; skipped: {name}")
            continue
        bodies.append((body, floats(box.get("pos", "0 0 0")), floats(box.get("size"))))

    slots = counter_slots(kitchen, [(half[0] + CLEARANCE, half[1] + CLEARANCE, 2 * half[2])
                                    for _, _, half in bodies])
    added = 0
    for (body, centre, half), slot in zip(bodies, slots):
        if slot is None:
            print(f"  no free counter space, skipped: {body.get('name')}")
            continue
        x, y, top, turned = slot
        if turned:
            body.set("quat", "0.7071068 0 0 0.7071068")
            centre = [-centre[1], centre[0], centre[2]]
        body.set("pos", f"{x - centre[0]:.4f} {y - centre[1]:.4f} "
                        f"{top - (centre[2] - half[2]):.4f}")
        world_out.append(body)
        added += 1

    ET.indent(root, "  ")
    out.write_text(ET.tostring(root, encoding="unicode"))
    return added


def vendor_assets(mjcf: pathlib.Path, assets_dir: pathlib.Path, package: pathlib.Path) -> int:
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

        # Under its package path: every fixture calls its mesh visual/model.obj.
        try:
            dest = assets_dir / src.resolve().relative_to(package)
        except ValueError:
            dest = assets_dir / src.parent.name / src.name
        # Meshes and textures never change, but the included kitchen.xml is regenerated every
        # run: keeping an older copy silently loads the world this build was meant to replace.
        if not dest.exists() or src.stat().st_mtime > dest.stat().st_mtime:
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

    package = (pathlib.Path(robocasa.__file__).parent / "models/assets").resolve()
    assets = package / "objects/lightwheel"
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
        copied = (vendor_assets(bare, world / "assets", package)
                  + vendor_assets(full, world / "assets", package))
        print(f"  vendored {copied} asset files into {world / 'assets'}")

    print(full)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
