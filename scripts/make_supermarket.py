#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Builds a five-aisle supermarket with a TIAGo as one self-contained MJCF; prints its path."""

import argparse
import math
import os
import pathlib
import shutil
import sys
import tarfile
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import yaml

CACHE = pathlib.Path(os.environ.get("VIVE_VR_CACHE", pathlib.Path.home() / ".cache/vive_vr_ros2"))
MENAGERIE = pathlib.Path(os.environ.get(
    "MJ_KDL_MENAGERIE", pathlib.Path.home() / ".cache/mj_kdl_wrapper/menagerie"))

REPO = "shubhamt2897/mujoco-supermarket-sim-asset-dc"
COMMIT = "e5b7bd02b1b14fce51685c80f40cbc012a98cca9"

# Raised off z = 0, where the client draws its own ground, or the two z-fight.
FLOOR_Z = 0.005

AISLES = 5
AISLE_WIDTH, SHELF_DEPTH, BAYS = 1.6, 0.47, 4
AISLE_Y = [-0.65 + (k - AISLES // 2) * (AISLE_WIDTH + 2 * SHELF_DEPTH) for k in range(AISLES)]
RUN_HALF = BAYS * 1.25 / 2

ROOM_X = (-5.2, 7.5)
ROOM_Y = (AISLE_Y[0] - AISLE_WIDTH / 2 - SHELF_DEPTH, AISLE_Y[-1] + AISLE_WIDTH / 2 + SHELF_DEPTH)
CEILING = 3.2
DOOR_Y = (-1.9, 0.6)
CHECKOUT_X, CHECKOUT_YS = 5.9, (1.9, 3.7)
PRODUCE_X, PRODUCE_YS = 4.4, (-6.0, -5.1, -4.2, -3.3)

# Five aisles is an odd count, so closing the loop means one U-turn at the front of aisle 0.
FRONT_X, BACK_X = RUN_HALF + 0.7, -RUN_HALF - 0.95
PATROL = []
for _k, _y in enumerate(AISLE_Y):
    _ends = (FRONT_X, BACK_X) if _k % 2 == 0 else (BACK_X, FRONT_X)
    PATROL += [(_ends[0], _y), (_ends[1], _y)]
PATROL.append((BACK_X, AISLE_Y[0]))
SPAWN = (FRONT_X, AISLE_Y[0], math.pi)

# Heavy and round stock low, cartons high, the way a bay is blocked; one tuple per deck.
DECK_PLANS = (
    (("canned_food", "can"), ("jam", "yogurt", "canned_food"),
     ("cereal", "boxed_food", "milk"), ("boxed_drink", "cereal", "boxed_food")),
    (("can", "canned_food"), ("yogurt", "jam", "milk"),
     ("boxed_food", "cereal"), ("milk", "boxed_drink", "cereal")),
    (("canned_food", "jam"), ("can", "yogurt"),
     ("boxed_drink", "milk"), ("cereal", "boxed_food")),
)

MATERIALS = {
    "store_floor": dict(rgba="0.62 0.60 0.56 1", roughness="0.45"),
    "store_wall": dict(rgba="0.84 0.82 0.77 1", roughness="0.9"),
    "store_band": dict(rgba="0.09 0.36 0.23 1", roughness="0.6"),
    "store_ceiling": dict(rgba="0.74 0.74 0.73 1", roughness="0.95"),
    "light_panel": dict(rgba="1 1 0.98 1", roughness="0.3"),
    "fridge_body": dict(rgba="0.80 0.81 0.82 1", roughness="0.4", metallic="0.2"),
    "fridge_glass": dict(rgba="0.16 0.22 0.26 1", roughness="0.15"),
    "fridge_frame": dict(rgba="0.25 0.26 0.28 1", roughness="0.4", metallic="0.7"),
    "counter_body": dict(rgba="0.30 0.31 0.34 1", roughness="0.5"),
    "counter_top": dict(rgba="0.72 0.72 0.70 1", roughness="0.3", metallic="0.3"),
    "belt": dict(rgba="0.07 0.07 0.07 1", roughness="0.8"),
    "crate": dict(rgba="0.55 0.40 0.24 1", roughness="0.85"),
    "fruit_apple": dict(rgba="0.62 0.08 0.07 1", roughness="0.35"),
    "fruit_orange": dict(rgba="0.93 0.50 0.08 1", roughness="0.6"),
    "fruit_lime": dict(rgba="0.38 0.62 0.12 1", roughness="0.45"),
    "fruit_lemon": dict(rgba="0.95 0.83 0.20 1", roughness="0.5"),
}

# Tucked, from the Menagerie model's own `home` keyframe.
ARM_HOME = {"arm_1_joint": 0.20, "arm_2_joint": -1.34, "arm_3_joint": -0.20,
            "arm_4_joint": 1.94, "arm_5_joint": -1.57, "arm_6_joint": 1.37}
GRAVCOMP_PREFIXES = ("arm_", "gripper_", "head_")


def fetch(downloads: pathlib.Path) -> pathlib.Path:
    d = downloads / f"mujoco-supermarket-{COMMIT[:12]}"
    if (d / "out/scene.xml").exists():
        return d
    url = f"https://codeload.github.com/{REPO}/tar.gz/{COMMIT}"
    print(f"  downloading {REPO}@{COMMIT[:12]}", file=sys.stderr)
    d.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url) as r, tarfile.open(fileobj=r, mode="r|gz") as tar:
        for member in tar:
            parts = pathlib.PurePosixPath(member.name).parts[1:]
            if not parts:
                continue
            member.name = str(pathlib.PurePosixPath(*parts))
            tar.extract(member, d, filter="data")
    return d


def quat_z(yaw: float) -> str:
    return f"{math.cos(yaw / 2):.7f} 0 0 {math.sin(yaw / 2):.7f}"


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def copy_into(src: pathlib.Path, dest: pathlib.Path) -> None:
    if not dest.exists() or src.stat().st_mtime > dest.stat().st_mtime:
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)


def build_aisle(repo: pathlib.Path, out: pathlib.Path) -> None:
    """The upstream generator, with every deck stocked and no restocking gap."""
    import dataclasses
    import importlib.util

    spec = importlib.util.spec_from_file_location("dual_arm_scene", repo / "scene.py")
    ds = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = ds
    spec.loader.exec_module(ds)

    class Config(ds.SceneConfig):
        @property
        def runs(self):
            runs = []
            for k, y in enumerate(AISLE_Y):
                for side, (face, yaw) in enumerate(((y + AISLE_WIDTH / 2, 0.0),
                                                    (y - AISLE_WIDTH / 2, math.pi))):
                    plan = DECK_PLANS[(2 * k + side) % len(DECK_PLANS)]
                    runs.append(ds.ShelfRun(f"a{k}{'ns'[side]}", (0.0, face), yaw, plan,
                                            dynamic_front_row=False, rows_deep=2, gap=None))
            return tuple(runs)

    shelf = dataclasses.replace(ds.ShelfDims(), stocked_decks=(0, 1, 2, 3), n_bays=BAYS,
                                aisle_width=AISLE_WIDTH)
    result = ds.build_scene(Config(shelf=shelf), out)
    print(f"  {len(result.placements)} products stocked", file=sys.stderr)


def load_store(repo: pathlib.Path, world: pathlib.Path) -> ET.Element:
    """The aisle, less its roll cage, tower mount and lights, with its assets copied in."""
    aisle = repo / "out/vive_vr_aisle.xml"
    build_aisle(repo, aisle)
    root = ET.parse(aisle).getroot()
    compiler = root.find("compiler")
    src = (aisle.parent / compiler.get("meshdir")).resolve()
    compiler.set("meshdir", "assets")
    compiler.set("texturedir", "assets")
    for el in root.find("asset"):
        if "file" in el.attrib:
            copy_into(src / el.get("file"), world / "assets/products" / el.get("file"))
            el.set("file", f"products/{el.get('file')}")

    world_body = root.find("worldbody")
    for el in list(world_body):
        name = el.get("name", "")
        if el.tag in ("light", "camera") or name in ("roll_cage", "tower_base_site") \
                or name.startswith("cage_"):
            world_body.remove(el)
    world_body.remove(world_body.find("geom[@name='floor']"))
    # One body per product made a >1 MB manifest, which rosbridge fragments and the client drops.
    stock = world_body.find("body[@name='static_stock']")
    for product in list(stock.findall("body")):
        stock.remove(product)
        pos, quat = product.get("pos", "0 0 0"), product.get("quat", "1 0 0 0")
        for geom in product.findall("geom"):
            if geom.get("name", "").endswith("_coll"):
                continue
            geom.set("pos", pos)
            geom.set("quat", quat)
            stock.append(geom)
    ET.SubElement(root, "size", {"memory": "64M"})
    for mat in root.find("asset").findall("material"):
        if mat.get("name") in ("shelf_metal", "shelf_dark", "price_rail"):
            mat.attrib.pop("specular", None)
            mat.attrib.pop("shininess", None)
            mat.set("roughness", "0.45")
            mat.set("metallic", "0.15")
            if mat.get("name") == "shelf_metal":
                mat.set("rgba", "0.83 0.83 0.84 1")
    add_room(root)
    return root


def box(parent: ET.Element, name: str, pos, size, material: str, collide: bool = True) -> None:
    ET.SubElement(parent, "geom", {
        "name": name, "type": "box", "material": material,
        "pos": " ".join(f"{v:.4f}" for v in pos), "size": " ".join(f"{v:.4f}" for v in size),
        "contype": "1" if collide else "0", "conaffinity": "2" if collide else "0",
        "group": "0" if collide else "2"})


def add_room(root: ET.Element) -> None:
    """Walls, ceiling, light panels, chilled cabinets, produce and checkouts round the aisle."""
    asset, world = root.find("asset"), root.find("worldbody")
    ET.SubElement(asset, "texture", {"name": "store_tiles", "type": "2d", "builtin": "checker",
                                     "rgb1": "0.58 0.56 0.52", "rgb2": "0.53 0.51 0.47",
                                     "width": "512", "height": "512"})
    for name, attrs in MATERIALS.items():
        ET.SubElement(asset, "material", {"name": name, **attrs})
    asset.find("material[@name='store_floor']").set("texture", "store_tiles")

    (x0, x1), (y0, y1) = ROOM_X, ROOM_Y
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "material": "store_floor",
                                  "pos": f"{cx} {cy} {FLOOR_Z}", "size": f"{hx} {hy} 0.1",
                                  "contype": "1", "conaffinity": "2", "condim": "3"})
    # 0.6 m vinyl tiles: the checker texture holds a 2x2 block of them.
    asset.find("material[@name='store_floor']").set(
        "texrepeat", f"{hx * 2 / 1.2:.0f} {hy * 2 / 1.2:.0f}")

    t, h = 0.1, CEILING / 2
    walls = [("wall_back", (x0 - t, cy, h), (t, hy + 2 * t, h)),
             ("wall_left", (cx, y1 + t, h), (hx + 2 * t, t, h)),
             ("wall_right", (cx, y0 - t, h), (hx + 2 * t, t, h))]
    for name, (a, b) in (("wall_front_a", (y0, DOOR_Y[0])), ("wall_front_b", (DOOR_Y[1], y1))):
        walls.append((name, (x1 + t, (a + b) / 2, h), (t, (b - a) / 2, h)))
    walls.append(("wall_front_lintel", (x1 + t, sum(DOOR_Y) / 2, CEILING - 0.35),
                  (t, (DOOR_Y[1] - DOOR_Y[0]) / 2, 0.35)))
    for name, pos, size in walls:
        box(world, name, pos, size, "store_wall")
        if name != "wall_front_lintel":
            box(world, f"{name}_band", (pos[0], pos[1], CEILING - 0.45),
                (size[0] + 0.01, size[1] + 0.01, 0.2), "store_band", collide=False)
    box(world, "ceiling", (cx, cy, CEILING + 0.05), (hx + 0.2, hy + 0.2, 0.05), "store_ceiling",
        collide=False)

    for i, x in enumerate(np.arange(x0 + 1.2, x1 - 0.8, 2.0)):
        for j, y in enumerate(np.arange(y0 + 1.0, y1 - 0.6, 1.6)):
            box(world, f"panel_{i}_{j}", (x, y, CEILING - 0.005), (0.6, 0.3, 0.006),
                "light_panel", collide=False)

    for name, (px, py), length, yaw in (("fridges_back", (x0 + 0.4, cy), 2 * hy - 1.0,
                                         math.pi / 2),):
        bank = ET.SubElement(world, "frame", {"name": name, "pos": f"{px} {py} 0",
                                              "quat": quat_z(yaw)})
        box(bank, name, (0, 0, 1.05), (length / 2, 0.4, 1.05), "fridge_body")
        doors = int(length / 0.75)
        for k in range(doors):
            x = -length / 2 + (k + 0.5) * length / doors
            box(bank, f"{name}_door_{k}", (x, -0.405, 1.1), (length / doors / 2 - 0.03, 0.005, 0.85),
                "fridge_glass", collide=False)
            box(bank, f"{name}_frame_{k}", (x + length / doors / 2 - 0.015, -0.41, 1.1),
                (0.015, 0.008, 0.85), "fridge_frame", collide=False)
        box(bank, f"{name}_kick", (0, -0.405, 0.12), (length / 2, 0.006, 0.12), "fridge_frame",
            collide=False)

    rng = np.random.default_rng(3)
    fruit = [("fruit_apple", 0.040), ("fruit_orange", 0.042), ("fruit_lime", 0.030),
             ("fruit_lemon", 0.033)]
    for k, y in enumerate(PRODUCE_YS):
        name, r = fruit[k % len(fruit)]
        stand = ET.SubElement(world, "frame", {"name": f"produce_{k}",
                                               "pos": f"{PRODUCE_X} {y} 0"})
        box(stand, f"produce_{k}_stand", (0, 0, 0.35), (0.5, 0.4, 0.35), "counter_body")
        crate = ET.SubElement(stand, "frame", {"pos": "0 0 0.72", "euler": "0 -0.25 0"})
        box(crate, f"produce_{k}_crate", (0, 0, 0.0), (0.48, 0.38, 0.02), "crate", collide=False)
        for side, (px, py, sx, sy) in enumerate(((0, 0.37, 0.48, 0.015), (0, -0.37, 0.48, 0.015),
                                                (0.47, 0, 0.015, 0.38), (-0.47, 0, 0.015, 0.38))):
            box(crate, f"produce_{k}_side_{side}", (px, py, 0.06), (sx, sy, 0.06), "crate",
                collide=False)
        for a in np.arange(-0.44 + r, 0.44 - r + 1e-6, 2 * r + 0.004):
            for b in np.arange(-0.34 + r, 0.34 - r + 1e-6, 2 * r + 0.004):
                jitter = rng.uniform(-0.004, 0.004, 3)
                ET.SubElement(crate, "geom", {
                    "type": "sphere", "size": f"{r * rng.uniform(0.92, 1.05):.4f}",
                    "material": name, "contype": "0", "conaffinity": "0", "group": "2",
                    "pos": f"{a + jitter[0]:.4f} {b + jitter[1]:.4f} {0.02 + r + jitter[2]:.4f}"})

    for k, y in enumerate(CHECKOUT_YS):
        box(world, f"checkout_{k}", (CHECKOUT_X, y, 0.42), (0.8, 0.3, 0.42), "counter_body")
        box(world, f"checkout_{k}_top", (CHECKOUT_X, y, 0.855), (0.82, 0.32, 0.015), "counter_top",
            collide=False)
        box(world, f"checkout_{k}_belt", (CHECKOUT_X - 0.15, y + 0.02, 0.874), (0.6, 0.22, 0.005),
            "belt", collide=False)

    # One light only: the client gives every light soft shadows, one full scene pass each.
    ET.SubElement(world, "light", {"name": "key", "directional": "true", "pos": "0 0 6",
                                   "dir": "0.25 0.15 -1", "castshadow": "true",
                                   "diffuse": "0.45 0.44 0.42", "ambient": "0.25 0.25 0.25"})


def add_tiago(root: ET.Element, world: pathlib.Path) -> None:
    """TIAGo under its own default class, so its geom defaults do not leak into the store."""
    src_dir = MENAGERIE / "pal_tiago"
    tiago = ET.parse(src_dir / "tiago.xml").getroot()
    velocity = ET.parse(src_dir / "tiago_velocity.xml").getroot()
    meshdir = src_dir / tiago.find("compiler").get("meshdir", "")

    store_asset = root.find("asset")
    for el in tiago.find("asset"):
        if el.tag == "mesh":
            f = el.get("file")
            el.set("name", el.get("name") or pathlib.Path(f).stem)
            copy_into(meshdir / f, world / "assets/tiago" / f)
            el.set("file", f"tiago/{f}")
        store_asset.append(el)

    default = root.find("default")
    tiago_default = tiago.find("default")
    tiago_default.set("class", "tiago")
    default.append(tiago_default)

    base = tiago.find("worldbody/body[@name='base_link']")
    base.set("childclass", "tiago")
    base.set("pos", f"{SPAWN[0]} {SPAWN[1]} {FLOOR_Z + 0.005}")
    base.set("quat", quat_z(SPAWN[2]))

    # Home pose baked in through `ref`, so the arm starts tucked without a keyframe reset.
    for body in base.iter("body"):
        if body.get("name", "").startswith(GRAVCOMP_PREFIXES):
            body.set("gravcomp", "1")
        joint = body.find("joint")
        if joint is None or joint.get("name") not in ARM_HOME:
            continue
        angle = ARM_HOME[joint.get("name")]
        q = np.array([float(v) for v in body.get("quat", "1 0 0 0").split()])
        q = quat_mul(q / np.linalg.norm(q),
                     np.array([math.cos(angle / 2), 0, 0, math.sin(angle / 2)]))
        body.set("quat", " ".join(f"{v:.7f}" for v in q))
        joint.set("ref", f"{angle}")
    root.find("worldbody").append(base)

    root.append(tiago.find("contact"))
    root.append(velocity.find("actuator"))


def write_attribution(repo: pathlib.Path, world: pathlib.Path) -> None:
    shutil.copy2(repo / "LICENSE", world / "LICENSE.supermarket")
    shutil.copy2(repo / "NOTICE", world / "NOTICE.supermarket")
    (world / "ATTRIBUTION.md").write_text(
        "# Attribution\n\n"
        f"- Store and products: Dual-Arm Supermarket, https://github.com/{REPO} at {COMMIT}, "
        "Apache-2.0; product meshes from RoboCasa (MIT) and Objaverse (ODC-By 1.0). "
        "See LICENSE.supermarket and NOTICE.supermarket. Modified: roll cage, tower mount, "
        "cameras and lights removed, floor raised, TIAGo added.\n"
        "- TIAGo: PAL Robotics via MuJoCo Menagerie, Apache-2.0, "
        "https://github.com/google-deepmind/mujoco_menagerie/tree/main/pal_tiago\n")


def write_patrol(world: pathlib.Path) -> None:
    patrol = {"base_body": "base_link",
              "actuators": ["wheel_left_joint_vel", "wheel_right_joint_vel"],
              "waypoints": [[float(x), float(y)] for x, y in PATROL]}
    (world / "patrol.yaml").write_text(yaml.safe_dump(patrol, sort_keys=False))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--worlds", type=pathlib.Path, default=CACHE / "worlds")
    p.add_argument("--downloads", type=pathlib.Path, default=CACHE / "downloads")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()

    if not (MENAGERIE / "pal_tiago/tiago.xml").exists():
        print(f"no TIAGo under {MENAGERIE}; run mj-kdl-fetch-menagerie", file=sys.stderr)
        return 1

    world = args.worlds / "supermarket"
    scene = world / "scene.xml"
    if args.force or not scene.exists():
        repo = fetch(args.downloads)
        world.mkdir(parents=True, exist_ok=True)
        root = load_store(repo, world)
        add_tiago(root, world)
        write_attribution(repo, world)
        write_patrol(world)
        ET.indent(root, "  ")
        scene.write_text(ET.tostring(root, encoding="unicode"))

    print(scene)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
