#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Dumps a RoboCasa kitchen as a single assembled MJCF.

The arena on its own is walls and floor: fixtures are merged in when an environment is
constructed, which is also where the asset renaming happens that lets dozens of models coexist.
So a whole environment is built and its model written out, rather than the arena.

Needs RoboCasa in the interpreter running this; see docs/run.md.
"""

import argparse

from robocasa.utils.env_utils import create_env


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", help="MJCF to write")
    p.add_argument("--env", default="NavigateKitchen", help="RoboCasa environment name")
    p.add_argument("--layout", type=int, default=1)
    p.add_argument("--style", type=int, default=1)
    args = p.parse_args()

    env = create_env(
        env_name=args.env,
        layout_ids=[args.layout],
        style_ids=[args.style],
        render_onscreen=False,
        seed=0,
    )
    env.reset()

    with open(args.out, "w") as f:
        f.write(env.sim.model.get_xml())

    print(f"{args.out}: {env.sim.model.nbody} bodies, {env.sim.model.ngeom} geoms, "
          f"{env.sim.model.nmesh} meshes, {env.sim.model.ntex} textures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
