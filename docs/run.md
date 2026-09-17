# Running {#page_run}

## Export a world

`scene_export` converts an MJCF model into the two files the headset fetches: `scene.glb` and
`manifest.json`.

```bash
ros2 run vive_vr_ros2 scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml -o /tmp/vive_vr_scene
```

```
scene_export <model.xml> [-o OUT_DIR] [--groups 0,1,2] [--segments N] [--rings N]
             [--plane-extent M] [--sky-radius M] [--export-ground] [--export-sky]
```

| Option | Meaning |
|---|---|
| `-o` | output directory; created if missing |
| `--groups` | MuJoCo geom groups to export (default `0,1,2`) |
| `--segments`, `--rings` | tessellation density for round primitives |
| `--plane-extent` | half-size substituted for an infinite plane, since MuJoCo size 0 means unbounded |
| `--export-ground` | bake MuJoCo's unbounded ground plane into the file |
| `--export-sky` | bake a dome carrying the skybox gradient |
| `--sky-radius` | that dome's radius |

**The ground and the sky are left out by default.** An unbounded plane has no honest mesh, and a
baked patch of it aliases into moiré at a grazing angle, costs thousands of triangles and still
stops at its own edge. The client draws both with a real material and Unity's skybox — mipmapped,
shadow-receiving, one quad. The two flags put them back for a viewer that has to show the file
standing alone.

Finite planes are kept: those are geometry someone modelled. A textured one is tiled into
alternating squares of the texture's two colours, which is MuJoCo's checkerboard without needing
a sampler or UVs.

## The kitchen

The large test world is a [RoboCasa](https://github.com/robocasa/robocasa) kitchen — around 190
bodies, which is what the pose stream's send-only-what-moved design was measured against. It is
generated, not stored: nothing in this repository ships a world.

RoboCasa is a heavy dependency and only needed to build the MJCF, so it goes in a throwaway venv
rather than the workspace:

Not in `/tmp`: the install runs to gigabytes and the worlds take it to rebuild, which is exactly
how the first kitchen was lost. It sits beside the other cached heavy things, next to
`~/.cache/mj_kdl_wrapper`:

```bash
VENV=~/.cache/vive_vr_ros2/robocasa-venv
python3 -m venv "$VENV"
"$VENV/bin/pip" install "git+https://github.com/robocasa/robocasa.git"
"$VENV/bin/pip" install --force-reinstall --no-deps \
    "git+https://github.com/ARISE-Initiative/robosuite.git@master"
"$VENV/bin/python" -m robocasa.scripts.download_kitchen_assets
```

Then build the world and put something graspable in it:

```bash
WORLDS=~/.cache/vive_vr_ros2/worlds && mkdir -p "$WORLDS"
RC=$("$VENV/bin/python" -c 'import os, robocasa; print(os.path.dirname(robocasa.__file__))')

"$VENV/bin/python" scripts/make_kitchen.py "$WORLDS/kitchen.xml"
python3 scripts/add_kitchen_objects.py \
    "$RC/models/assets/objects/lightwheel" "$WORLDS/kitchen.xml" "$WORLDS/kitchen_objects.xml"

ros2 run vive_vr_ros2 scene_export "$WORLDS/kitchen_objects.xml" -o /tmp/vive_vr_kitchen
```

The export goes to `/tmp` because it is derived — one command rebuilds it from the MJCF. The
MJCF is not, so it does not.

The second step exists because **every fixture in a RoboCasa kitchen is welded**, so a kitchen
on its own has nothing that can be picked up. It adds six objects with a freejoint each on the
counter's near edge.

`--env`, `--layout` and `--style` pick a different kitchen; `NavigateKitchen` with layout 1,
style 1 is the one the figures above were measured on.

## What the file contains

Per body: its geometry in body-local coordinates, and its pose at the loaded state written into
the glTF node. The poses mean the file renders as an assembled scene in any glTF viewer, and the
client shows the world correctly the instant it loads rather than as a heap of parts waiting for
the first pose message.

MuJoCo's lights are exported as `KHR_lights_punctual` — type, position, direction and colour — so
a viewer lights the scene the way the simulator does.

Menagerie models put visual meshes in group 2 and collision shapes in group 3, so the default
gives the visual model. Fully transparent geoms (`rgba` alpha 0) are skipped, matching what
MuJoCo's own renderer shows — those are collision shapes that would otherwise cost triangles and
alpha blending on the headset GPU.

Geometry is baked **per body in body-local coordinates**: the client creates one object per body
and drives its pose from the live stream, so no body transform is stored in the file. Bodies with
no renderable geometry get `"node": -1` in the manifest.

The `.glb` is written in glTF's own Y-up convention, so it opens correctly in any glTF viewer —
which is how this step is verified. See [Interfaces](interfaces.md#coordinate-conventions) for
what happens to it on the client.

## Launch: tracking only

Everything the headset reports, calibrated into the world frame, with no simulation and no world
drawn on the device — the headset shows its welcome panel and waits.

```bash
ros2 launch vive_vr_ros2 tracking.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | `config/vive_vr.yaml` | topics, frames, rates, calibration |
| `rosbridge_port` | `9090` | |
| `publish_hand_joints` | `true` | republish `<hand>/joints` |
| `publish_hand_tf` | `true` | 26 TF frames per hand |
| `publish_gaze` | `true` | republish `<out>/gaze` |
| `container_name` | `vr_container` | where `sim.launch.py` loads the simulation |

This starts `rosbridge_websocket` and a component container holding `vive_vr_ros2::InputNode`, which also
publishes `<out>/pc_time` — the beacon the client's clock offset is estimated from, so poses are
stamped on PC time whether or not anything is being simulated.

The three `publish_*` arguments drop the **republish** only. The headset sends what it sends;
turning one off saves ROS traffic and TF listeners' work, not Wi-Fi.

## Launch: simulation and a world

```bash
ros2 launch vive_vr_ros2 sim.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vive_vr_scene
```

| Argument | Default | Meaning |
|---|---|---|
| `model` | — | MJCF to simulate; required |
| `scene_dir` | `/tmp/vive_vr_scene` | directory served over HTTP; where `scene_export` wrote |
| `host_ip` | the default route's | the address **the headset can reach**; goes into the `.glb` URL |
| `http_port` | `8000` | |
| `env_glb`, `env_yaw_deg`, `env_scale` | — | scenery drawn but not simulated |

Everything `tracking.launch.py` takes is accepted here too and passed through.

`host_ip` is embedded in the URL the headset fetches, so it has to be an address the headset can
reach — `0.0.0.0` means nothing to a remote client. The default is now this machine's address on
the default route, which is right whenever the headset is on the interface that route uses. It is
a default and not a detection: a machine whose default route is Ethernet while the headset is on
Wi-Fi still has to be told, `host_ip:=192.168.2.118`.

This includes `tracking.launch.py`, adds a plain `python3 -m http.server` for the `.glb`, and
loads `vive_vr_ros2::SceneNode` into the container that launch already started. The two components share
the container, so their topics cross intra-process rather than through the network stack.

With `rmw_zenoh` as the RMW, a router must be running:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## What should appear

```bash
ros2 topic list | grep vr
ros2 topic hz /vive_vr/body_poses          # ~60 Hz
ros2 topic echo /vive_vr/scene --once      # url + manifest, even if you subscribe late
```

`/vive_vr/scene` is latched (transient local), so a client that connects minutes later still receives
the world description.

## Reset

```bash
ros2 service call /vive_scene/reset std_srvs/srv/Trigger
```

This is `mj_kdl::reset`, which restores a keyframe **and** re-synchronises registered `Robot`
command ports — which a bare `mj_resetData` does not. It is what episode-based demonstration
recording needs.

## Running one component alone

Each component also builds as a standalone executable, which is easier to debug than a container:

```bash
ros2 run vive_vr_ros2 scene_node --ros-args --params-file config/vive_vr.yaml -p model:=/path/to.xml
ros2 run vive_vr_ros2 input_node --ros-args --params-file config/vive_vr.yaml
```

Next: [First run on the headset](bringup.md).
