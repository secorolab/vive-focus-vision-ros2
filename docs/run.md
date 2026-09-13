# Running {#page_run}

## Export a world

`scene_export` converts an MJCF model into the two files the headset fetches: `scene.glb` and
`manifest.json`.

```bash
ros2 run vr scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml -o /tmp/vr_robot
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

## Launch the stack

```bash
ros2 launch vr vr.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vr_robot \
    host_ip:=192.168.2.118
```

| Argument | Default | Meaning |
|---|---|---|
| `model` | — | MJCF to simulate; required |
| `scene_dir` | `/tmp/vr_scene` | directory served over HTTP; where `scene_export` wrote |
| `host_ip` | `0.0.0.0` | the address **the headset can reach**; goes into the `.glb` URL |
| `params_file` | `config/vr.yaml` | everything else — see [Configuration](configuration.md) |
| `rosbridge_port` | `9090` | |
| `http_port` | `8000` | |

`host_ip` must be this machine's LAN address, not `0.0.0.0`: it is embedded in the URL the
headset fetches, and `0.0.0.0` means nothing to a remote client.

This starts three things: `rosbridge_websocket`, a plain `python3 -m http.server` for the `.glb`,
and a component container holding `vr::SceneNode` and `vr::InputNode`. The two components share
the container, so their topics cross intra-process rather than through the network stack.

With `rmw_zenoh` as the RMW, a router must be running:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## What should appear

```bash
ros2 topic list | grep vr
ros2 topic hz /vr/body_poses          # ~60 Hz
ros2 topic echo /vr/scene --once      # url + manifest, even if you subscribe late
```

`/vr/scene` is latched (transient local), so a client that connects minutes later still receives
the world description.

## Reset

```bash
ros2 service call /vr_scene/reset std_srvs/srv/Trigger
```

This is `mj_kdl::reset`, which restores a keyframe **and** re-synchronises registered `Robot`
command ports — which a bare `mj_resetData` does not. It is what episode-based demonstration
recording needs.

## Running one component alone

Each component also builds as a standalone executable, which is easier to debug than a container:

```bash
ros2 run vr scene_node --ros-args --params-file src/vr/config/vr.yaml -p model:=/path/to.xml
ros2 run vr input_node --ros-args --params-file src/vr/config/vr.yaml
```

Next: [First run on the headset](bringup.md).
