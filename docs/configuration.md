# Configuration {#page_configuration}

Nothing is hardcoded. The PC reads its settings from a ROS parameter file, the headset from a
JSON file on the device. The two describe the same contract from opposite ends, so `raw_ns` and
`out_ns` must agree between them.

## ROS parameters

`src/vr/config/vr.yaml` is installed to `share/vr/config/vr.yaml` and loaded by the launch file.
Another file can be supplied in its place:

```bash
ros2 launch vr vr.launch.py model:=... params_file:=/path/to/my.yaml
```

A single value can be overridden on the command line:

```bash
ros2 run vr input_node --ros-args -p motion_eps_m:=0.01
```

### Shared by both components

| Parameter | Default | Meaning |
|---|---|---|
| `raw_ns` | `/vr/raw` | namespace the headset publishes into |
| `out_ns` | `/vr` | namespace the PC publishes into |

The two must differ. `vr::InputNode` refuses to start otherwise, because it would be subscribing
to its own output.

### Scene component

| Parameter | Default | Meaning |
|---|---|---|
| `model` | — | MJCF path; required, supplied by the launch file |
| `manifest` | `""` | `manifest.json` written by `scene_export` |
| `scene_url` | `""` | URL the headset fetches `scene.glb` from |
| `frame_id` | `world` | frame the body poses are expressed in |
| `rate_hz` | `60.0` | body pose stream rate |
| `pc_time_topic` | `/vr/pc_time` | clock topic for the client's offset estimate |
| `pc_time_rate_hz` | `10.0` | |
| `timestep` | `0.002` | simulation timestep |
| `add_floor` | `false` | let the scene builder add a ground plane |
| `add_skybox` | `false` | let the scene builder add a sky and directional light |
| `gravity_z` | `-9.81` | |

### Input component

| Parameter | Default | Meaning |
|---|---|---|
| `world_frame` | `world` | |
| `origin_frame` | `vr_origin` | the headset's play space |
| `origin_xyz` | `[0, 0, 0]` | play-space calibration, translation |
| `origin_rpy` | `[0, 0, 0]` | play-space calibration, rotation |
| `motion_eps_m` | `0.003` | movement below this does not count as movement |
| `stale_after_s` | `1.0` | no movement for this long, and the controller is reported inactive |
| `hands` | `[left, right]` | |
| `head_name` | `head` | tracked and published, but has no buttons |
| `publish_hand_tf` | `true` | 26 TF frames per hand; the `PoseArray` is published either way |

### Grabbing

Read by the scene component, which owns the grabber.

| Parameter | Default | Meaning |
|---|---|---|
| `enable_grab` | `true` | |
| `grab_button` | `1` | index into `Joy` buttons; 1 is squeeze |
| `grab_reach_m` | `0.15` | a body further than this from the hand is not caught |
| `grab_kp`, `grab_kd` | `400`, `40` | translation spring and damping [N/m, Ns/m] |
| `grab_kp_rot`, `grab_kd_rot` | `15`, `2` | rotation spring and damping [Nm/rad, Nms/rad] |
| `grab_max_force`, `grab_max_torque` | `200`, `20` | clamps, or a long reach launches the object |

The calibration parameters are identity until something measures the offset between the
headset's play space and the robot's world frame; see [Known
limits](limits.md#play-space-calibration). `publish_hand_tf` exists because two hands at 60 Hz
are 3120 transforms a second, which every `tf2` listener pays for; see
[Interfaces](interfaces.md#hand-joints).

## Headset settings

`VrConfig` reads `vr_config.json` from the device at startup, so the PC's address, the namespaces
and the publish rates change without rebuilding the APK:

```bash
adb push vr_config.json /sdcard/Android/data/sh.vamsi.vrros/files/vr_config.json
```

```json
{
  "host": "192.168.2.118",
  "port": 9090,
  "rawNs": "/vr/raw",
  "outNs": "/vr",
  "maxRateHz": 90.0,
  "handRateHz": 60.0,
  "gazeRateHz": 60.0,
  "moveSpeed": 1.5,
  "snapDegrees": 45.0,
  "spawnPosition": { "x": -1.5, "y": 0.0, "z": 0.0 },
  "spawnYawDegrees": 0.0,
  "frameId": "world"
}
```

`spawnPosition` is in ROS coordinates and decides where the user stands when the app starts.
Everything is simulated, so this is a free choice — put it clear of the scene rather than inside
a table. `moveSpeed` and `snapDegrees` tune the stick locomotion.

When the file is missing the inspector defaults are used and written out, so after the first run
there is always a file to edit rather than a format to guess. Its path is logged at startup and
can be read with `adb logcat -s Unity`.

`VR_HOST` at build time only seeds the default that ships inside the APK. The file wins at
runtime.

## OpenXR features

These are stored per platform in `Assets/XR/Settings/OpenXR Package Settings.asset` and are
committed, so a fresh clone has them:

| Feature | Why |
|---|---|
| VIVE Focus 3 Controller Interaction | without it the `CommonUsages` reads come back empty |
| VIVE XR Support | |
| VIVE XR Hand Tracking | without it `XRHandSubsystem` never reports a hand |
| VIVE XR Eye Tracker (Beta) | |
| VIVE XR Display Refresh Rate | |

Meta Quest Support is off. Unity's own Hand Tracking Subsystem must stay off; it collides with
VIVE's and produces a duplicate extension error.

When reading that file from a shell, `m_enabled` is the feature flag. `m_Enabled` with a capital
E is MonoBehaviour's own field and means something else.
