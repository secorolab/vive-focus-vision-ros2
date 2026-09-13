# Configuration {#page_configuration}

Nothing is hardcoded. Both ends read their settings from a file, and the two files describe the
same contract seen from opposite sides — keep `rawNs`/`outNs` consistent between them.

## ROS side: `config/vr.yaml`

Installed to `share/vr/config/vr.yaml` and loaded by the launch file. Point the launch at a
different one without touching anything else:

```bash
ros2 launch vr vr.launch.py model:=... params_file:=/path/to/my.yaml
```

or override a single value:

```bash
ros2 run vr input_node --ros-args -p motion_eps_m:=0.01
```

### Shared

| Parameter | Default | Meaning |
|---|---|---|
| `raw_ns` | `/vr/raw` | namespace the headset publishes into |
| `out_ns` | `/vr` | namespace the PC publishes into |

They must differ; `InputNode` refuses to start otherwise, because it would be subscribing to its
own output.

### `vr_scene`

| Parameter | Default | Meaning |
|---|---|---|
| `model` | — | MJCF path; required, supplied by the launch file |
| `manifest` | `""` | `manifest.json` from `scene_export` |
| `scene_url` | `""` | URL the headset fetches `scene.glb` from |
| `frame_id` | `world` | frame the body poses are expressed in |
| `rate_hz` | `60.0` | body pose stream rate |
| `pc_time_topic` | `/vr/pc_time` | clock topic for the client's offset estimate |
| `pc_time_rate_hz` | `10.0` | |
| `timestep` | `0.002` | simulation timestep |
| `add_floor`, `add_skybox` | `false` | let the scene builder add a ground plane / sky |
| `gravity_z` | `-9.81` | |

### `vr_input`

| Parameter | Default | Meaning |
|---|---|---|
| `world_frame` | `world` | |
| `origin_frame` | `vr_origin` | the headset's play space |
| `origin_xyz` | `[0,0,0]` | play-space calibration, translation |
| `origin_rpy` | `[0,0,0]` | play-space calibration, rotation |
| `motion_eps_m` | `0.003` | motion below this does not count as movement |
| `stale_after_s` | `1.0` | no movement for this long ⇒ controller reported inactive |
| `hands` | `[left, right]` | |
| `head_name` | `head` | tracked and published, but has no buttons |
| `publish_hand_tf` | `true` | 26 TF frames per hand; see [Interfaces](interfaces.md#hand-joints) |

`origin_xyz`/`origin_rpy` are the play-space calibration and are identity until something measures
it. See [Known limits](limits.md#play-space-calibration).

## Client side: `vr_config.json`

`VrConfig` reads this from the device at startup, so the PC's address, the namespaces and the
publish rates change **without rebuilding the APK**:

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
  "originFrame": "vr_origin"
}
```

If the file is missing, the inspector defaults are used **and written out**, so after the first
run there is always a file to edit rather than a format to guess. The path is logged at startup;
read it with `adb logcat -s Unity`.

`VR_HOST` at build time only seeds the default that ships inside the APK — the file wins at
runtime.

## OpenXR features

Stored per platform in `Assets/XR/Settings/OpenXR Package Settings.asset` and committed, so a
fresh clone has them:

```
ON  VIVE Focus 3 Controller Interaction   ← without this the CommonUsages reads come back empty
ON  VIVE XR Support
ON  VIVE XR Hand Tracking                 ← without this XRHandSubsystem never reports a hand
ON  VIVE XR Eye Tracker (Beta)
ON  VIVE XR Display Refresh Rate
```

Meta Quest Support is off. Unity's own **Hand Tracking Subsystem must stay off** — it collides
with VIVE's and produces a duplicate extension error.

Reading this file from a shell: `m_enabled` is the feature flag. `m_Enabled` (capital E) is
MonoBehaviour's own field and means something else entirely.
