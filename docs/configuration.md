# Configuration {#page_configuration}

Nothing is hardcoded. The PC reads its settings from a ROS parameter file, the headset from a
JSON file on the device. The two describe the same contract from opposite ends, so `raw_ns` and
`out_ns` must agree between them.

## ROS parameters

`src/vr/config/vr.yaml` is installed to `share/vr/config/vr.yaml` and loaded by the launch file.
Another file can be supplied in its place:

```bash
ros2 launch vr sim.launch.py model:=... params_file:=/path/to/my.yaml
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
| `pc_time_topic` | `/vr/pc_time` | clock topic for the client's offset estimate |
| `pc_time_rate_hz` | `10.0` | |

The clock beacon is published by the input component, so a tracking-only run is stamped on PC
time too; the topic name is shared because it is part of the contract with the client either way.

The two namespaces must differ. `vr::InputNode` refuses to start otherwise, because it would be subscribing
to its own output.

### Scene component

| Parameter | Default | Meaning |
|---|---|---|
| `model` | — | MJCF path; required, supplied by the launch file |
| `manifest` | `""` | `manifest.json` written by `scene_export` |
| `scene_url` | `""` | URL the headset fetches `scene.glb` from |
| `frame_id` | `world` | frame the body poses are expressed in |
| `rate_hz` | `60.0` | body pose stream rate |
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
| `publish_hand_joints` | `true` | republish `<hand>/joints` |
| `publish_hand_tf` | `true` | 26 TF frames per hand; the `PoseArray` is published either way |
| `publish_gaze` | `true` | republish `<out>/gaze` |

### Grabbing

Read by the scene component, which owns the grabber.

| Parameter | Default | Meaning |
|---|---|---|
| `enable_grab` | `true` | |
| `grab_button` | `1` | index into `Joy` buttons; 1 is squeeze |
| `grab_reach_m` | `0.15` | fallback only: how close a hand must be when no pointer target exists |
| `grab_kp`, `grab_kd` | `400`, `40` | translation response [1/s², 1/s] |
| `grab_kp_rot`, `grab_kd_rot` | `100`, `20` | rotation response [1/s², 1/s] |
| `grab_max_accel`, `grab_max_ang_accel` | `150`, `100` | clamps [m/s², rad/s²] |
| `grab_vel_filter` | `0.3` | low-pass weight on the hand velocity the damper is fed |
| `grab_max_hand_speed` | `4.0` | [m/s] above this a sample is jitter, not motion |
| `grab_max_hand_turn_rate` | `15.0` | [rad/s] |

The gains are **accelerations, not forces**, and are scaled by each body's own mass and inertia.
A force-limited spring behaves completely differently on a 150 g ball and a 2 kg robot link — the
same 200 N clamp is 1300 m/s² for one and 100 m/s² for the other — so a reach that merely nudged
an arm would fling a loose object across the world. `kp` and `kd` describe a critically damped
second-order response: ω = √`kp`, and `kd` = 2ω.

The calibration parameters are identity until something measures the offset between the
headset's play space and the robot's world frame; see [Known
limits](limits.md#play-space-calibration). `publish_hand_tf` exists because two hands at 60 Hz
are 3120 transforms a second, which every `tf2` listener pays for; see
[Interfaces](interfaces.md#hand-joints).

The three `publish_*` parameters drop the republish, not the stream: the headset publishes into
`raw_ns` regardless, so they save ROS traffic and listener work rather than Wi-Fi.

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
  "turnSpeedDegPerSec": 90.0,
  "pointerRange": 8.0,
  "eyeHeight": 1.6,
  "spawnPosition": { "x": -1.5, "y": 0.0, "z": 0.0 },
  "spawnYawDegrees": 0.0,
  "frameId": "world",
  "resetService": "/vr_scene/reset",
  "inputMode": "controllers"
}
```

`spawnPosition` is in ROS coordinates and decides where the user stands when the app starts.
Everything is simulated, so this is a free choice — put it clear of the scene rather than inside
a table.

`inputMode` is `controllers` or `hands`, and is either/or rather than a preference: the runtime
stops reporting tracked hands while a controller is awake, so running both would leave one set
frozen wherever it was last seen. The menu button switches at runtime.

`eyeHeight` is where the floor goes. The client tracks in device space so that it needs no play
area, and the cost of that is that nothing measures the floor — see [Known
limits](limits.md#the-floor-is-assumed-not-measured).

When the file is missing the inspector defaults are used and written out, so after the first run
there is always a file to edit rather than a format to guess. Its path is logged at startup and
can be read with `adb logcat -s Unity`. Note that a file written by an older build keeps its old
fields: anything added since is filled from the defaults, not from the file.

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
