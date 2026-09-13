# vr — MuJoCo worlds and controller input on a VIVE Focus Vision

A MuJoCo scene is exported to glTF, loaded once by a standalone Android app on the headset, and
driven live by body poses over ROS 2. The headset's controller poses and buttons come back as
standard ROS 2 topics. Physics stays on the PC: MuJoCo has no official Android build, and keeping
one sim clock is what makes recorded demonstrations line up.

```
PC (Ubuntu 24.04, ROS 2 Jazzy)                     Focus Vision (Android APK)
┌──────────────────────────────┐                   ┌──────────────────────────┐
│ scene_export  MJCF -> .glb ──┼──── HTTP once ───▶│ glTFast runtime load     │
│ stream_node   mj_step ───────┼──── 60 Hz ───────▶│ Body_<name> transforms   │
│               TF, rosbag  ◀──┼──── 90 Hz ────────┤ grip pose + 5 buttons    │
│ rosbridge_websocket          │                   │ Unity 6 + VIVE OpenXR    │
└──────────────────────────────┘                   └──────────────────────────┘
```

## Status

| Part | State |
|---|---|
| `scene_export` MJCF → glTF | works; validated on primitives and mesh assets |
| `stream_node` sim + topics | works; 60 Hz measured |
| rosbridge round trip | verified both directions with a fake headset |
| Unity project + APK | builds, 38 MB, zero errors |
| **On-device run** | **not done yet — nothing has run on the headset** |

Verified by running, not by building:

- `scene_export` on `mug_table.xml` (primitives) and Menagerie `kinova_gen3/scene.xml` (meshes):
  9 bodies, 86,950 triangles, 7.3 MB `.glb`, node names matching link names. Structural
  validation (chunk layout, accessor bounds, index range, material indices) passes.
- `stream_node`: 11 topics, `/vr/body_poses` at 59.99 Hz, mug at z=0.042 matching the MJCF,
  `/vr/scene` delivered to a subscriber that connected *after* publication (transient-local QoS
  behaving as the design needs).
- Full stack from one launch command, exercised by `tools/fake_headset.py` speaking the same
  rosbridge contract the Unity client uses: client received body poses at 58.3 Hz, `pc_time` at
  9.7 Hz, and the scene manifest; the node received `buttons=101001` exactly as sent and
  republished the controller pose to TF bit-exact — `(0.11, 0.22, 0.33)`, quaternion
  `(0, 0, 0.7071068, 0.7071068)`. That test used three distinct translation components and a
  non-identity rotation so a transposed axis or dropped sign could not pass unnoticed.

What that does **not** prove: it validates the transport, not the handedness conversion — the
fake headset sends ROS-convention poses directly, while Unity runs them through `FrameConv`
first. Nor does an APK that compiles prove `ClientWebSocket` works under IL2CPP on ARM64, or
that the glTF correction rotation has the right sign. All three need the device.

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/vr/head/pose`, `/vr/{left,right}/pose` | `geometry_msgs/PoseStamped` | headset → PC | 90 Hz |
| `/vr/{left,right}/joy` | `sensor_msgs/Joy` | headset → PC | 90 Hz |
| `/vr/scene` | `std_msgs/String` (transient local) | PC → headset | once |
| `/vr/body_poses` | `geometry_msgs/PoseArray` | PC → headset | 60 Hz |
| `/vr/pc_time` | `builtin_interfaces/Time` | PC → headset | 10 Hz |

`Joy` carries no schema, so the layout is a contract:

```
buttons = [trigger, squeeze, primary(X|A), secondary(Y|B), stick_click, menu]
axes    = [stick_x, stick_y, trigger, squeeze]
```

`menu` exists on the left controller only and reads 0 on the right. The system button is reserved
by the runtime and never reaches the app. Poses are the **grip** pose (the one to retarget to an
end-effector); aim is not published yet.

`/vr/body_poses` index *i* is MuJoCo body *i*, the same order as `bodies[]` in the manifest.

## PC side

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vr_mujoco --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

MuJoCo is taken from `~/.cache/mj_kdl_wrapper/mujoco-3.9.0` (the same copy `mj_kdl_wrapper`
fetches) — override with `-DVR_MUJOCO_DIR=...`. No system paths are searched, so the
`/opt/mujoco-3.8.0` on this machine cannot silently produce an ABI mismatch.

```bash
# 1. convert a world
ros2 run vr_mujoco scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    -o /tmp/vr_robot

# 2. rosbridge + HTTP file server + the sim stream
ros2 launch vr_mujoco vr_mujoco.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vr_robot \
    host_ip:=192.168.2.118
```

`host_ip` must be the address the headset can reach (this machine's Wi-Fi IP, `wlp128s20f3`), not
`0.0.0.0` — it goes into the `.glb` URL the headset fetches. With `rmw_zenoh` as the RMW, a
router must be running: `ros2 run rmw_zenoh_cpp rmw_zenohd`.

### scene_export options

```
scene_export <model.xml> [-o OUT_DIR] [--groups 0,1,2] [--segments N] [--rings N]
             [--plane-extent M]
```

`--groups` selects which MuJoCo geom groups to export; Menagerie models put visual meshes in
group 2 and collision shapes in group 3, so the default `0,1,2` gives the visual model. Fully
transparent geoms are skipped, matching what MuJoCo's own renderer shows.

## Unity side

**The project is already created and configured.** It lives in `unity/VrRos/` and was set up
headlessly; the sections below record how, so it can be rebuilt or changed without guesswork.

Installed on this machine:

| Component | Version | Location |
|---|---|---|
| Unity Editor | 6000.0.83f1 (Unity 6 LTS) | `~/Unity/Hub/Editor/6000.0.83f1/` |
| VIVE OpenXR Plugin | 2.5.1 | vendored tarball, `unity/VrRos/vendor/` |
| Unity glTFast | 6.15.1 | package |
| OpenXR Plugin | 1.16.1 | package |
| XR Interaction Toolkit | 3.0.11 | package |
| Newtonsoft Json | 3.2.2 | package |
| rosbridge_suite | 2.7.1 | apt, `ros-jazzy-rosbridge-suite` |

Licence is Unity Personal, activated through the Hub GUI (the Hub CLI cannot sign in). Modern
Unity records it at `~/.config/unity3d/Unity/licenses/UnityEntitlementLicense.xml`, not the old
`Unity_lic.ulf`.

### How the project was made

```bash
UNITY=~/Unity/Hub/Editor/6000.0.83f1/Editor/Unity

# create the project over the existing Assets/ and Packages/ in unity/VrRos
$UNITY -batchmode -quit -nographics -createProject ~/work/p/vr/unity/VrRos

# scene, component wiring and Android player settings
VR_HOST=192.168.2.118 $UNITY -batchmode -quit -nographics \
  -projectPath ~/work/p/vr/unity/VrRos -buildTarget Android \
  -executeMethod VrRosSetup.SetupAll

# APK -> unity/VrRos/Build/VrRos.apk
$UNITY -batchmode -quit -nographics \
  -projectPath ~/work/p/vr/unity/VrRos -buildTarget Android \
  -executeMethod VrRosSetup.BuildApk
```

Only one Unity instance may hold a project: close the Editor before running these, or use the
**VrRos** menu the same script adds to the Editor menu bar (*Setup Scene and Android Settings*,
*Build APK*). The Editor path does not pick up `VR_HOST` — set `RosBridge.host` in the inspector
instead.

`Assets/Editor/VrRosSetup.cs` is the single source of truth for the setup. It sets IL2CPP, ARM64
only, min SDK 29, app id `sh.vamsi.vrros`, splash off; builds the scene by invoking the same
`GameObject/XR/XR Origin (VR)` menu command a human would, creates `VrRos` (`RosBridge`,
`ClockSync`, `VrInputPublisher`, `BodyPoseApplier`) with a `VrScene` child (`SceneLoader`), wires
every reference, and saves `Assets/Scenes/Main.unity` into the build list.

### Done by hand in the Editor

OpenXR features are stored per platform in `Assets/XR/Settings/OpenXR Package Settings.asset`,
keyed by GUID; these were ticked in the GUI and are now saved in the project:

```
ON  VIVE Focus 3 Controller Interaction   <- without this the CommonUsages reads come back empty
ON  VIVE XR Support
ON  VIVE XR Display Refresh Rate
ON  VIVE XR Eye Tracker (Beta)
```

Meta Quest Support is off. *Hand Tracking Subsystem* must stay off — it collides with VIVE's own
hand tracking. To inspect the state from a shell, grep the feature blocks for `m_enabled`; note
that `m_Enabled` (capital E) is MonoBehaviour's own field and means something else.

### VIVE plugin

Vendored as a UPM tarball in `unity/VrRos/vendor/` and referenced from `manifest.json` by
relative path, so opening the project resolves it with no installer step. To move to a newer
version, drop the new `.tgz` from
[the releases](https://github.com/ViveSoftware/VIVE-OpenXR-Unity/releases) beside it and update
the path. The `.unitypackage` on that page is *not* the plugin — it is a 2 KB bootstrap script
that downloads it, and is unnecessary here.

`vendor/` is 361 MB — exclude it if this workspace ever becomes a git repo, along with
`unity/VrRos/{Library,Temp,Logs,Build,UserSettings}`.

## Next: first run on the headset

Nothing has run on the device yet. In order, each step with a check that is not "it built":

1. **Install.** Headset in developer mode, connected by USB:
   `adb devices` (accept the prompt inside the headset), then
   `adb install -r ~/work/p/vr/unity/VrRos/Build/VrRos.apk`.
2. **Input.** Start the stack, launch the app, then on the PC: `ros2 topic echo /vr/right/joy`.
   Press each button in turn and watch the right index in `buttons` flip — the same
   `101001`-shaped output the fake headset produced, but from real presses.
   `ros2 topic hz /vr/right/pose` should show roughly the display rate.
   Watch `adb logcat -s Unity` for the client's own logs.
3. **Static load.** The world appears. If it is rotated 180°, negate the Y component of
   `SceneLoader.gltfCorrectionEuler` — see the note in `SceneLoader.cs`; ±90° are the only two
   possibilities.
4. **Live stream.** Objects move. Verify from a rosbag replay against sim state, not by eye.
5. **Teleop.** Controller grip pose to an IK target, with a clutch button. Not built yet; this is
   where `mj_kdl_wrapper`'s `Robot::jnt_pos_cmd` / `jnt_trq_cmd` ports come in.

Most likely first-run failures, in order: `ClientWebSocket` under IL2CPP (fallback is
[NativeWebSocket](https://github.com/endel/NativeWebSocket), a change confined to
`RosBridge.cs`); the glTF correction sign; and the headset not reaching `192.168.2.118` because
it is on a different Wi-Fi network or band.

A known VIVE quirk for step 5: controllers keep reporting `isTracked` even when set down on a
table, so teleop must gate on pose delta or input recency rather than the tracked flag.

## Known limits

- **Play-space origin.** `/vr/*/pose` is expressed in `vr_origin`, the headset's own play space,
  and `stream_node` currently treats that as identical to `world`. Anything needing the headset
  and the robot in one metric frame needs a calibration step first. This is the limitation that
  matters most once a real robot is involved.
- **No teleop yet.** `stream_node` republishes controller poses to TF for verification; it does
  not drive anything.
- **Clock offset** is estimated one-way from `/vr/pc_time`, so stamps are biased late by about
  half the Wi-Fi RTT (single-digit ms). `ClockSync.cs` documents the round-trip upgrade.
- **Materials** are flat colours. Textures in the MJCF are not exported.
- **Normals** are flat-shaded, which triples the vertex count (the Kinova scene is 87k triangles,
  7.3 MB). Smooth normals and indexed dedup are the fix if size matters.
- **JSON, not CBOR.** Fine for tens of bodies. A scene with hundreds should move
  `/vr/body_poses` to rosbridge's CBOR compression before blaming the renderer.
- **`vr_mujoco` links MuJoCo directly**, not through `mj_kdl_wrapper`: that checkout is built
  in-tree but never installed, so there is no `Targets.cmake` to import, and nothing here needs
  KDL yet. It becomes a dependency at step 5.
- `/vr/{left,right}/haptic` does not exist yet; the controllers can only receive.

## Layout

```
src/vr_mujoco/             ROS 2 package
  src/scene_export.cpp     MJCF -> scene.glb + manifest.json
  src/gltf_writer.cpp      minimal .glb writer
  src/geom_mesh.cpp        MuJoCo geom tessellation
  src/stream_node.cpp      sim loop, pose stream, controller TF
  launch/                  rosbridge + file server + stream
unity/VrRos/
  Assets/Editor/VrRosSetup.cs   scripted project setup and APK build
  Assets/Scripts/
    RosBridge.cs           rosbridge v2 over WebSocket
    FrameConv.cs           the only place handedness is converted
    VrInputPublisher.cs    grip poses + buttons, via UnityEngine.XR CommonUsages
    SceneLoader.cs         /vr/scene -> HTTP fetch -> glTFast -> body holders
    BodyPoseApplier.cs     /vr/body_poses -> transforms
    ClockSync.cs           headset clock vs PC ROS clock
  vendor/                  VIVE OpenXR 2.5.1 UPM tarball (361 MB)
  Build/VrRos.apk          38 MB, ARM64
tools/
  fake_headset.py          stands in for the client; exercises every topic both ways
  check_glb.py             structural validation of an exported .glb
```

### tools

```bash
# with the stack running: publishes pose+joy at 90 Hz, reports what comes back
python3 tools/fake_headset.py

# chunk layout, accessor bounds, index range, material indices
python3 tools/check_glb.py /tmp/vr_robot/scene.glb
```

`fake_headset.py` needs `python3-websockets` (installed). It is the fastest way to tell whether a
problem is on the PC side or the headset side: if it still passes, the PC half is fine.

## Why this shape

The design follows what shipping VR-teleop stacks converged on rather than a generic bridge, but
deliberately stops short of adopting one. [XRoboToolkit](https://arxiv.org/abs/2508.00097)
(PICO's open-source framework) is the closest prior art: custom TCP, fixed ~56-byte pose packets
at 90 Hz, a thin ROS node publishing `xr_msgs/Controller`. Adopting it was rejected after
measuring the port cost — its Quest fork rewrote `TrackingData.cs` with 693 changed lines against
a 512-line file, 39 files differ between the two existing forks with no abstraction seam, and it
carries a 995 MB Qt6/gRPC PC service plus a Foxy/Humble ROS side, almost all of it in service of
streaming camera video from a real robot. Two things were taken from it: the `Joy` layout shape,
and the confirmation that `UnityEngine.XR.InputDevices` + `CommonUsages` is enough for controller
pose and buttons — about 40 lines, not a framework.

Transport is rosbridge alone because it is the only option released and maintained for Jazzy
(2.7.1, Aug 2026), it is RMW-agnostic so `rmw_zenoh` changes nothing, and the split
binary-TCP-plus-rosbridge design was dropped in favour of keeping one protocol and one client.
