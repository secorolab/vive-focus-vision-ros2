# vr — MuJoCo worlds and headset input on a VIVE Focus Vision

A MuJoCo scene is exported to glTF, loaded once by a standalone Android app on the headset, and
driven live by body poses over ROS 2. What the headset sees of the user — controller poses and
buttons, 26 hand joints per hand, per-eye gaze — comes back as ROS 2 topics and TF. Physics stays
on the PC: MuJoCo has no official Android build, and keeping one sim clock is what makes recorded
demonstrations line up.

```
PC (Ubuntu 24.04, ROS 2 Jazzy)                     Focus Vision (Android APK)
┌──────────────────────────────┐                   ┌──────────────────────────┐
│ scene_export  MJCF -> .glb ──┼──── HTTP once ───▶│ glTFast runtime load     │
│ vr::SceneNode  mj_step ──────┼──── 60 Hz ───────▶│ Body_<name> transforms   │
│ vr::InputNode         ◀──────┼──── 90/60 Hz ─────┤ grip pose + 5 buttons    │
│   calibrate, TF, activity    │                   │ hand joints, eye gaze    │
│ rosbridge_websocket          │                   │ Unity 6 + VIVE OpenXR    │
└──────────────────────────────┘                   └──────────────────────────┘
```

## Status

| Part | State |
|---|---|
| `scene_export` MJCF → glTF | works; validated on primitives and mesh assets |
| `vr::SceneNode` sim + stream + reset | works; 60 Hz measured |
| `vr::InputNode` calibration, TF, hands, gaze | works; verified through rosbridge |
| Unity project + APK | builds, 39 MB, zero errors |
| **On-device run** | **not done yet — nothing has run on the headset** |

Verified by running, not by building:

- `scene_export` on `mug_table.xml` (primitives) and Menagerie `kinova_gen3/scene.xml` (meshes):
  9 bodies, 86,950 triangles, 7.3 MB `.glb`. Structural validation (chunk layout, accessor
  bounds, index range, material indices) passes — `tools/check_glb.py`.
- `/vr/body_poses` at 59.99 Hz, mug at z=0.042 matching the MJCF, `/vr/scene` delivered to a
  subscriber that connected *after* publication (transient-local QoS behaving as needed).
- Full stack from one launch command, exercised by `tools/fake_headset.py` speaking the same
  rosbridge contract the Unity client uses: buttons arrive as sent (`101001`), the controller
  pose round-trips bit-exact to TF — `(0.11, 0.22, 0.33)`, quaternion `(0,0,0.7071068,0.7071068)`
  — the `active` flag reads `true` when moving and `false` when held still, gaze arrives in the
  `world` frame with pupil data intact, and all 26 hand joints appear as correctly parented TF
  frames at 57 Hz (checked with `ros2 run tf2_tools view_frames`).

What that does **not** prove: it validates the transport, not the handedness conversion — the
fake headset sends ROS-convention poses directly, while Unity runs them through `FrameConv`
first. Nor does a compiling APK prove `ClientWebSocket` works under IL2CPP on ARM64, or that the
glTF correction rotation has the right sign, or whether eye tracking needs an Android permission
(no permission string exists anywhere in the VIVE plugin, but that is not proof). All need the
device.

## Interfaces

Nothing below is hardcoded. Every namespace, frame, rate and the calibration live in
`src/vr/config/vr.yaml`; the client side lives in a JSON file on the headset (see *Device
config*).

The headset publishes into `raw_ns` (`/vr/raw`); `vr::InputNode` calibrates and republishes into
`out_ns` (`/vr`). They must differ, or the node would subscribe to its own output.

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `<raw>/{head,left,right}/pose` → `<out>/…` | `geometry_msgs/PoseStamped` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joy` → `<out>/…` | `sensor_msgs/Joy` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joints` → TF | `geometry_msgs/PoseArray` (26) | headset → PC | 60 Hz |
| `<raw>/gaze` → `<out>/gaze` | `vr/EyeGaze` | headset → PC | 60 Hz |
| `<out>/{left,right}/active` | `std_msgs/Bool` (latched) | PC | on change |
| `<out>/scene` | `std_msgs/String` (transient local) | PC → headset | once |
| `<out>/body_poses` | `geometry_msgs/PoseArray` | PC → headset | 60 Hz |
| `<out>/pc_time` | `builtin_interfaces/Time` | PC → headset | 10 Hz |
| `/vr_scene/reset` | `std_srvs/Trigger` | service | on call |

`Joy` carries no schema, so the layout is a contract:

```
buttons = [trigger, squeeze, primary(X|A), secondary(Y|B), stick_click, menu]
axes    = [stick_x, stick_y, trigger, squeeze]
```

`menu` exists on the left controller only and reads 0 on the right; the system button is reserved
by the runtime and never reaches the app. Poses are the **grip** pose (the one to retarget to an
end-effector); aim is not published yet.

`/vr/body_poses` index *i* is MuJoCo body *i*, the same order as `bodies[]` in the manifest.

**`active`** exists because VIVE controllers keep reporting a tracked pose while lying on a table.
A controller that has not moved `motion_eps_m` within `stale_after_s` is reported inactive; a
consumer should refuse to act on a pose that is merely the last one seen.

**Hands** become TF, not `JointState`: `sensor_msgs/JointState` carries scalar joint positions,
while XR Hands reports 6-DoF poses per joint, and deriving angles would need a hand kinematic
model this project does not have. The result is a real skeleton — `world → vr_<hand>_wrist →
vr_<hand>_palm`, and each finger `metacarpal → proximal → intermediate → distal → tip`. At 60 Hz
that is 3120 transforms a second that every tf2 listener pays for, so `publish_hand_tf: false`
turns it off.

**Gaze** is the one custom message. Nothing gaze-, eye-, hand- or skeleton-shaped exists in any
installed msgs package, so `vr/msg/EyeGaze` carries a pose per eye with its own validity, plus
pupil diameter in millimetres with its own validity. Per-eye rather than one averaged ray on
purpose: losing one eye is exactly what recorded data needs to be able to explain.

## PC side

### Workspace dependencies

`vr` depends on `mj_kdl_wrapper`, which needs the secorolab Orocos KDL fork built as its own
workspace package — the distro ships `liborocos-kdl.so.1.5` with the same SONAME, and a process
can only hold one. Neither is committed here; both are their own git repos:

```bash
cp -r ~/work/ms/src/mj_kdl_wrapper ~/work/ms/src/orocos_kinematics_dynamics src/
# or clone: github.com/vamsikalagaturu/mj_kdl_wrapper (dev)
#           github.com/secorolab/orocos_kinematics_dynamics (feature/achd_fixed_joint)
```

`colcon.meta` turns off the wrapper's examples and tests for this workspace. `unity/COLCON_IGNORE`
exists because colcon otherwise discovers a package inside Unity's Android build temp and aborts
the build.

```bash
sudo apt install ros-jazzy-rosbridge-suite libeigen3-dev libglfw3-dev libgl-dev libegl-dev ffmpeg
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

MuJoCo (3.9.0) comes from `~/.cache/mj_kdl_wrapper/mujoco-3.9.0`, the same copy the wrapper
fetches — override with `-DVR_MUJOCO_DIR=...`. No system paths are searched, so the
`/opt/mujoco-3.8.0` on this machine cannot silently produce an ABI mismatch.

### Run

```bash
# 1. convert a world
ros2 run vr scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml -o /tmp/vr_robot

# 2. rosbridge + HTTP file server + both components in one container
ros2 launch vr vr.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vr_robot \
    host_ip:=192.168.2.118            # the address the headset can reach; never 0.0.0.0

# different settings without touching the launch file
ros2 launch vr vr.launch.py ... params_file:=/path/to/my.yaml
```

With `rmw_zenoh` as the RMW, a router must be running: `ros2 run rmw_zenoh_cpp rmw_zenohd`.

`SceneNode` and `InputNode` share one container, so their topics cross intra-process.

### scene_export

```
scene_export <model.xml> [-o OUT_DIR] [--groups 0,1,2] [--segments N] [--rings N]
             [--plane-extent M]
```

`--groups` selects which MuJoCo geom groups to export; Menagerie models put visual meshes in
group 2 and collision shapes in group 3, so the default `0,1,2` gives the visual model. Fully
transparent geoms are skipped, matching what MuJoCo's own renderer shows.

### Embedding in an existing simulation

`vr::BodyPosePublisher` is a class, not a node — the same shape as `mj_kdl`'s
`CameraRosPublisher`. It creates no node, executor or thread, owns no `mjData`, and links only
MuJoCo (no KDL), so an existing application streams itself to the headset with two lines:

```cpp
vr::BodyPosePublisher vr_out(*node, model, conf);
while (running) {
    mj_step(model, data);
    if (vr_out.wants_update(data->time)) vr_out.publish(data);
}
```

`vr::SceneNode` is that class plus a simulation, for when you just want to look at an MJCF. It
builds the world through `mj_kdl::init_env`, so scenes stay composable and `/vr_scene/reset`
restores a keyframe *and* re-syncs Robot command ports — which `mj_resetData` alone does not.

## Unity side

### Setup from a fresh clone

The VIVE OpenXR plugin is a 361 MB tarball and is deliberately **not** in git. `manifest.json`
references it by relative path, so Unity cannot open the project until it is fetched:

```bash
./tools/fetch_vive_plugin.sh          # -> unity/VrRos/vendor/com.htc.upm.vive.openxr-2.5.1.tgz
```

Idempotent and checksum-pinned. Set `VIVE_OPENXR_VERSION` for a different release, then point
`manifest.json` at the new filename. Everything else — packages, scene, Android settings — is in
the repo and resolves on first open.

### Editor and licence

```bash
sudo install -d /etc/apt/keyrings
curl -fsSL https://hub.unity3d.com/linux/keys/public \
  | sudo gpg --dearmor -o /etc/apt/keyrings/unityhub.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/unityhub.gpg] \
https://hub.unity3d.com/linux/repos/deb stable main" \
  | sudo tee /etc/apt/sources.list.d/unityhub.list
sudo apt update && sudo apt install unityhub

unityhub --headless install --version 6000.0.83f1 \
  --module android android-sdk-ndk-tools android-open-jdk --childModules
```

The licence cannot be done headlessly: open Unity Hub, sign in, take a **Personal** licence. Until
then the editor refuses to open a project. Modern Unity records it at
`~/.config/unity3d/Unity/licenses/UnityEntitlementLicense.xml`, not the old `Unity_lic.ulf`.

| Component | Version |
|---|---|
| Unity Editor | 6000.0.83f1 (Unity 6 LTS), `~/Unity/Hub/Editor/` |
| VIVE OpenXR Plugin | 2.5.1 (vendored tarball) |
| glTFast / OpenXR / XR Hands / XRI / Newtonsoft | 6.15.1 / 1.16.1 / 1.5.1 / 3.0.11 / 3.2.2 |

Versions are pinned: `manifest.json` and `packages-lock.json` agree, so nothing drifts on open.

### Build

`Assets/Editor/VrRosSetup.cs` is the source of truth for scene, wiring and Android settings —
none of it is a list of things to click:

```bash
UNITY=~/Unity/Hub/Editor/6000.0.83f1/Editor/Unity
VR_HOST=192.168.2.118 $UNITY -batchmode -quit -nographics \
  -projectPath ~/work/p/vr/unity/VrRos -buildTarget Android -executeMethod VrRosSetup.SetupAll
$UNITY -batchmode -quit -nographics -projectPath ~/work/p/vr/unity/VrRos \
  -buildTarget Android -executeMethod VrRosSetup.BuildApk     # -> Build/VrRos.apk
```

Only one Unity instance may hold a project: **close the Editor first**, or use the `VrRos` menu
the same script adds to the menu bar (which does not read `VR_HOST`).

It sets IL2CPP, ARM64 only, min SDK 29, app id `sh.vamsi.vrros`, splash off; builds the scene by
invoking the same `GameObject/XR/XR Origin (VR)` menu command a human would; creates `VrRos`
(`VrConfig`, `RosBridge`, `ClockSync`, `VrInputPublisher`, `BodyPoseApplier`, `HandPublisher`,
`GazePublisher`) with a `VrScene` child (`SceneLoader`); wires every reference.

### Device config

`VrConfig` reads `vr_config.json` from the device, so the PC address, namespaces and rates change
without rebuilding:

```bash
adb push vr_config.json /sdcard/Android/data/sh.vamsi.vrros/files/vr_config.json
```

If the file is missing the inspector defaults are used and written out, so there is always
something to edit. Keep `rawNs`/`outNs` consistent with `config/vr.yaml` — same contract, two
ends.

### OpenXR features

Stored per platform in `Assets/XR/Settings/OpenXR Package Settings.asset`, and in the repo:

```
ON  VIVE Focus 3 Controller Interaction   <- without this the CommonUsages reads come back empty
ON  VIVE XR Support
ON  VIVE XR Hand Tracking                 <- without this XRHandSubsystem never reports a hand
ON  VIVE XR Eye Tracker (Beta)
ON  VIVE XR Display Refresh Rate
```

Meta Quest Support is off. Unity's *Hand Tracking Subsystem* must stay off — it collides with
VIVE's. Reading this file from a shell: `m_enabled` is the feature flag; `m_Enabled` is
MonoBehaviour's own field and means something else.

## Next: first run on the headset

Nothing has run on the device yet. Each step has a check that is not "it built":

1. **Install.** Developer mode on, connected by USB: `adb devices` (accept the prompt inside the
   headset), then `adb install -r unity/VrRos/Build/VrRos.apk`.
2. **Input.** Start the stack, launch the app, then `ros2 topic echo /vr/right/joy`. Press each
   button and watch the right index flip — the same `101001`-shaped output the fake headset
   produced, but from real presses. `adb logcat -s Unity` carries the client's own logs.
3. **Static load.** The world appears. If it is rotated 180°, negate the Y component of
   `SceneLoader.gltfCorrectionEuler`; ±90° are the only two possibilities.
4. **Live stream.** Objects move. Verify from a rosbag replay against sim state, not by eye.
5. **Hands and gaze.** `ros2 run tf2_tools view_frames` should show both hand skeletons;
   `ros2 topic echo /vr/gaze` should report valid eyes once eye tracking is calibrated in the
   headset's own settings.
6. **Teleop.** Controller grip pose to an IK target, with a clutch button. Not built yet; this is
   where `mj_kdl_wrapper`'s IK/ACHD solvers and `Robot::jnt_pos_cmd` come in.

Most likely first-run failures, in order: `ClientWebSocket` under IL2CPP (fallback is
[NativeWebSocket](https://github.com/endel/NativeWebSocket), a change confined to
`RosBridge.cs`); the glTF correction sign; the headset not reaching the PC because it is on a
different Wi-Fi network or band; and eye tracking needing a permission or calibration nobody
has done.

## Known limits

- **Play-space origin.** `origin_xyz`/`origin_rpy` in `vr.yaml` are identity, so the headset's
  guardian origin and the robot's world frame are only coincidentally the same place. Anything
  needing both in one metric frame needs a calibration step first. This is the limit that matters
  most once a real robot is involved.
- **No teleop yet.** `InputNode` publishes calibrated poses and TF; nothing drives a robot.
- **Clock offset** is estimated one-way from `/vr/pc_time`, so stamps are biased late by about
  half the Wi-Fi RTT (single-digit ms). `ClockSync.cs` documents the round-trip upgrade.
- **Materials** are flat colours; textures in the MJCF are not exported. **Normals** are flat,
  which triples the vertex count (Kinova: 87k triangles, 7.3 MB).
- **JSON, not CBOR.** Fine for tens of bodies. Hundreds should move `/vr/body_poses` to
  rosbridge's CBOR compression before blaming the renderer.
- `/vr/{left,right}/haptic` does not exist; the controllers can only receive.

## Layout

```
src/vr/                        one package, composable nodes
  msg/EyeGaze.msg              the only custom type, because ROS 2 has no gaze message
  config/vr.yaml               every topic, frame, rate and the calibration
  include/vr/
    body_pose_publisher.hpp    embeddable in any MuJoCo app; no KDL
    gltf_writer.hpp, geom_mesh.hpp
  src/
    body_pose_publisher.cpp, gltf_writer.cpp, geom_mesh.cpp
    scene_export.cpp           MJCF -> scene.glb + manifest.json
    components/
      scene_node.cpp           vr::SceneNode   sim, world stream, reset service
      input_node.cpp           vr::InputNode   calibration, TF, hands, gaze, activity
  launch/vr.launch.py
unity/VrRos/Assets/Scripts/
  VrConfig.cs                  device-side JSON config
  RosBridge.cs                 rosbridge v2 over WebSocket
  FrameConv.cs                 the only place handedness is converted
  VrInputPublisher.cs          grip poses + buttons, via UnityEngine.XR CommonUsages
  HandPublisher.cs             26 joints per hand, via XRHandSubsystem
  GazePublisher.cs             per-eye gaze + pupil, via ViveEyeTracker
  SceneLoader.cs               /vr/scene -> HTTP fetch -> glTFast -> body holders
  BodyPoseApplier.cs           /vr/body_poses -> transforms
  ClockSync.cs                 headset clock vs PC ROS clock
tools/
  fetch_vive_plugin.sh         downloads the untracked VIVE OpenXR tarball
  fake_headset.py              stands in for the client; exercises every topic both ways
  check_glb.py                 structural validation of an exported .glb
```

```bash
python3 tools/fake_headset.py --seconds 10          # with the stack running
python3 tools/fake_headset.py --still               # exercises the inactive branch
python3 tools/check_glb.py /tmp/vr_robot/scene.glb
```

`fake_headset.py` is the fastest way to tell whether a problem is on the PC side or the headset:
if it still passes, the PC half is fine.

## Why this shape

The design follows what shipping VR-teleop stacks converged on, but deliberately stops short of
adopting one. [XRoboToolkit](https://arxiv.org/abs/2508.00097) (PICO's open-source framework) is
the closest prior art: custom TCP, fixed ~56-byte pose packets at 90 Hz, a thin ROS node
publishing `xr_msgs/Controller`. Adopting it was rejected after measuring the port cost — its
Quest fork rewrote `TrackingData.cs` with 693 changed lines against a 512-line file, 39 files
differ between the two existing forks with no abstraction seam, and it carries a 995 MB Qt6/gRPC
PC service plus a Foxy/Humble ROS side, almost all of it serving camera-video streaming this
project does not need. Two things were taken from it: the `Joy` layout shape, and the
confirmation that `UnityEngine.XR.InputDevices` + `CommonUsages` is enough for controller pose
and buttons — about 40 lines, not a framework.

Transport is rosbridge alone because it is the only option released and maintained for Jazzy
(2.7.1, Aug 2026), and being rclpy-based it is RMW-agnostic, so `rmw_zenoh` changes nothing.
