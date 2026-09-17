# vr {#page_vr}

MuJoCo worlds on a standalone VIVE Focus Vision, and what the headset sees of the user back as
ROS 2 topics.

A scene is exported to glTF and loaded once by an Android app on the headset, then driven live by
body poses over ROS 2. Controller poses and buttons, 26 hand joints per hand, and per-eye gaze
come back the other way. Physics stays on the PC — MuJoCo has no official Android build, and one
sim clock is what makes recorded demonstrations line up.

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

## Documentation

- [Installation](install.md) — workspace dependencies, ROS build, Unity editor and licence
- [Running](run.md) — export a world, launch the stack, what should appear
- [Interfaces](interfaces.md) — topics, TF frames, the `Joy` layout, `vr/EyeGaze`, frame conventions
- [Configuration](configuration.md) — `vr.yaml`, the device config file, OpenXR features
- [The Unity client](unity.md) — project layout, scripts, scripted setup and APK build
- [Embedding in a simulation](embedding.md) — `BodyPosePublisher` in an existing application
- [First run on the headset](bringup.md) — the on-device checklist
- [Testing](testing.md) — what is verified, how, and what is not
- [Design notes](design.md) — why the system is shaped this way, and what was rejected
- [Known limits](limits.md) — what is missing or approximate, and what it costs

## Status

| Part | State |
|---|---|
| `scene_export` MJCF → glTF | works; validated on primitives and mesh assets |
| `vr::SceneNode` sim, world stream, reset | works; 60 Hz measured |
| `vr::InputNode` calibration, TF, hands, gaze | works; verified through rosbridge |
| Unity project and APK | builds, 82 MB, zero errors |
| On-device run | works; scene loads, poses stream, pointing and grabbing verified |
| Hand tracking | untested end to end — the runtime only reports hands once the controllers idle |
| Eye tracking | untested on the device |

The PC side is verified by running it, against a stand-in client that speaks the same rosbridge
contract the Unity app does. The headset side is verified by running it on the headset. See
[Testing](testing.md) for exactly which claims rest on evidence and which do not, and [Known
limits](limits.md) for the two that bite in practice: the clock offset after the app is
suspended, and losing the boundary, which silently stops all input.

## Quick start

```bash
./tools/fetch_vive_plugin.sh                      # 361 MB, untracked on purpose
cp -r ~/work/ms/src/mj_kdl_wrapper ~/work/ms/src/orocos_kinematics_dynamics src/
source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
ros2 run vr scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml -o /tmp/vr_robot
ros2 launch vr sim.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vr_robot
```

`ros2 launch vr tracking.launch.py` is the same stack without the simulation: poses, buttons,
hands and gaze only.

Full detail in [Installation](install.md) and [Running](run.md).
