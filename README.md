# vr

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
│ vive_vr_ros2::SceneNode  mj_step ──────┼──── 60 Hz ───────▶│ Body_<name> transforms   │
│ vive_vr_ros2::InputNode         ◀──────┼──── 90/60 Hz ─────┤ grip pose + 5 buttons    │
│   calibrate, TF, activity    │                   │ hand joints, eye gaze    │
│ rosbridge_websocket          │                   │ Unity 6 + VIVE OpenXR    │
└──────────────────────────────┘                   └──────────────────────────┘
```

## Quick start

This repository is one ROS 2 package. It is cloned into a workspace, not built in place:

```bash
mkdir -p ~/work/p/vrws/src && cd ~/work/p/vrws
git clone git@github.com:secorolab/vive-focus-vision-ros2.git src/vive-vr-ros2
vcs import src < src/vive-vr-ros2/dependencies.repos
./src/vive-vr-ros2/scripts/fetch_vive_plugin.sh     # 361 MB, untracked on purpose

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Then export a world and run it:

```bash
ros2 run vive_vr_ros2 scene_export \
    ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml
ros2 launch vive_vr_ros2 sim.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml
```

For poses, buttons, hands and gaze without a simulation — no model, no world on the headset:

```bash
ros2 launch vive_vr_ros2 tracking.launch.py
```

And the headset half, on any machine with the Unity editor installed:

```bash
./scripts/build_apk.sh --setup --install --logcat    # build, install, launch, tail the log
```

## Documentation

- [Installation](docs/install.md) — workspace dependencies, ROS build, Unity editor and licence
- [Running](docs/run.md) — export a world, launch the stack, what should appear
- [Interfaces](docs/interfaces.md) — topics, TF frames, `Joy` layout, `vr/EyeGaze`, frame conventions
- [Configuration](docs/configuration.md) — `vive_vr.yaml`, the device config file, OpenXR features
- [The Unity client](docs/unity.md) — project layout, scripts, scripted setup and APK build
- [Embedding in a simulation](docs/embedding.md) — `BodyPosePublisher` in an existing application
- [First run on the headset](docs/bringup.md) — the on-device checklist
- [Testing](docs/testing.md) — what is verified, how, and what is not
- [Design notes](docs/design.md) — why the system is shaped this way, and what was rejected
- [Teleoperation](docs/teleop.md) — the clutch, the delta stream, and why it needs no calibration
- [Known limits](docs/limits.md) — what is missing or approximate

## Status

The PC side works and is verified by running it. **Nothing has run on the headset yet** — see
[Testing](docs/testing.md) for which claims rest on evidence, and
[First run](docs/bringup.md) for the on-device checklist.

MIT licensed.
