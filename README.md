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
│ vr::SceneNode  mj_step ──────┼──── 60 Hz ───────▶│ Body_<name> transforms   │
│ vr::InputNode         ◀──────┼──── 90/60 Hz ─────┤ grip pose + 5 buttons    │
│   calibrate, TF, activity    │                   │ hand joints, eye gaze    │
│ rosbridge_websocket          │                   │ Unity 6 + VIVE OpenXR    │
└──────────────────────────────┘                   └──────────────────────────┘
```

## Quick start

```bash
./tools/fetch_vive_plugin.sh                      # 361 MB, untracked on purpose
cp -r ~/work/ms/src/mj_kdl_wrapper ~/work/ms/src/orocos_kinematics_dynamics src/
source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
ros2 run vr scene_export ~/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml -o /tmp/vr_robot
ros2 launch vr vr.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml \
    scene_dir:=/tmp/vr_robot host_ip:=<this machine's LAN IP>
```

## Documentation

- [Installation](docs/install.md) — workspace dependencies, ROS build, Unity editor and licence
- [Running](docs/run.md) — export a world, launch the stack, what should appear
- [Interfaces](docs/interfaces.md) — topics, TF frames, `Joy` layout, `vr/EyeGaze`, frame conventions
- [Configuration](docs/configuration.md) — `vr.yaml`, the device config file, OpenXR features
- [The Unity client](docs/unity.md) — project layout, scripts, scripted setup and APK build
- [Embedding in a simulation](docs/embedding.md) — `BodyPosePublisher` in an existing application
- [First run on the headset](docs/bringup.md) — the on-device checklist
- [Testing](docs/testing.md) — what is verified, how, and what is not
- [Design notes](docs/design.md) — why the system is shaped this way, and what was rejected
- [Known limits](docs/limits.md) — what is missing or approximate

## Status

The PC side works and is verified by running it. **Nothing has run on the headset yet** — see
[Testing](docs/testing.md) for which claims rest on evidence, and
[First run](docs/bringup.md) for the on-device checklist.

## Layout

```
src/vr/                        one package, composable nodes
  msg/EyeGaze.msg              the only custom type, because ROS 2 has no gaze message
  config/vr.yaml               every topic, frame, rate and the calibration
  include/vr/, src/            BodyPosePublisher, glTF writer, geom tessellation, scene_export
  src/components/              vr::SceneNode, vr::InputNode
  launch/vr.launch.py
unity/VrRos/Assets/            the headset client; see docs/unity.md
tools/
  fetch_vive_plugin.sh         downloads the untracked VIVE OpenXR tarball
  fake_headset.py              stands in for the client; exercises every topic both ways
  check_glb.py                 structural validation of an exported .glb
docs/
```

MIT licensed.
