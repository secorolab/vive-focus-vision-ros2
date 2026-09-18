# vive_vr_ros2

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

This repository is one ROS 2 package, cloned into a workspace rather than built in place:

```bash
mkdir -p ~/work/p/vrws && cd ~/work/p/vrws
git clone git@github.com:secorolab/vive-focus-vision-ros2.git src/vive-vr-ros2
vcs import src < src/vive-vr-ros2/dependencies.repos

python3 -m venv --system-site-packages venv && source venv/bin/activate
./src/vive-vr-ros2/scripts/setup.sh

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SCENES=ON
source install/setup.bash
```

`BUILD_SCENES=ON` exports the worlds in `scenes/` — a Kinova arm on a table, and a RoboCasa
kitchen — into `~/.cache/vive_vr_ros2/scenes/`. Then run one:

```bash
ros2 launch vive_vr_ros2 sim.launch.py \
    model:=$HOME/.cache/mj_kdl_wrapper/menagerie/kinova_gen3/scene.xml
```

For poses, buttons, hands and gaze without a simulation — no model, no world on the headset:

```bash
ros2 launch vive_vr_ros2 tracking.launch.py
```

And the headset half, on any machine with the Unity editor installed:

```bash
cd ~/work/p/vrws/src/vive-vr-ros2
python3 scripts/fetch_vive_plugin.py
./scripts/build_apk.sh --setup --install --logcat    # build, install, launch, tail the log
```

## OpenArm V1 teleoperation

The OpenArm application drives the **right arm in MuJoCo** from the right controller.
Orocos KDL solves all seven arm joints for the gripper position and orientation; the left arm
holds its home pose. This is a motion-test simulation with gravity and contacts disabled,
not physical robot control or a grasping simulation. Your elbow is not tracked, so the robot
can use a different elbow posture to reach the same gripper pose.

The commands below use **Zsh**, `~/vive_vr_ws` for the ROS workspace, and
`~/openarm_ws` for the OpenArm description workspace. The source checkout is
`~/vive_vr_ws/src/vive-vr-ros2` (a symlink to a Desktop checkout also works).
For Bash, use `setup.bash` instead of `setup.zsh`. Install the base dependencies using
[Installation](docs/install.md) first. The helper expects the OpenArm V1 description at
`~/openarm_ws/src/openarm_description`; see [OpenArm setup](docs/openarm_sim.md) for path overrides.
An existing compatible headset app works without an APK rebuild for these PC-side changes.

### Build and prepare once

```bash
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
cd ~/vive_vr_ws
colcon build --packages-select vive_vr_ros2 \
  --cmake-args -DBUILD_SCENES=OFF -DBUILD_OPENARM_SIM=ON
source ~/vive_vr_ws/install/setup.zsh

cd ~/vive_vr_ws/src/vive-vr-ros2
python3 scripts/openarm_sim.py prepare
```

`prepare` generates the robot models, scene export and configuration under
`~/vive_vr_ws/models/openarm_v1`, outside the source checkout. It preserves a saved controller
alignment. Rebuild after C++ changes; re-run `prepare` when the generated model/configuration
needs updating. The OpenArm source description is not modified.

### Start and align with one terminal

Stop any previous simulation or tracking launch with **Ctrl+C**, then:

```bash
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
export ROS_DOMAIN_ID=42
cd ~/vive_vr_ws/src/vive-vr-ros2

python3 scripts/openarm_sim.py launch --teleop --align --delay 10
```

1. Put on the headset during the 10-second delay and open the VR app.
2. Keep the right side grip released. Hold your hand down beside you, matching the hanging
   robot gripper, with your chosen controller tip direction pointing down.
3. Hold still for at least one second. The script captures alignment and automatically restarts
   the stack. Wait for the headset to reconnect and the robot to reappear.
4. Match the gripper orientation, then press and hold the right side grip. Start with a small
   hand movement and wrist rotation. Release the grip to hold position.

Keep the terminal open while using VR. This one launch runs the robot simulation, ROS bridge,
discovery server and scene server. **Ctrl+C stops them all.** There is no green indicator or
additional headset plugin required for this workflow.

For later sessions with the same controller grip and orientation pairing, reuse the saved
alignment instead of capturing again:

```bash
python3 scripts/openarm_sim.py launch --teleop
```

| Input | Effect while the side grip is held |
|---|---|
| Move the right hand | Move the gripper target at 1:1 translation scale. |
| Rotate the wrist | Rotate the gripper target; 90-degree turns fit within the 100-degree bound per press. |
| Pull/release the index trigger | Close/open the gripper after the initial trigger sample is captured. |
| Release the side grip | Hold the arm; re-grip starts a new relative movement. |

Robot reach, joint limits and speed limits still apply. The generated configuration bounds
translation to 1 m per press. Engagement requires a saved alignment and fresh controller
orientation within about 11 degrees of the robot gripper. If it does not engage, release the
side grip, match the gripper orientation and press again. Unreachable targets hold the arm;
move closer or release and re-grip. Position is relative to each grip press, not an absolute
hand-to-robot position calibration.

### Connection and diagnostics

The headset and PC must be able to reach each other on the local network. If the app stays on
“searching for PC,” check that the launch terminal is still running and reports
`Rosbridge WebSocket server started on port 9090`. Discovery uses UDP 9091; scene downloads
use TCP 8000. Stop duplicate launches before starting another.

If automatic address selection chooses the wrong network interface, append
`--host-ip YOUR_PC_LAN_IP` to the launch command. This selects the scene download address;
the headset's ROS bridge address is discovered separately or set in `vr_config.json`.
Use the same `ROS_DOMAIN_ID` in any additional ROS terminals.

From another sourced terminal, these commands inspect input or reset the simulated robot:

```bash
python3 scripts/openarm_sim.py inputs
python3 scripts/openarm_sim.py reset
```

Release the side grip before resetting. For configuration, manual alignment, model details
and test commands, see [OpenArm simulation](docs/openarm_sim.md).

## Headset settings

Everything the client can be told without rebuilding the APK lives in one file, on the headset:

```
/storage/emulated/0/Android/data/de.uni_bremen.secoro.vrros/files/vr_config.json
```

It does not exist until the app has run once, and it is read in `VrConfig.Awake()`, so edits
apply on the next start:

```bash
adb pull /storage/emulated/0/Android/data/de.uni_bremen.secoro.vrros/files/vr_config.json
adb push vr_config.json /storage/emulated/0/Android/data/de.uni_bremen.secoro.vrros/files/
adb shell am force-stop de.uni_bremen.secoro.vrros
```

Worth knowing: `spawnPosition` and `spawnYawDegrees` put you somewhere else in a world that
declares no spawn point of its own,
`eyeHeight` makes the table the right height, `inputMode` picks controllers or hands, and the
`*RateHz` fields throttle the uplink. `host` is a cache the app maintains itself — discovery
overwrites it whenever the configured address fails, so pin it only to choose between PCs that
could both answer, or set `discoveryPort` to `0` to forbid discovery entirely. Full field table
in [Configuration](docs/configuration.md#headset-settings).

## Documentation

- [Installation](docs/install.md) — workspace dependencies, ROS build, Unity editor and licence
- [Running](docs/run.md) — export a world, launch the stack, what should appear
- [Interfaces](docs/interfaces.md) — topics, TF frames, `Joy` layout, `EyeGaze`, frame conventions
- [Configuration](docs/configuration.md) — `vive_vr.yaml`, the device config file, OpenXR features
- [The Unity client](docs/unity.md) — project layout, scripts, scripted setup and APK build
- [Embedding in a simulation](docs/embedding.md) — `BodyPosePublisher` in an existing application
- [First run on the headset](docs/bringup.md) — the on-device checklist
- [Testing](docs/testing.md) — what is verified, how, and what is not
- [Design notes](docs/design.md) — why the system is shaped this way, and what was rejected
- [OpenArm simulation](docs/openarm_sim.md) — right-arm IK, alignment, controls and tests
- [Teleoperation](docs/teleop.md) — the clutch, the delta stream, and why it needs no calibration
- [Known limits](docs/limits.md) — what is missing or approximate

## Status

The Focus Vision client has connected to the PC, downloaded the OpenArm scene, streamed
controller data and supplied a saved controller/tool alignment. Automated checks cover IK,
alignment math and capture, clutch gating, limits, timeouts and calibration-launch cleanup.
Full headset motion testing across the workspace remains to be completed; physical hardware
control and contact-based grasping are not implemented by the OpenArm bridge.
See [OpenArm verification](docs/openarm_sim.md#verification-and-next-work) and
[Testing](docs/testing.md) for the scope of the checks.

MIT licensed.
