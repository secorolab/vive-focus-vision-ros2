# The Unity client {#page_unity}

The project lives in `unity/VrRos/`. It was created and configured headlessly, and it stays that
way: `Assets/Editor/VrRosSetup.cs` is the source of truth for the scene, the component wiring and
the Android settings. None of it is a list of things to click.

## Building

```bash
UNITY=~/Unity/Hub/Editor/6000.0.83f1/Editor/Unity

# scene, wiring and Android player settings
VR_HOST=192.168.2.118 $UNITY -batchmode -quit -nographics \
  -projectPath ~/work/p/vr/unity/VrRos -buildTarget Android -executeMethod VrRosSetup.SetupAll

# -> Build/VrRos.apk
$UNITY -batchmode -quit -nographics -projectPath ~/work/p/vr/unity/VrRos \
  -buildTarget Android -executeMethod VrRosSetup.BuildApk
```

**Close the Editor first.** Only one Unity instance may hold a project; otherwise these fail on
`Temp/UnityLockfile`. The same script also adds a `VrRos` menu to the Editor menu bar (*Setup
Scene and Android Settings*, *Build APK*) for when the Editor is already open — that path does not
read `VR_HOST`, so set the host in the inspector or in the device config instead.

`SetupAll` sets IL2CPP, ARM64 only, min SDK 29, app id `sh.vamsi.vrros`, splash off; builds the
scene by invoking the same `GameObject/XR/XR Origin (VR)` menu command a human would; creates the
`VrRos` object and its `VrScene` child; and wires every inspector reference.

```
XR Origin (VR)          camera, camera offset, tracked pose driver
VrRos                   VrConfig, RosBridge, ClockSync, VrInputPublisher,
                        BodyPoseApplier, HandPublisher, GazePublisher
  └── VrScene           SceneLoader — parents the loaded world under its own transform
```

## Scripts

| Script | Responsibility |
|---|---|
| `VrConfig` | reads `vr_config.json` from the device; see [Configuration](configuration.md) |
| `RosBridge` | rosbridge v2 over one WebSocket: advertise, publish, subscribe, reconnect |
| `FrameConv` | the only place handedness is converted |
| `ClockSync` | estimates the offset between the headset clock and the PC's ROS clock |
| `VrInputPublisher` | head and controller grip poses, buttons and axes |
| `HandPublisher` | 26 joints per hand via `XRHandSubsystem` |
| `GazePublisher` | per-eye gaze and pupil via `ViveEyeTracker` |
| `SceneLoader` | `<out>/scene` → HTTP fetch → glTFast → one holder per body |
| `BodyPoseApplier` | `<out>/body_poses` → transforms |

### Vendor-neutral input

Controllers are read through `UnityEngine.XR.InputDevices` and `CommonUsages`, and hands through
Unity's `XRHandSubsystem` — never VIVE's own gesture API. The VIVE plugin maps the Focus 3
interaction profile onto those same usages and implements the hand subsystem provider, so the
same code runs unchanged on another OpenXR headset. Only `GazePublisher` is vendor-specific,
because eye tracking has no cross-vendor Unity abstraction.

### Serialization

Outgoing messages are built as strings with a cached `StringBuilder` rather than through a JSON
object model. The publish path runs at display rate, and serializing through `JObject` there
allocates every frame. Incoming messages are parsed with Newtonsoft, where the cost is paid once
per received message rather than 90 times a second.

All floats are formatted with `CultureInfo.InvariantCulture`: a device set to a comma decimal
separator would otherwise emit JSON that is not JSON.

### Threading

The WebSocket receive loop runs on a background task and pushes raw frames into a
`ConcurrentQueue`. Handlers run in `Update`, on the main thread, because they touch `Transform`s.

## Deploying

```bash
adb devices                                   # accept the prompt inside the headset
adb install -r unity/VrRos/Build/VrRos.apk
adb logcat -s Unity                           # the client's own logs
```

See [First run on the headset](bringup.md) for what to check once it is installed.
