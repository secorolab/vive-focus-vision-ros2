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

`SetupAll` sets IL2CPP, ARM64 only, min SDK 29, app id `sh.vamsi.vrros`, splash off;
builds the scene by invoking the same `GameObject/XR/XR Origin (VR)` menu command a human would;
creates the `VrRos` object and its `VrScene` child; and wires every inspector reference.

```
XR Origin (VR)          camera, camera offset, tracked pose driver,
                        VrLocomotion, VrPointer, VrDeviceVisuals, VrWelcomePanel
VrRos                   VrConfig, RosBridge, ClockSync, VrInputPublisher,
                        BodyPoseApplier, HandPublisher, GazePublisher
  └── VrScene           SceneLoader — parents the loaded world under its own transform
```

It also imports TextMeshPro's Essential Resources if they are missing, which the welcome panel
needs and which Unity otherwise asks for through a dialog.

## Before a world arrives

With no `<out>/scene` message the client shows a panel in front of the user — rosbridge address
and connection state, input mode, the launch commands, and the path of the device config file —
standing on a metre grid. It hides when a world loads and returns if the bridge drops, so an
empty headset view always says which of the three possible failures it is.

## Scripts

| Script | Responsibility |
|---|---|
| `VrConfig` | reads `vr_config.json` from the device; see [Configuration](configuration.md) |
| `RosBridge` | rosbridge v2 over one WebSocket: advertise, publish, subscribe, reconnect |
| `VrDiscovery` | a UDP broadcast asking the PC for its address, when the configured one fails |
| `FrameConv` | the only place handedness is converted |
| `ClockSync` | estimates the offset between the headset clock and the PC's ROS clock |
| `VrInputPublisher` | head and controller grip poses, buttons and axes |
| `HandPublisher` | 26 joints per hand via `XRHandSubsystem` |
| `GazePublisher` | per-eye gaze and pupil via `ViveEyeTracker` |
| `SceneLoader` | `<out>/scene` → HTTP fetch → glTFast → one holder per body, plus the ground |
| `BodyPoseApplier` | `<out>/body_poses` → transforms |
| `VrLocomotion` | stick locomotion, the spawn point and recentring |
| `VrPointer` | the selection ray, the highlight, and `<raw>/<hand>/target` |
| `VrBodyTag` | a body's manifest index, so a raycast hit can be named |
| `VrDeviceVisuals` | VIVE's own controller and hand models, one set or the other |
| `VrSimControls` | buttons that act on the simulation rather than the rig |
| `VrWelcomePanel` | connection, input mode and what to run on the PC, until a world arrives |

### Controls

| Input | Action |
|---|---|
| Left stick | walk (push) and turn (tilt) |
| Left X | recentre: back to spawn, eyes at `eyeHeight` |
| Left Y | reset the simulation |
| Left menu | switch controllers ↔ hands |
| Right A / B | up / down |
| Grip, or a pinch in hand mode | grab whatever the ray is on |

One stick does both walking and turning. Turning is continuous rather than snapped; snap turn is
gentler on motion sickness, but it was not what this reads well as in practice.

All of it is read through `UnityEngine.XR.InputDevices` rather than the XR Interaction Toolkit's
locomotion providers, so there is one input path and no action assets to keep in sync.

Because the rig moves, every published pose is transformed by it before being sent — a device
pose alone is in tracking space and would ignore where the user walked to. That transform is the
**camera offset**, not the XR Origin root: `XROrigin` puts the eye height on the offset object in
device space, so converting through the root leaves every pose a metre and a half out.

### Pointing at things

A ray leaves each hand, whatever it lands on is tinted, and that body's manifest index goes to
the PC on `<raw>/<hand>/target`. Grabbing by proximity alone was unusable in a headset: it wanted
the controller inside a 15 cm bubble around a 6 cm cube, with nothing on screen saying whether
you were close enough.

The ray is deliberately forgiving, because hand tracking is not precise enough for a bare ray at
arm's length: it is a 3 cm sphere cast, the aim is low-pass filtered, a new body has to hold the
ray for four frames before the selection moves to it, and a tracking dropout keeps the last aim
for a quarter of a second.

In hand mode the ray is aimed **from the eye through the pinch point**, not along the hand. The
hand's own axes run along the back of the hand and sit well above where the user believes they
are pointing; sighting through the fingers is what makes it land where it looks like it should.
The origin is still the pinch point, so the ray appears to leave the fingers.

Two colours, and the difference matters: **yellow** is what the ray is on, **green** is what the
PC reports actually holding on `<out>/<hand>/held`. They disagree whenever the grabber refuses a
body — a static one such as a table has no mass — so you can see the refusal rather than wonder
why nothing moved. Green is applied optimistically the moment the button goes down and corrected
by the next `/held`, because waiting for the round trip on this link was visible.

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

## Checking a world without a headset

`VrScenePreview` imports an exported `.glb` and either renders it to a PNG or saves it as a scene
to open:

```bash
DISPLAY=:0 VR_GLB=/tmp/vr_robot/scene.glb VR_PNG=~/out.png \
  $UNITY -batchmode -quit -projectPath unity/VrRos -executeMethod VrScenePreview.Render
```

`-nographics` cannot be used: it disables the graphics device, so the camera renders nothing and
the PNG comes out blank.

The welcome panel is checked the same way, from where the user's eyes will be, with no world
loaded and nothing connected:

```bash
DISPLAY=:0 VR_PNG=~/welcome.png \
  $UNITY -batchmode -quit -projectPath unity/VrRos -executeMethod VrScenePreview.RenderWelcome
```

With an Editor open, the Unity CLI drives it directly, which is faster and shows the real thing:

```bash
unity status                                     # look for state "ready"
unity command open_scene --path Assets/Scenes/Preview.unity
unity command capture_game_view --width 1280 --height 800 --save_path Temp/shot.png
```

This needs `com.unity.pipeline` in the project (`unity pipeline install`, already done) and an
Editor that has focused at least once since. Capture paths must be inside the project.

Bear in mind the Editor scene is not the headset: it has its own lighting environment, so
brightness there says little about what VR will look like.

## Running the app on the PC

The same scene builds as a desktop player, which is the quickest way to watch the app actually
start: the welcome panel, the rosbridge connection and discovery all behave as they do on the
device, in a window.

```bash
$UNITY -batchmode -quit -nographics -projectPath unity/VrRos \
  -buildTarget Linux64 -executeMethod VrRosSetup.BuildLinux   # -> Build/Linux/VrRos

ros2 launch vr tracking.launch.py &
./unity/VrRos/Build/Linux/VrRos -screen-width 1600 -screen-height 900 -screen-fullscreen 0
```

Unlike the renders above this is a real player, so it writes and reads its own
`vr_config.json` — under `~/.config/unity3d/vamsi/VrRos/`, not on any device — and it will
discover and save the PC's address there like any other client.

Standalone has no XR loader configured, which is what makes this work: XR never initialises, so
the camera renders flat instead of waiting for an OpenXR runtime that a PC does not have. The
cost is that nothing is head-tracked and no controller exists, so this shows startup, networking
and anything on the panel — not interaction. Eye gaze is compiled out entirely: VIVE's assembly
does not cover Linux, so `GazePublisher` logs that it has no tracker and publishes nothing.

## Deploying

```bash
adb devices                                   # accept the prompt inside the headset
adb install -r unity/VrRos/Build/VrRos.apk
adb logcat -s Unity                           # the client's own logs
```

See [First run on the headset](bringup.md) for what to check once it is installed.
