# First run on the headset {#page_bringup}

Nothing in this project has run on the device yet. Everything below is therefore a checklist to
work through, not a description of something known to work — see [Testing](testing.md) for what
*is* established.

Each step has a check that is not "it built".

## 1. Install

Developer mode on, headset connected by USB:

```bash
adb devices                                   # accept the prompt inside the headset
adb install -r unity/VrRos/Build/VrRos.apk
adb logcat -s Unity                           # keep this open; the client logs here
```

## 2. Controller input

Start the stack ([Running](run.md)), launch the app, then on the PC:

```bash
ros2 topic echo /vr/right/joy
```

Press each button in turn and watch the corresponding index flip — the same `101001`-shaped
output the stand-in client produces, but from real presses. Then:

```bash
ros2 topic hz /vr/right/pose          # roughly the display rate
ros2 topic echo /vr/right/active      # true while held, false a second after putting it down
```

If poses arrive but every button reads 0, the **VIVE Focus 3 Controller Interaction Profile** is
not enabled; see [Configuration](configuration.md#openxr-features).

## 3. Static world

The world should appear once the client fetches the `.glb`. If it does not, check in order: that
`adb logcat` shows the fetch URL and no HTTP error, that the URL's host is reachable *from the
headset* (`host_ip` must not be `0.0.0.0`), and that `/vr/scene` is being published.

**If the world appears rotated 180°**, negate the Y component of `SceneLoader.gltfCorrectionEuler`.
±90° are the only two possibilities — see
[Interfaces](interfaces.md#coordinate-conventions) for why.

## 4. Live stream

Objects should move. Verify from a rosbag replay against sim state rather than by eye:

```bash
ros2 bag record /vr/body_poses /vr/right/pose /vr/right/joy /tf
```

## 5. Hands and gaze

```bash
ros2 run tf2_tools view_frames        # both hand skeletons, 26 joints each
ros2 topic echo /vr/gaze
```

Hands that never appear usually mean the **VIVE XR Hand Tracking** feature is off — the published
symptom is hands rendering while `isTracked` stays false.

Gaze reporting `left_valid: false` most likely means eye tracking is not enabled or not
calibrated in the headset's own settings; `GazePublisher` logs a specific warning for this rather
than going quiet. Whether an Android permission is also required is **unverified** — no permission
string exists anywhere in the VIVE plugin, but that is not proof.

## 6. Teleop

Not built. This is where `mj_kdl_wrapper`'s IK/ACHD solvers and `Robot::jnt_pos_cmd` come in, and
it needs the play-space calibration from [Known limits](limits.md#play-space-calibration) to be
meaningful.

## Most likely failures, in order

1. **`ClientWebSocket` under IL2CPP on ARM64.** Chosen to avoid an extra package dependency, and
   the one runtime behaviour a successful compile does not establish. The fallback is
   [NativeWebSocket](https://github.com/endel/NativeWebSocket), a change confined to
   `RosBridge.cs`.
2. **The glTF correction sign** — step 3 above, a one-field fix.
3. **Network reachability** — headset on a different Wi-Fi network or band from the PC, or a
   firewall on ports 9090/8000.
4. **Eye tracking** needing calibration or a permission nobody has set.
