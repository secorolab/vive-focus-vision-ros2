# First run on the headset {#page_bringup}

Nothing in this project has run on the device yet. Everything below is therefore a checklist to
work through, not a description of something known to work — see [Testing](testing.md) for what
*is* established.

Each step has a check that is not "it built".

## 1. Install

Developer mode on, headset connected by USB — the **right-side** USB-C port, since the rear one
is charge-only and produces an empty `adb devices`:

```bash
./tools/build_apk.sh --setup --install --logcat
```

That builds, installs, launches and tails the log. By hand it is:

```bash
adb devices                                   # accept the prompt inside the headset
adb install -r unity/VrRos/Build/VrRos.apk
adb logcat -s Unity                           # keep this open; the client logs here
```

Over Wi-Fi instead of a cable: `adb tcpip 5555` once over USB, then
`adb connect <headset-ip>:5555`, which is lost on reboot. Use `ANDROID_SERIAL` when both
transports are attached.

**An app installed before 2026-09-17 is not replaced by this build.** The application id changed
to `de.uni_bremen.secoro.vrros`, and Android treats a different id as a different application, so
the old one sits alongside it with its own config file. `adb uninstall sh.vamsi.vrros` clears it;
`build_apk.sh` warns when it sees it.

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
headset* (the `host_ip` default follows this machine's default route, which is the wrong one if
the headset is on another interface), and that `/vr/scene` is being published.

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

Half built: `vr::TeleopNode` publishes the clutch, the hand delta and the gripper, and nothing
consumes them yet. To see the stream, with `teleop_node` running:

```bash
ros2 topic echo /vr/teleop/right/clutch     # grip button, held
ros2 topic echo /vr/teleop/right/delta      # motion since the press
ros2 topic echo /vr/teleop/right/gripper    # trigger pull, 0 to 1
```

It does **not** need the play-space calibration, which an earlier version of this page said it
did: a delta taken against a reference in the same frame cancels the unknown transform exactly.
See [Teleoperation](teleop.md). What is still missing is the other end — `mj_kdl_wrapper`'s
IK/ACHD solvers and `Robot::jnt_pos_cmd` — and a measured `tool_from_controller_rpy`.

## Most likely failures, in order

1. **`ClientWebSocket` under IL2CPP on ARM64.** Chosen to avoid an extra package dependency, and
   the one runtime behaviour a successful compile does not establish. The fallback is
   [NativeWebSocket](https://github.com/endel/NativeWebSocket), a change confined to
   `RosBridge.cs`.
2. **The glTF correction sign** — step 3 above, a one-field fix.
3. **Network reachability** — headset on a different Wi-Fi network or band from the PC, or a
   firewall on ports 9090/8000.
4. **Eye tracking** needing calibration or a permission nobody has set.
