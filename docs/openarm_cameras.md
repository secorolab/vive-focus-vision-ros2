# Real robot camera previews

Run the camera service in a separate terminal:

```sh
cd ~/Desktop/vive-focus-vision-ros2
/usr/bin/python3 scripts/openarm_cameras.py
```

It selects the D455f main camera and D405 right-wrist color camera using stable USB device paths. Both capture 640×480 color at 30 frames/second. Close other applications using these cameras first. The Python dependency is `python3-opencv`.

Open http://localhost:8081 on the laptop to check both images. Keep the service running alongside your usual teleoperation launch. The updated VR APK requests previews from the ROS bridge PC address on TCP port 8081, so the headset and PC must be able to reach each other over the LAN. Use this on your trusted local network; the preview service has no authentication.

Three head-following panels appear when an image arrives. While the world is unlocked, they follow headset position and rotation, staying in front of your eyes. Locking the world freezes their placement. They do not follow eye-gaze independently of head movement. A missing or stale stream turns black and is labelled CAMERA OFFLINE. This preview does not change any controller mappings or robot settings.

Install the updated APK with the headset connected over USB:

```sh
adb install -r unity/VrRos/Build/VrRos.apk
```

Then restart the VR app. No ROS rebuild is needed for the camera preview.

Optional camera overrides: `--main /dev/v4l/by-id/... --right /dev/v4l/by-id/...`. The VR client uses port 8081. Main and right cameras are retried automatically when disconnected.

Without `--record-root`, this is live viewing only. For optional right-arm episode recording, see `openarm_recording.md`. No model is downloaded or trained.

## Operator layout

The real configuration enables `operator_view: true`. The headset eyes start at approximate body-camera coordinates
ROS (-0.10, 0, 1.0) metres, facing robot +X; robot +Y is on the operator's left.
This overrides the older viewing orbit for the real scene only. It is a starting
viewpoint, not a calibration between your body and the robot. Simulation retains
its existing spawn. Set `operator_view: false` in the real configuration to restore
the previous view.

The main panel is centered and larger; the left wrist (if connected) is left and the right wrist is right. Panels follow your head while WORLD FREE and stay fixed when WORLD LOCKED. Offline streams go black and display CAMERA
OFFLINE; other streams continue independently.

A camera present at startup keeps its stable device identity across reconnection.
An initially absent main/right camera is rediscovered periodically. When using two
D405 cameras, specify both stable paths explicitly to avoid ambiguous assignment:

```sh
/usr/bin/python3 scripts/openarm_cameras.py \
  --wrist /dev/v4l/by-id/LEFT_CAMERA_COLOR_DEVICE \
  --right /dev/v4l/by-id/RIGHT_CAMERA_COLOR_DEVICE
```

Replace those placeholders with the actual color-device paths. The single connected D405 is assigned to the right wrist by default. Restart the camera service when changing arguments.
Install the updated APK to apply the operator layout. The client also recognizes
the existing real launch using its controlled OpenArm model and 90-degree viewing
orbit, so the currently installed ROS package does not need rebuilding. An explicit
`operator_view` field, when available, takes precedence over that compatibility rule.
Joystick locomotion is disabled in body view, while normal head tracking remains.
The eye position is an approximation, not measured camera extrinsics; adjust
`RobotEyeRos` in VrLocomotion.cs to match the mounting. No recording or motor commands are added by this feature.

### Place and lock the panels

In robot-body view, panels start 1.2 m ahead. While WORLD FREE, center the left
stick first, then push up to move the panels farther away or down to bring them
closer (0.75–2.5 m). Panels follow the headset while unlocked. Tap the left menu
button to lock the world: the panels retain their world position and distance.
The locked sticks retain their existing gripper controls; distance adjustment is
inactive. Unlocking restores head-following panels at the chosen distance.
Distance is retained for this app session, not saved across app restarts.
