# Known limits {#page_limits}

What is missing, approximate, or true only under conditions worth knowing about.

## Play-space calibration

`origin_xyz` and `origin_rpy` are identity, which means the headset's guardian origin and the
robot's world frame are the same place only by coincidence. Everything downstream inherits this:
a controller pose in the `world` frame is only as meaningful as that transform.

It does not matter for looking at a simulated world, where the user can stand anywhere. It matters
completely for teleoperating a real robot, or for any dataset that claims the operator's hand and
the robot were in one metric frame.

The fix is a calibration procedure — touch three known points, or align a tracked object — writing
the result into `vive_vr.yaml`. Not built.

## No teleop

Half of it exists. `vive_vr_ros2::TeleopNode` publishes the clutch and the hand delta per arm in the tool
frame ([Teleoperation](teleop.md)), verified against `scripts/teleop_check.py` — but **nothing
consumes it**. There is no solver, no joint-limit handling, no arm. The delta stream is an offer,
and until something takes it up, no robot moves.

`tool_from_controller_rpy` is also still identity, which is wrong for any real gripper; the value
has to be observed rather than derived.

## Clock offset is one-way, and a suspend poisons it

The client estimates its offset from `/vive_vr/pc_time` and keeps the sample with the smallest
observed difference, which removes queueing jitter but not the constant one-way transit time.
From a cold start this works: measured stamp lag is **20–30 ms, stable over 70 s**.

Suspending the app breaks it. Take the headset off and the client stops running while its socket
keeps receiving, so every `/vive_vr/pc_time` sample it then processes is stale by the depth of the
backlog, and that staleness is baked into the offset. Measured after a few minutes off the head:
stamps **158 s in the past**, recovering only as fast as the backlog drains — about 0.7 s per
second. `tf2` rejects everything in the meantime with `TF_OLD_DATA`.

The 30 s re-estimate does not save it, because it re-samples the same backlogged stream. The fix
is to drop stale frames rather than process them — the newest message on `/vive_vr/pc_time` and
`/vive_vr/body_poses` is the only one that matters. Not built.

## The floor is assumed, not measured

The client asks for device-space tracking so that it runs anywhere without a play area being
drawn. Nothing then measures the floor: the ground sits `eyeHeight` below wherever the headset
was when the app started, and X recentres it.

This is not a choice between good options. The runtime offers only `VIEW`, `LOCAL` and `STAGE` —
it does not implement `XR_EXT_local_floor`, which is the one that would give a real floor without
a stage. And its `STAGE` is not a fallback: with no boundary configured it reports `floor bound
enable false` and puts the stage origin at the headset, which was measured putting the user's
head at **z = 0.039 m**. So floor space costs a play area and still does not measure a floor.

## Losing the boundary stops input entirely

If the headset decides it is outside its boundary — moving to a new room does it — the runtime
hands input focus to its own system overlay. OpenXR only delivers action and tracking data to a
focused session, so the app then receives no buttons, no hand joints, and about 5 fps.

Nothing in this project can prevent or detect that beyond observing the silence. It presents as
intermittent grabbing rather than an outright failure, because focus bounces back and forth, and
a grab held across one of those windows dies mid-lift. The symptom to recognise is
`notifyOutOfBoundary() out = true` and `focusCapturedBySystem = true` in `adb logcat`.

## Rendering fidelity

- **Materials are flat colours.** Mesh textures in the MJCF are not exported; the exporter reports
  how many it skipped. A textured finite plane is the exception, tiled as a checkerboard.
- **No shadows.** The lights are exported, but whether anything casts a shadow is a client render
  setting.
- **Light intensity is a guess.** glTF measures directional lights in lux and the others in
  candela, while MuJoCo's `light_diffuse` is a 0–1 weight; there is no principled conversion, so
  the exporter uses values that look right.
- **Normals are flat-shaded**, which triples the vertex count — the Kinova scene is 87k triangles
  in a 7.3 MB file. Smooth normals with indexed deduplication would shrink it substantially.
- **No lighting model** beyond whatever the glTF material and the Unity scene provide.

## Bandwidth

`/vive_vr/body_poses` is JSON over rosbridge: about 500 kB/s for 100 bodies at 60 Hz, which is fine.
Scenes with hundreds of bodies should move that topic to rosbridge's CBOR compression before
concluding the renderer is slow.

Hand TF is the heaviest stream in the system: 26 joints × 2 hands × 60 Hz is 3120 transforms a
second, and every `tf2` listener in the graph pays for it. `publish_hand_tf: false` turns it off.

## Missing interfaces

- **No haptics.** `/vr/{left,right}/haptic` does not exist; the controllers can only send.
- **Grip pose only.** The aim pose — the pointing ray, for UI — is not published.
- **No wrist trackers.** The Focus Vision supports them; nothing here reads them.
- **Eye data is partial.** Gaze and pupil diameter are published; the runtime's geometric data
  (openness, squeeze, wide) is available and unused.

## Verification gaps

Listed in full in [Testing](testing.md#what-is-not-established). The client now runs on the
headset, which settled the handedness conversion, `ClientWebSocket` under IL2CPP and the glTF
correction sign. Eye tracking permissions and anything quantitative about latency and comfort
remain unexercised.
