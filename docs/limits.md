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
the result into `vr.yaml`. Not built.

## No teleop

`InputNode` publishes calibrated poses, TF and activity, and the grabber uses them to move
simulated bodies directly — but nothing drives a *robot*. That component is deliberately last,
because retargeting a hand onto an arm is where the clutch and the solvers come in.

## Clock offset is one-way

The client estimates its offset from `/vr/pc_time` and keeps the sample with the smallest
observed difference, which removes queueing jitter but not the constant one-way transit time.
Stamps are therefore biased late by roughly half the round trip — single-digit milliseconds on
5/6 GHz Wi-Fi.

For rendering this is irrelevant. For a demonstration dataset it is a small systematic error in
every recorded controller pose. The upgrade is a round trip: publish a ping the PC echoes with
both stamps, about thirty lines, documented in `ClockSync.cs`.

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

`/vr/body_poses` is JSON over rosbridge: about 500 kB/s for 100 bodies at 60 Hz, which is fine.
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

Listed in full in [Testing](testing.md#what-is-not-established). The short version: the
handedness conversion, `ClientWebSocket` under IL2CPP, the glTF correction sign and eye tracking
permissions are all unexercised, because nothing has run on the device yet.
