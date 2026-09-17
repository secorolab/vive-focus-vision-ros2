# Testing {#page_testing}

## Tools

```bash
python3 scripts/fake_headset.py --seconds 10     # with the stack running
python3 scripts/fake_headset.py --still          # exercises the inactive branch
python3 scripts/fake_grab.py --body 2            # pinches an object and checks it rises
python3 scripts/check_glb.py ~/.cache/vive_vr_ros2/exports/kinova_gen3/scene.glb
```

`fake_headset.py` stands in for the Unity client, speaking the same rosbridge contract over the
same socket: it advertises and publishes controller pose, buttons, 26 hand joints and gaze, and
reports what comes back. Nothing in it is hardcoded — `--host`, `--raw-ns`, `--out-ns`, `--hand`,
`--rate` match whatever the ROS side is configured with.

It is the fastest way to localise a failure: **if it still passes, the PC half is fine and the
problem is on the headset.**

`check_glb.py` validates an export structurally — chunk layout and alignment, buffer and accessor
bounds, index ranges against vertex counts, material indices, and that `POSITION` accessors carry
the mandatory min/max.

## What is established

Measured by running the system, not inferred from a successful build:

| Claim | Evidence |
|---|---|
| Export handles primitives and mesh assets | `mug_table.xml` and Menagerie `kinova_gen3/scene.xml`: 9 bodies, 86,950 triangles, 7.3 MB; validator passes |
| Body pose stream holds its rate | 59.99 Hz measured; mug at z=0.042 matching the MJCF |
| Latched scene reaches late subscribers | `/vive_vr/scene` received by a subscriber that connected after publication |
| Controller data survives the round trip | buttons `101001` exactly as sent; pose bit-exact to TF — `(0.11, 0.22, 0.33)`, quaternion `(0, 0, 0.7071068, 0.7071068)` |
| Activity detection works both ways | `true` with a moving controller, `false` when held still |
| Hand joints become a correct skeleton | 26 frames per hand at 57 Hz, every parent link verified with `view_frames` |
| Gaze survives calibration | `frame_id: world`, both eyes, pupil data intact |
| Stream survives a reset | 58.8 Hz before, 60.0 Hz after `/vive_scene/reset` |
| Grabbing lifts an object | scripted pinch caught `cube`, which rose from z=0.030 to z=0.375 and stayed |
| An exported world renders assembled | the Kinova arm, imported into Unity from the `.glb` alone |
| The client runs on the headset | `rosbridge: connected`, `scene: loaded … 12 bodies`, all `/vive_vr/raw/*` advertised |
| `ClientWebSocket` works under IL2CPP/ARM64 | it connected and carried the whole session |
| The glTF correction sign is right | the Kinova arm stands upright in the headset |
| Poses agree with what is rendered | head at `(-1.23, -0.32, 1.13)`, pitch +32°, yaw −43° — matching a capture showing the arm 57° to the left and the horizon high |
| Pointing selects, and grabbing lifts | `target → 10`, `grip → 1`, `HELD → 10`, cube from z=0.730 to 0.984 (grab was on the grip button then; it is the thumbstick click now) |
| Reset restores loose bodies | cube and ball back at their MJCF poses on the table |

The pose test uses three distinct translation components and a non-identity rotation
specifically so that a transposed axis or a dropped sign cannot pass unnoticed.

## What is not established

Be precise about this, because a green build is misleading here.

- **Eye tracking.** No permission string exists anywhere in the VIVE plugin, which is not the
  same as knowing none is needed, and no gaze data has been seen from the device.
- **Hand tracking end to end.** The subsystem runs and the topics are advertised, but no joint
  message has ever been published: `XRHand.isTracked` stays false until the runtime switches its
  interaction profile to `ext/hand_interaction_ext`, which it does only once the controllers go
  idle. That switch has been observed; a published joint has not.
- **Anything quantitative about how it feels.** Latency, jitter and comfort are unmeasured. What
  is known is that the Wi-Fi link swings between 6 ms and 183 ms round trip, which is large
  enough to matter and is not currently compensated.
- **Any recorded dataset.** Nothing has been rosbagged, and two known problems would corrupt one:
  the clock offset after a suspend, and rig height adjustments landing in published world poses.

## Bugs these tests caught

Each of these was silent — the system kept running and looked plausible.

- **The scene component truncated worlds.** Building through `mj_kdl::init_env` attaches only the
  *first root body* of each `RobotSpec`, which is right for a robot and wrong for a world file: a
  3-body test scene loaded as 2, the floor vanished and the object fell forever. It also broke the
  contract that `/vive_vr/body_poses` index *i* is manifest body *i*, so the client would have rendered
  the wrong geometry. Now loaded with `mj_loadXML` into the `Env`, which keeps `reset()` working.
- **The client deleted the sky and the lights.** `SceneLoader` reparented the nodes named in the
  manifest and destroyed the import root — taking with it everything the manifest does not list,
  which is exactly the scene dressing and the exported lights.
- **Exported worlds were a heap of parts.** Geometry is body-local with node transforms at
  identity, so until the first pose message arrived nothing was in its right place — in the
  headset as much as in a viewer. Body poses now go into the glTF nodes.

## Bugs the first on-device run caught

None of these could have been found on the PC.

- **Every material rendered magenta.** Nothing in the project references glTFast's shaders — the
  materials only exist once a `.glb` has been fetched at runtime — so the build stripped all
  three. The Editor never strips, which is exactly why the preview looked right. They are now in
  `m_AlwaysIncludedShaders`; the APK grew from 40 MB to 82 MB, which is the stripped variants
  coming back.
- **The scene never loaded.** Unity blocks cleartext HTTP on Android by default, so the `.glb`
  fetch died with "Non-secure network connections disabled in Player Settings" while rosbridge
  kept working — a live topic list and an empty world.
- **A held object span on its axis forever.** `mju_quat2Vel` reads a quaternion with negative *w*
  as a turn of more than half a circle, so the controller chased the orientation the long way
  round and flipped sign again on the way. The error quaternion is now negated when *w* < 0.
- **A grabbed ball left at 82 m.** The spring was force-limited, so the same 200 N clamp meant
  1300 m/s² on a 150 g ball. Gains are now accelerations scaled by mass and inertia.
- **Held objects trailed the hand.** The damper fought the body's absolute velocity rather than
  its velocity relative to the target, leaving a standing error of `kd·v/kp` — 10 cm per m/s.
  Fixing it needed the hand's velocity differenced *between pose messages*: a first attempt
  differenced per simulation step, which is 500 Hz against a 50 Hz stream, so it read zero on
  nine steps in ten and spiked on the tenth.
- **Reset dropped every loose body at the origin.** `SceneNode` reset to keyframe 0, but a
  robot's keyframe only covers that robot's joints — the cube and ball were zeroed rather than
  restored. `reset_keyframe` now defaults to the model's initial state.
- **Rays and published poses were 1.5 m out.** `XROrigin` applies `CameraYOffset` only in device
  space, so the camera offset object had always been at identity and converting device poses
  through the origin root happened to work. Switching tracking mode broke that assumption
  everywhere at once.

## A regression worth remembering

Adding the reset service introduced a silent stall: `reset()` rewinds sim time to zero, but the
publisher's rate gate still held the pre-reset deadline, so the stream waited for the simulation
to climb back to the last frame sent. The symptom was a stream that quietly dropped to 27 Hz, then
10 Hz — indistinguishable from a dead renderer.

`BodyPosePublisher::wants_update` now treats a backwards jump in sim time as due. The general
lesson: any rate gate holding an absolute deadline needs to handle its clock going backwards, and
a stall that produces *fewer* messages rather than *none* is easy to miss.
