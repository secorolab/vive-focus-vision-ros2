# Testing {#page_testing}

## Tools

```bash
python3 tools/fake_headset.py --seconds 10     # with the stack running
python3 tools/fake_headset.py --still          # exercises the inactive branch
python3 tools/check_glb.py /tmp/vr_robot/scene.glb
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
| Latched scene reaches late subscribers | `/vr/scene` received by a subscriber that connected after publication |
| Controller data survives the round trip | buttons `101001` exactly as sent; pose bit-exact to TF — `(0.11, 0.22, 0.33)`, quaternion `(0, 0, 0.7071068, 0.7071068)` |
| Activity detection works both ways | `true` with a moving controller, `false` when held still |
| Hand joints become a correct skeleton | 26 frames per hand at 57 Hz, every parent link verified with `view_frames` |
| Gaze survives calibration | `frame_id: world`, both eyes, pupil data intact |
| Stream survives a reset | 58.8 Hz before, 60.0 Hz after `/vr_scene/reset` |

The pose test uses three distinct translation components and a non-identity rotation
specifically so that a transposed axis or a dropped sign cannot pass unnoticed.

## What is not established

Be precise about this, because a green build is misleading here.

- **The handedness conversion.** `fake_headset.py` sends ROS-convention poses directly, so it
  validates the *transport*, not `FrameConv`. The Unity client runs poses through that conversion
  first, and nothing has exercised it.
- **`ClientWebSocket` under IL2CPP on ARM64.** It compiles; that is all that is known.
- **The glTF correction rotation.** Derived to be ±90° about Y, defaulted to +90°, never observed.
- **Eye tracking permissions.** No permission string exists anywhere in the VIVE plugin, which is
  not the same as knowing none is needed.
- **Anything about how it feels.** Latency, jitter, comfort, whether 60 Hz body poses look smooth
  in a headset — all unmeasured.

## A regression worth remembering

Adding the reset service introduced a silent stall: `reset()` rewinds sim time to zero, but the
publisher's rate gate still held the pre-reset deadline, so the stream waited for the simulation
to climb back to the last frame sent. The symptom was a stream that quietly dropped to 27 Hz, then
10 Hz — indistinguishable from a dead renderer.

`BodyPosePublisher::wants_update` now treats a backwards jump in sim time as due. The general
lesson: any rate gate holding an absolute deadline needs to handle its clock going backwards, and
a stall that produces *fewer* messages rather than *none* is easy to miss.
