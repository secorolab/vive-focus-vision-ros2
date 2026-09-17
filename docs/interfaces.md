# Interfaces {#page_interfaces}

Everything here is configurable; the names below are the defaults from
[`config/vive_vr.yaml`](configuration.md).

The headset publishes into `raw_ns` (`/vive_vr/raw`). `vive_vr_ros2::InputNode` calibrates and republishes into
`out_ns` (`/vive_vr`). The two must differ, or the node would subscribe to its own output — it refuses
to start if they match.

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `<raw>/{head,left,right}/pose` → `<out>/…` | `geometry_msgs/PoseStamped` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joy` → `<out>/…` | `sensor_msgs/Joy` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joints` → `<out>/…` and TF | `geometry_msgs/PoseArray` (26) | headset → PC | 60 Hz |
| `<raw>/gaze` → `<out>/gaze` | `vr/EyeGaze` | headset → PC | 60 Hz |
| `<raw>/{left,right}/target` → `<out>/…` | `std_msgs/Int32` | headset → PC | on change |
| `<out>/{left,right}/held` | `std_msgs/Int32`, transient local | PC → headset | on change |
| `<out>/{left,right}/active` | `std_msgs/Bool`, latched | PC | on change |
| `<out>/scene` | `std_msgs/String`, transient local | PC → headset | once |
| `<out>/body_poses` | `geometry_msgs/PoseArray` | PC → headset | 60 Hz |
| `<out>/pc_time` | `builtin_interfaces/Time` | PC → headset | 10 Hz |
| `<out>/teleop/<arm>/delta` | `geometry_msgs/TransformStamped` | PC | while clutched |
| `<out>/teleop/<arm>/clutch` | `std_msgs/Bool`, latched | PC | on change |
| `<out>/teleop/<arm>/gripper` | `std_msgs/Float32`, 0–1 | PC | while clutched |

Services: `/vive_scene/reset` (`std_srvs/Trigger`).

## Controller buttons

`sensor_msgs/Joy` carries no schema, so the layout is a contract. The names are the ones printed
in the Focus Vision manual, so a binding can be discussed without translating between
vocabularies:

```
buttons = [trigger, grip, A|X, B|Y, thumbstick_click, menu]
axes    = [thumbstick_x, thumbstick_y, trigger, grip]
```

Index 2 is **A** on the right controller and **X** on the left; index 3 is **B** and **Y**.
`menu` exists on the left controller only and reads 0 on the right. The **VIVE button is
reserved by the runtime** and never reaches the application, so it cannot be used for anything.

Who owns what, so that nothing is bound twice:

| Control | Index | Owner |
|---|---|---|
| Trigger | `axes[2]`, `buttons[0]` | teleop gripper, 0 open to 1 closed |
| Grip button | `buttons[1]`, `axes[3]` | teleop clutch, `clutch_button` |
| Thumbstick click | `buttons[4]` | the simulated grabber, `grab_button` |
| A, B (right) | `buttons[2,3]` | locomotion up and down |
| X, Y (left) | `buttons[2,3]` | recentre, reset the simulation |
| Thumbstick | `axes[0,1]` | walk and turn (left controller) |
| Menu (left) | `buttons[5]` | switch controllers ↔ hands, held 1 s |

The two hand-held actions sit on the two fingers that do them: the index finger works the
gripper, and the whole hand grips to hold the pose. The simulated grabber moved off the grip
button to make room, and in hand mode it is still a pinch.

The pose published is the **grip** pose — the hand/handle pose, the one to retarget to an
end-effector. The aim pose (the pointing ray, for UI) is not published yet.

## Controller activity

`<out>/<hand>/active` exists because VIVE controllers keep reporting a valid, tracked pose while
lying on a table. A controller that has not moved `motion_eps_m` within `stale_after_s` is
reported inactive.

It answers "is anyone holding this", not "is this tracked". The topic is latched, so a subscriber
gets the current state on connect rather than waiting for the next change.

**Do not gate teleop on it.** An earlier version of this page advised exactly that, and it is
wrong: an operator holding a position looks identical to a controller on a table, so the clutch
would drop mid-task for standing still. `vive_vr_ros2::TeleopNode` treats the *absence of pose messages* as
the dropout instead — see [Teleoperation](teleop.md).

## Hand joints

The client publishes 26 joint poses per hand in `XRHandJointID` order; `vive_vr_ros2::InputNode` turns each
array into TF frames, reparenting them into a real skeleton:

```
world → vr_<hand>_wrist → vr_<hand>_palm
                        → vr_<hand>_thumb_metacarpal → _proximal → _distal → _tip
                        → vr_<hand>_index_metacarpal → _proximal → _intermediate → _distal → _tip
                        → … middle, ring, little
```

TF stores each frame relative to its parent, so `InputNode` computes `parent⁻¹ · child` for every
joint; only the wrist is placed in the world frame.

**Why TF and not `sensor_msgs/JointState`:** `JointState` carries scalar joint *positions*, while
XR Hands reports 6-DoF *poses* per joint. Converting to angles would need a hand kinematic model
with defined degrees of freedom, which this project does not have, and the conversion would be
lossy and model-dependent. TF represents a tree of poses natively, which is exactly what this is.

The cost is real: at 60 Hz, two hands are 3120 transforms a second that every `tf2` listener in
the system pays for. `publish_hand_tf: false` turns it off.

The joint order is a contract shared between `HandPublisher.cs` and the `kHandJoints` table in
`input_node.cpp`. Neither may be reordered independently.

The calibrated joints are also republished as a `PoseArray` on `<out>/<hand>/joints`, because a
consumer acting on a whole hand at once — grabbing, for one — wants a single timestamped snapshot
rather than 26 TF lookups.

## Grabbing

`vive_vr_ros2::Grabber`, owned by the scene component, lets the user pick up and push simulated bodies. It
consumes topics that already exist rather than adding any: the grip pose, the `Joy` buttons and
the hand joints.

A grab starts on either **a pinch** (thumb tip to index tip closer than `pinch_close_m`, released
past `pinch_open_m` so it cannot chatter) or **the thumbstick click**.

**What is caught is what the client's ray is on**, published as a manifest index on
`<out>/<hand>/target`. `reach_m` is only the fallback for a hand with no pointer. The body keeps
the offset it had when caught rather than snapping to the palm, so it is dragged from where it
sat and moving the hand up moves it up. The grabber answers on `<out>/<hand>/held` with what it
is actually holding, which is not always what was pointed at: a body with no mass is refused.

Held bodies are pulled by a clamped spring-damper written into `xfrc_applied` — not teleported.
Mass, contact and actuators still decide what happens, so a heavy object resists and pushing a
robot link fights its actuators. That is also what makes the resulting motion worth recording.

Two details that are not obvious and were both found by getting them wrong:

- The gains are **accelerations scaled by each body's mass and inertia**, not forces. A fixed
  force clamp means wildly different accelerations across a scene, and a reach that merely nudges
  a robot arm will fling a loose object out of the world.
- The damper works on the body's velocity **relative to the target**, and that target velocity is
  differenced between incoming pose messages rather than between simulation steps. Damping
  absolute velocity leaves a standing error of `kd·v/kp` while the hand moves; differencing per
  step reads zero on most steps, because `apply()` runs at 500 Hz against a 50 Hz stream.

## Eye gaze

`vr/EyeGaze` is the only custom message in the project. Nothing gaze-, eye-, hand- or
skeleton-shaped exists in any installed msgs package — not `sensor_msgs`, `geometry_msgs` or
`vision_msgs` — which is the whole justification for defining one.

```
std_msgs/Header header
geometry_msgs/Pose left
geometry_msgs/Pose right
bool  left_valid
bool  right_valid
float32 left_pupil_diameter_mm
float32 right_pupil_diameter_mm
bool  left_pupil_valid
bool  right_pupil_valid
```

Each pose is a **ray, not a point**: position is the eye, orientation points along the gaze.

It is per-eye rather than one averaged ray on purpose. The tracker can lose one eye, or hold the
pose while dropping the pupil measurement, and each quantity therefore carries its own validity.
Averaging here would throw away exactly the case that recorded data needs to be able to explain.

## Body poses

`<out>/body_poses` index *i* is MuJoCo body *i*, matching `bodies[]` in the manifest. The client
indexes straight into it, so nothing may reorder. A length mismatch means the running model is
not the one that was exported, and the client says so rather than rendering nonsense.

Flat arrays rather than named poses keep it small: 100 bodies at 60 Hz is roughly 500 kB/s as
`PoseArray`. For scenes with hundreds of bodies, move this topic to rosbridge's CBOR compression
before blaming the renderer.

## Coordinate conventions

This is where most of the subtle bugs in a system like this live, so all of it is in one place.

- **MuJoCo and ROS agree.** Both are right-handed, Z-up, X-forward (REP-103). Body poses need no
  conversion and are published exactly as MuJoCo reports them.
- **Unity is left-handed, Y-up.** `FrameConv` is the single place this is handled, applied once
  at the publish/apply boundary. The map is `unity = (-ros.y, ros.z, ros.x)`; because it flips
  handedness, the quaternion's scalar part flips sign with it — a rotation of +θ about **n**
  becomes −θ about the mapped **n**.
- **glTF is right-handed Y-up.** `scene_export` rotates −90° about X on the way out
  (`(x,y,z) → (x,z,−y)`), so the file is correct in any standard glTF viewer.
- **One residual rotation.** glTFast applies its own handedness flip on import. That flip composed
  with the export rotation differs from the live-pose conversion by exactly one rotation about Y,
  which `SceneLoader.gltfCorrectionEuler` carries. It can only be ±90°, so if the world loads
  rotated 180°, negate it. The derivation: with `U` the ROS→Unity map, `R` the export rotation and
  `G` glTFast's flip, the correction is `C = U·R⁻¹·G⁻¹`, whose determinant is +1 — a pure
  rotation — regardless of which axis glTFast chooses to negate.
- **Frames.** Everything is simulated, so the client is told where it stands: `spawnPosition` in
  the device config places the rig in the world, and every published pose is transformed by the
  rig before it is sent. Walk five metres and the published hand moves five metres, which is what
  makes reaching for an object work at all. `InputNode`'s `vr_origin → world` calibration is
  therefore identity, and only becomes meaningful if a real robot has to share the frame — see
  [Known limits](limits.md).

## Timestamps

The client stamps its messages with its own clock plus an offset estimated from `<out>/pc_time`,
so recorded controller poses and sim state share a timeline. `InputNode` preserves the client's
stamp and only falls back to its own clock if the client sent none.

The estimate is one-way, so it is biased late by roughly half the Wi-Fi round trip — single-digit
milliseconds. [Known limits](limits.md) describes the round-trip upgrade.
