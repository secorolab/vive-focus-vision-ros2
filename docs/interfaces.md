# Interfaces {#page_interfaces}

Everything here is configurable; the names below are the defaults from
[`config/vr.yaml`](configuration.md).

The headset publishes into `raw_ns` (`/vr/raw`). `vr::InputNode` calibrates and republishes into
`out_ns` (`/vr`). The two must differ, or the node would subscribe to its own output — it refuses
to start if they match.

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `<raw>/{head,left,right}/pose` → `<out>/…` | `geometry_msgs/PoseStamped` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joy` → `<out>/…` | `sensor_msgs/Joy` | headset → PC | 90 Hz |
| `<raw>/{left,right}/joints` → TF | `geometry_msgs/PoseArray` (26) | headset → PC | 60 Hz |
| `<raw>/gaze` → `<out>/gaze` | `vr/EyeGaze` | headset → PC | 60 Hz |
| `<out>/{left,right}/active` | `std_msgs/Bool`, latched | PC | on change |
| `<out>/scene` | `std_msgs/String`, transient local | PC → headset | once |
| `<out>/body_poses` | `geometry_msgs/PoseArray` | PC → headset | 60 Hz |
| `<out>/pc_time` | `builtin_interfaces/Time` | PC → headset | 10 Hz |

Services: `/vr_scene/reset` (`std_srvs/Trigger`).

## Controller buttons

`sensor_msgs/Joy` carries no schema, so the layout is a contract:

```
buttons = [trigger, squeeze, primary(X|A), secondary(Y|B), stick_click, menu]
axes    = [stick_x, stick_y, trigger, squeeze]
```

`menu` exists on the left controller only and reads 0 on the right. The **system button is
reserved by the runtime** and never reaches the application, so it cannot be used for anything.

The pose published is the **grip** pose — the hand/handle pose, the one to retarget to an
end-effector. The aim pose (the pointing ray, for UI) is not published yet.

## Controller activity

`<out>/<hand>/active` exists because VIVE controllers keep reporting a valid, tracked pose while
lying on a table. A controller that has not moved `motion_eps_m` within `stale_after_s` is
reported inactive.

A consumer that acts on poses — teleop above all — should gate on this rather than on the
runtime's own tracked flag, which stays true. The topic is latched, so a subscriber gets the
current state on connect rather than waiting for the next change.

## Hand joints

The client publishes 26 joint poses per hand in `XRHandJointID` order; `vr::InputNode` turns each
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
- **Frames.** The headset publishes in `vr_origin`, its own play space, whose origin is wherever
  the guardian was drawn. `InputNode` applies the `vr_origin → world` calibration and republishes
  in `world`. That calibration is identity until something measures it — see
  [Known limits](limits.md).

## Timestamps

The client stamps its messages with its own clock plus an offset estimated from `<out>/pc_time`,
so recorded controller poses and sim state share a timeline. `InputNode` preserves the client's
stamp and only falls back to its own clock if the client sent none.

The estimate is one-way, so it is biased late by roughly half the Wi-Fi round trip — single-digit
milliseconds. [Known limits](limits.md) describes the round-trip upgrade.
