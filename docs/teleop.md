# Teleoperation {#page_teleop}

> **Proposed, not built.** This page is the design to review before any code exists. [Known
> limits](limits.md#no-teleop) still lists teleop as missing, and will until this is implemented.

## What this component is responsible for

One thing: while the clutch is held, publish how far the operator's hand has moved since the
press, per arm, expressed in the tool frame.

It does not solve IK, does not know joint limits, does not know what a Kinova is. Whoever
consumes the delta owns the arm. That boundary is the point — the same stream drives a simulated
arm, a real one, or a recording, and none of them change this component.

## Why a delta, and why this removes calibration

The client reports hand poses in a frame whose relationship to the robot's world frame is
unknown; `origin_xyz` and `origin_rpy` are identity by assumption, not by measurement (see
[Known limits](limits.md#play-space-calibration)).

A delta taken against a reference captured in that same frame makes the question disappear. If
the unknown play-space transform is `R`, every pose is `R·T`, and

```
(R·T_ref)⁻¹ · (R·T_now) = T_ref⁻¹ · R⁻¹ · R · T_now = T_ref⁻¹ · T_now
```

`R` cancels exactly. Not approximately, and not only for translation: the whole unknown
transform drops out, orientation included. There is nothing left to calibrate, which is why
teleop does not wait on the calibration procedure that `limits.md` describes.

This holds only because the reference and the current pose are in the same frame. It stops
holding the moment anything absolute is published, which is the real cost of the extension in
[Later](#later).

## Frames

The two frames involved do not coincide, and assuming they do points the gripper backwards.

**OpenXR grip pose**, which is what `UnityEngine.XR.InputDevices` reports and what
`InputNode` republishes: origin at the fist centroid; `-Z` along the grip toward the thumb; `±X`
perpendicular to the palm, `+X` out the back of the right hand; `+Y` implied right-handed,
roughly toward the forearm.

**A robot tool frame**: conventionally `+Z` along the approach axis, out of the flange.

So a fixed rotation `R_tc` maps controller axes onto tool axes. Because the published quantity is
a transform rather than a point, it enters as a conjugation, not an offset:

```
Δ_tool = R_tc⁻¹ · Δ_controller · R_tc
```

`R_tc` is a constant of the pairing — this controller, that gripper — and belongs in
`vr.yaml`, not in a per-session procedure. It ships as identity: the correct value is found by
watching which way the gripper actually goes in simulation, and writing down what was observed
rather than what the conventions imply.

## The clutch

| Event | Behaviour |
|---|---|
| Button pressed | capture `T_ref` for that hand; publish `clutch = true`; delta is identity |
| Held | publish `T_ref⁻¹ · T_now`, rotated into the tool frame, at the incoming pose rate |
| Released | publish `clutch = false` and stop publishing deltas |
| Pressed again | a **new** `T_ref`; the arm does not jump back |

Re-pressing is how the operator recovers reach, exactly as lifting a mouse does. Nothing
accumulates across clutches: each press starts from wherever the arm already is, which is the
consumer's business, not this component's.

## Interfaces

Published, per arm:

| Topic | Type | Meaning |
|---|---|---|
| `<out>/teleop/<arm>/delta` | `geometry_msgs/TransformStamped` | motion since the press, in tool |
| `<out>/teleop/<arm>/clutch` | `std_msgs/Bool` | on change, not per tick |

`TransformStamped` rather than `PoseStamped` deliberately: a pose names a place in a frame, and
this is a relative motion with no place attached. `frame_id` is the arm's tool frame as it was at
the press, `child_frame_id` the commanded tool frame.

Subscribed, per arm:

| Topic | Type | Used for |
|---|---|---|
| `<arm>/ee_pose` | `geometry_msgs/PoseStamped` | nothing yet; see [Later](#later) |

The EE pose is taken in from the start so the topic contract does not change when the extensions
below arrive. The first version reads it and logs staleness, nothing more.

## Configuration

```yaml
teleop:
  left:
    hand: left
    delta_topic: /vr/teleop/left/delta
    ee_pose_topic: /left_arm/ee_pose
    clutch_button: 1
    tool_from_controller_rpy: [0.0, 0.0, 0.0]
  right:
    hand: right
    ...
```

Dual-arm is two blocks. One arm is one block. Nothing in the component counts arms.

`clutch_button` indexes `Joy` buttons the way `grab_button` already does, so the two share a
convention rather than inventing a second one. `1` is squeeze.

## What happens when things fail

Silence must never be mistaken for "keep going".

- **Controller tracking drops** — the hand is already reported inactive after `stale_after_s`.
  The clutch force-opens, `clutch = false` goes out, and deltas stop. The last delta is never
  republished to fill the gap.
- **rosbridge drops** — the stream simply ends. The consumer must treat absence of deltas as
  hold, not as continue; that requirement belongs in the consumer and is stated here because
  this component cannot enforce it.
- **App suspended** — indistinguishable from a tracking drop, and handled as one. Note the clock
  offset is separately poisoned by a suspend; see
  [Known limits](limits.md#clock-offset-is-one-way-and-a-suspend-poisons-it).

No smoothing, no rate limiting, no scaling in the first version. The deltas are raw at the
incoming pose rate, 60–90 Hz, and the consumer is where a jerky arm gets fixed — it is the only
place that knows the arm's limits.

## Where it lives

A third component, `vr::TeleopNode`, loaded into the same container as `vr::SceneNode` and
`vr::InputNode`, following the
[one package, composable nodes](design.md#one-package-composable-nodes) decision.

It runs entirely on the PC and consumes what `InputNode` already publishes — calibrated poses and
`Joy`. **The client does not change and the APK does not need rebuilding**, which also means the
headset cannot be the reason teleop misbehaves.

## Later {#later}

Both extensions need the EE pose that is already subscribed, and neither is in the first version:

- **Absolute targets.** Anchor at the clutch and publish `EE_ref ∘ Δ` as a pose to follow. Easier
  for a consumer, but it reintroduces exactly the frame question the delta removed, so it needs
  the calibration from `limits.md` to be honest.
- **Drift detection.** Compare commanded against actual and force a re-clutch when the arm has
  fallen far enough behind that the operator's hand no longer corresponds to the tool.

## How it will be verified

Not by a passing build.

- `tools/fake_headset.py` extended to press, hold, move and release: the delta must be identity
  at the press, `dz` must match a commanded 10 cm move to the millimetre, and a 90° wrist
  rotation must appear as the right quaternion in the tool frame.
- A dropout test: stop publishing poses mid-clutch and confirm `clutch = false` is published and
  deltas cease, rather than the last value repeating.
- In simulation, drive a body with the delta stream and confirm the direction of motion matches
  the hand, which is also how `tool_from_controller_rpy` gets its real value.
