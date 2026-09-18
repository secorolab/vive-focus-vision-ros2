# Teleoperation {#page_teleop}

> **Built as `vive_vr_ros2::TeleopNode`.** This component publishes controller deltas.
> The separate [OpenArm simulation](openarm_sim.md) consumes them with right-arm IK;
> the generic scene viewer does not. See [Known limits](limits.md#no-teleop).

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
`vive_vr.yaml`, not in a per-session procedure. It ships as identity: the correct value is found by
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
| `<out>/teleop/<arm>/gripper` | `std_msgs/Float32` | 0 open to 1 closed, while clutched |

`TransformStamped` rather than `PoseStamped` deliberately: a pose names a place in a frame, and
this is a relative motion with no place attached. `frame_id` is the arm's tool frame as it was at
the press, `child_frame_id` the commanded tool frame.

The gripper is the index finger's **analog** pull, `axes[2]`, not the trigger button: a partial
grip survives the trip, and a consumer that only wants two states can threshold it. Setting
`gripper_axis` to `-1` falls back to the button. It is published only while the clutch is closed,
for the same reason the deltas are — a disengaged operator must not be closing a real hand. On
release the last value is simply not followed by another, so a gripper holding something keeps
holding it rather than springing open.

Subscribed, per arm:

| Topic | Type | Used for |
|---|---|---|
| `<arm>/ee_pose` | `geometry_msgs/PoseStamped` | nothing yet; see [Later](#later) |

The EE pose is taken in from the start so the topic contract does not change when the extensions
below arrive. The first version subscribes and discards; leaving `ee_pose_topic` empty skips the
subscription entirely.

## Configuration

```yaml
vr_teleop:
  ros__parameters:
    teleop.pose_timeout_s: 0.25
    teleop:
      right:
        hand: right
        clutch_button: 1
        gripper_axis: 2
        ee_pose_topic: ""
        tool_from_controller_rpy: [0.0, 0.0, 0.0]
      left:
        hand: left
        ...
```

Dual-arm is two blocks. One arm is one block. Nothing in the component counts arms — the arm
names are discovered from whichever `teleop.<arm>.*` parameters were passed, so there is no list
to keep in step with the blocks.

`delta_topic` and `clutch_topic` may be set per arm and default to
`<out_ns>/teleop/<arm>/{delta,clutch}`, so the block above leaves them out.

`clutch_button` indexes `Joy` buttons the way `grab_button` already does, so the two share a
convention rather than inventing a second one.

It defaults to `1`, the **grip button**: the hand holds the pose the way it holds the controller,
and the index finger works the gripper on the **trigger**. The simulated grabber moved to the
thumbstick click to make room, since one press must not both grab a body and engage an arm. The
full allocation is in [Interfaces](interfaces.md#controller-buttons).

## What happens when things fail

Silence must never be mistaken for "keep going".

- **Controller tracking drops** — measured as poses no longer arriving, after
  `teleop.pose_timeout_s`. The clutch force-opens, `clutch = false` goes out, and deltas stop.
  The last delta is never republished to fill the gap.

  This is deliberately *not* `<out>/<hand>/active`, which an earlier draft of this page proposed.
  That flag means "has not moved `motion_eps_m` within `stale_after_s`", which is exactly what an
  operator holding a position looks like — it would drop the clutch mid-task for standing still.
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

A third component, `vive_vr_ros2::TeleopNode`, loaded into the same container as `vive_vr_ros2::SceneNode` and
`vive_vr_ros2::InputNode`, following the
[one package, composable nodes](design.md#one-package-composable-nodes) decision.

It runs entirely on the PC and consumes what `InputNode` already publishes — calibrated poses and
`Joy` — so no teleop logic lives on the headset and the client cannot be the reason teleop
misbehaves.

Freeing the grip button did cost one client change, contrary to what this page first claimed:
`VrPointer` tinted a body green the moment the grip button went down, and that optimistic
highlight had to follow the grabber onto the thumbstick click. An APK built before 2026-09-17
will highlight on the wrong control.

## Later {#later}

Both extensions need the EE pose that is already subscribed, and neither is in the first version:

- **Absolute targets.** Anchor at the clutch and publish `EE_ref ∘ Δ` as a pose to follow. Easier
  for a consumer, but it reintroduces exactly the frame question the delta removed, so it needs
  the calibration from `limits.md` to be honest.
- **Drift detection.** Compare commanded against actual and force a re-clutch when the arm has
  fallen far enough behind that the operator's hand no longer corresponds to the tool.

## How it is verified

`scripts/teleop_check.py` drives `<raw>/right/{pose,joy}` the way the headset does — poses
streaming on a timer, because a clutch held still must stay closed — and asserts what comes out
the other end. With the stack and `teleop_node` running:

```bash
ros2 launch vive_vr_ros2 tracking.launch.py &
ros2 run vive_vr_ros2 teleop_node --ros-args --params-file install/vive_vr/share/vive_vr_ros2/config/vive_vr.yaml &
python3 scripts/teleop_check.py
```

Eleven checks, all passing as of 2026-09-17: the clutch closes on press and opens on release; the
delta is exactly identity at the press; a 10 cm move gives `dz = 0.100000`; a 90° yaw gives
`qz = 0.707107`; the gripper follows the trigger axis; neither deltas nor gripper commands are
published while the clutch is open; and when the poses stop, the clutch opens and the last delta
is *not* repeated.

Still unverified: anything with an arm on the end of it. `tool_from_controller_rpy` stays
identity until a gripper has been watched moving, because the observed value is the only honest
one.
