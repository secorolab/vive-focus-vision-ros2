# Design notes {#page_design}

Why the system is shaped this way, and what was considered and rejected. Recorded so the
decisions do not get re-litigated from scratch.

## Physics on the PC, rendering on the headset

MuJoCo has no official Android build. Community arm64 binaries exist and the old `qsort_r`
blocker is fixed upstream, but `mj_kdl_wrapper` pins MuJoCo 3.9.0 alongside a KDL fork, GLFW, EGL,
ffmpeg and Eigen — none of which is going on a headset.

That constraint points the same way the use case does: one sim clock on one machine is what makes
a recorded demonstration line up. The headset renders and reports what the user does; it never
owns state.

## Standalone APK, not PC VR

The Focus Vision's PC-VR paths — VIVE Hub streaming and the DisplayPort Wired Streaming Kit — are
Windows-only. On Linux the options are ALVR (no wired DisplayPort, breaks against SteamVR ≥
2.16.7) or WiVRn (does not list the Focus Vision at all; Focus 3 and XR Elite are listed as
"laggy"). A hybrid Intel/NVIDIA laptop adds PRIME offload problems on top.

A standalone Android app sidesteps all of it: the headset runs its own OpenXR runtime, and the PC
only has to speak ROS.

## Transport: rosbridge alone

Evaluated against what people actually ship, not from first principles.

[XRoboToolkit](https://arxiv.org/abs/2508.00097), PICO's open-source XR teleoperation framework
and the closest prior art, uses a custom TCP protocol with fixed ~56-byte pose packets at 90 Hz
and a thin ROS node publishing `xr_msgs/Controller`. NVIDIA's IsaacTeleop is CloudXR-based.
Neither uses a generic ROS bridge for the hot path.

An initial design followed that lead: binary TCP for the 90 Hz input stream, rosbridge for
everything else, HTTP for the scene. It was dropped in favour of one protocol, because two wire
formats and two codecs is a real maintenance cost for a latency budget already dominated by a
5–10 ms Wi-Fi leg.

What remains is rosbridge, which is the only option **released and maintained for Jazzy** (2.7.1,
August 2026). Being `rclpy`-based it is RMW-agnostic, so `rmw_zenoh` changes nothing about it.
ROS-TCP-Connector was the alternative; its upstream support badges stop at Galactic.

## Why not adopt XRoboToolkit

It was evaluated seriously, by measuring the port rather than guessing:

- Its Quest fork rewrote `TrackingData.cs` with **693 changed lines against a 512-line file** — a
  rewrite, not a parameterisation.
- **39 files differ** between the two existing forks, 23 exist only in one and 14 only in the
  other. There is no abstraction seam; adopting it means becoming fork #3 and owning that merge
  burden permanently.
- It carries a **995 MB Qt6/gRPC PC service** with a vendored closed binary, and a Foxy/Humble ROS
  side that would need porting to Jazzy.
- The bulk of its value — roughly 3300 lines of network code plus a video pipeline — exists to
  stream **camera video from a real robot**. This project renders MuJoCo worlds locally.

Two things were taken from it: the `Joy` layout shape, and the confirmation that
`UnityEngine.XR.InputDevices` + `CommonUsages` is sufficient for controller pose and buttons —
about 40 lines, not a framework.

This would be worth revisiting if streaming stereo video from a real robot became a requirement;
that pipeline is genuinely weeks of work to rebuild.

## One package, composable nodes

The first working version was a single node that ran the simulation, published the world *and*
handled controller input — three unrelated jobs. The split follows what varies independently: the
headset can change, the robot can change, the world source can change.

`vive_vr_ros2::InputNode` holds everything headset-specific, so swapping the Unity client for another
OpenXR client changes one component and nothing downstream. `vive_vr_ros2::SceneNode` holds the world.
Teleop will be a third, and deliberately will not know whether its target is sim or real.

They are components rather than separate packages because that keeps one build and one config
file, and loading them into one container means their topics cross intra-process — which also
answers the objection that splitting adds serialization hops.

## `BodyPosePublisher` as a class

The valuable seam is not the node, it is the thing inside it. Shaped exactly like `mj_kdl`'s
`CameraRosPublisher` — no node, no executor, no thread, no `mjData` ownership — an existing
application adds VR output in two lines instead of restructuring itself around someone else's
node. See [Embedding](embedding.md).

It links MuJoCo but not KDL, deliberately, so that embedding it costs nothing extra.

## Using `mj_kdl_wrapper`

`SceneNode` builds worlds through `mj_kdl::init_env` rather than `mj_loadXML`. That buys
composable scenes (robots, attachments, floor, skybox, objects) and, more importantly for
demonstration recording, a `reset()` that restores a keyframe **and** re-synchronises `Robot`
command ports — which `mj_resetData` alone does not do. Teleop will need the same library's IK
and ACHD solvers.

## The exporter carries simulation, not scenery

The `.glb` holds bodies, their geometry, their loaded poses and MuJoCo's lights. It does not hold
the ground or the sky, which the client draws.

That split came from trying the other way. An unbounded ground plane has no honest mesh, so the
exporter baked a 20 m patch of it and tiled it into checker squares; at a grazing angle those
squares aliased into moiré stripes, because MuJoCo can afford a fine checker only by rendering it
from a mipmapped texture. The sky had the same shape of problem: a skybox is a renderer texture,
and the dome standing in for it was visibly banded and left a seam at the horizon. Both are one
textured quad and a skybox on the client, drawn better and for nothing.

What stays in the file is what a viewer cannot invent: which bodies exist, what they look like,
where they start, and how the scene is lit.

## Grabbing through MuJoCo's own forces

A held body is pulled by a spring-damper written into `xfrc_applied`, never teleported. Contact,
mass and actuators still decide the outcome, so an object too heavy to lift stays put and pushing
a robot link fights its actuators. Teleporting would be easier and would produce demonstration
data that no policy should learn from.

The same mechanism covers both things asked of it: a free body is picked up, and a body in an
articulated chain is pushed.

## Hands as TF, gaze as a custom message

`sensor_msgs/JointState` carries scalar joint positions; XR Hands reports 6-DoF poses per joint.
Converting to angles needs a hand kinematic model with defined degrees of freedom, which this
project does not have, and the result would be lossy and model-dependent. TF represents a tree of
poses natively.

Gaze is the one custom message because nothing gaze-, eye-, hand- or skeleton-shaped exists in
any installed msgs package. It is per-eye with independent validity flags, because the tracker can
lose one eye or drop the pupil measurement while holding the pose — and averaging to a single ray
would discard exactly the case recorded data needs to explain.

## Vendor-neutral input

Controllers through `CommonUsages`, hands through `XRHandSubsystem`, never VIVE's own gesture API.
The VIVE plugin maps its interaction profile onto those usages and implements the hand subsystem
provider, so the same code runs on another OpenXR headset. Only gaze is vendor-specific, because
Unity has no cross-vendor eye tracking abstraction.

## Configuration out of code

Topic names, frames, rates and the play-space calibration live in `config/vive_vr.yaml` on the ROS
side and in a device-side JSON file on the client. The client file matters most: without it,
changing the PC's IP address means rebuilding and reinstalling an APK.
