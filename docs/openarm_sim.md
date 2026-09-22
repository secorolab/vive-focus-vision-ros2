# OpenArm V1 motion-test setup {#page_openarm_sim}

`scripts/openarm_sim.py` replaces the manual Python blocks used to prepare the dual-arm
OpenArm V1 model. It generates files, exports the headset geometry, and provides a short
launch command. It does not install dependencies, build an APK, or connect to physical motors.

This is a **motion-test simulation**: gravity and contacts are disabled. The imported collision
shapes overlap at the starting pose, so contact physics must be corrected before grasping tests.
The dual-arm IK bridge is opt-in with `launch --teleop`. It runs inside the separate
`openarm_sim_node` application and writes only to its simulated motors. Each controller drives its matching arm independently. `teleop_node` by itself still
only produces controller deltas; it needs this consumer to move a robot.

## Scope and existing IK

The upstream SceneNode, generic simulation launch, TeleopNode and VR core are unchanged.
`openarm_sim_node` owns the robot simulation and embeds `BodyPosePublisher`, following
[Embedding](embedding.md). CMake builds the extra application separately with
`BUILD_OPENARM_SIM`; it can be disabled without changing the core VR package.

Orocos KDL is already a dependency through `mj_kdl_wrapper`, and the OpenArm V1 MoveIt
configuration also selects KDL. The adapter takes the chain and the Jacobian from KDL rather
than reimplementing them. This controls the gripper pose, not a unique human-like elbow posture:
a 7-joint arm can reach one gripper pose in multiple configurations.

Other upstream options exist: [OpenArm MuJoCo](https://github.com/enactic/openarm_mujoco)
contains native V1 assets, and [OpenArm Control](https://github.com/enactic/openarm_control)
provides reusable MuJoCo/Mink kinematics. Neither is installed by this helper. The latter's
model/frame integration has not been validated against this converted V1 model. Switching
models would also require verifying TCP placement, joint names, actuators and collision geometry.

## Run it

After changing C++ code or launch files, build once:

```bash
source /opt/ros/jazzy/setup.zsh
cd ~/vive_vr_ws
colcon build --packages-select vive_vr_ros2 --cmake-args -DBUILD_SCENES=OFF -DBUILD_OPENARM_SIM=ON
```

Requires the already-built ROS Jazzy, OpenArm description and VIVE workspaces, Python PyYAML
(available with the ROS setup), and the wrapper's cached MuJoCo 3.9.0 library. No extra Python
MuJoCo installation is needed. In a Zsh terminal:

```bash
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
export ROS_DOMAIN_ID=42
cd ~/Desktop/vive-focus-vision-ros2

python3 scripts/openarm_sim.py prepare
```

`prepare` regenerates the files listed below in `~/vive_vr_ws/models/openarm_v1` and exports
`vr_scene_controlled`. It overwrites these generated files on repeat runs. Keep manual model
experiments in a separate directory. The OpenArm source repository is not modified.

Stop any previous tracking/simulation launch with Ctrl+C, then:

```bash
python3 scripts/openarm_sim.py launch
```

For controller motion on both arms, stop the preview and any separately running `teleop_node`, then:

```bash
python3 scripts/openarm_sim.py launch --teleop
```

This starts tracking, the simulation and exactly one `teleop_node`. Re-run `prepare` after
updating this script to generate `teleop.yaml` and the straight-down home-pose keyframe.

1. Release both side grips before starting. Calibrate each hand separately, held down beside you,
   matching the hanging robot arm. Perform the alignment below before the first motion test.
2. Match your controller orientation to the robot gripper using your calibrated grip.
3. Hold either side grip to move its matching arm. Translation follows your hand at 1:1 scale in the
   generated teleop configuration; wrist rotation changes the gripper orientation.
   Begin with 5–10 cm movements before raising the arm farther. If it does not engage,
   release grip, match orientation and retry. Both grips can be held at the same time.
4. Release the side grip to hold the arm. Re-grip to continue from its current pose.
5. While engaged, squeeze that controller's index trigger to close its gripper;
   release that trigger to open it. Engaging the side grip alone preserves the gripper position.
   The first trigger sample after re-gripping is captured without applying it; subsequent changes
   control the opening. This prevents an idle trigger from opening the gripper on clutch press.

To inspect the installed headset app's input stream without commanding anything:

```bash
python3 scripts/openarm_sim.py inputs
```

Press the rear trigger alone: `rear trigger` should change. Press the side grip alone:
`side grip` should change. The source contract is axes[2] for trigger and buttons[1] for clutch.
If the observed layout differs, inspect the installed APK before changing bindings blindly.

The robot starts in the straight-down arm pose shown by the original RViz description, with
partly open fingers. At this pose the tool's local +Z points down toward the fingertips; world
+Z points up. These are different frames. The solver can retry from a bent *scratch* pose to
escape the straight-arm singularity without changing the live starting pose. Targets beyond
the arm's reach (for example farther down from full extension) still cannot be followed.
`prepare --home bent` retains the earlier bent starting pose as an optional experiment.

### Alignment without returning to the keyboard

For a new alignment, stop any running simulation first, then use one terminal:

```bash
python3 scripts/openarm_sim.py launch --teleop --align --delay 10
```

Put on the headset during the 10-second delay, keep the side grip released, and hold the
controller down beside you in the orientation that represents the hanging gripper. The
calibration phase blocks robot engagement, captures one second of steady poses, then restarts
the stack automatically with the saved rotation. The headset briefly disconnects and reconnects.
After reconnection, release the grip and press it again to engage. A failed capture stops the
stack without starting teleop. Ctrl+C stops the owned calibration launch too.

Saved alignment is reused by ordinary `launch --teleop`; repeat calibration only when the
controller/tool pairing changes (for example, a different grip or play-space orientation).
Do not start this command alongside an existing launch: stop the old one with Ctrl+C first.
The separate `align` command also accepts `--delay 20`, but still requires a manual restart.

### Align the controller with the gripper

This measures a fixed rotation for the **tool-frame** delta pipeline, not an absolute position
offset and not physical robot calibration. With simulation running at home, grip released,
and a fresh controller pose stream, open a second terminal with the same sourced environment:

```bash
export ROS_DOMAIN_ID=42
cd ~/Desktop/vive-focus-vision-ros2
python3 scripts/openarm_sim.py align
```

You have ten seconds to put your hand down beside your body. Point the controller direction
you intend as the tool tip (for example, its top) down, matching the robot's blue tool +Z.
Keep that orientation, including wrist twist, steady for one second. The script reads both
orientations in the shared VR world and calculates `R_tc = R_controller^-1 R_tool`.
The existing teleop mapping then uses `Delta_tool = R_tc^-1 Delta_controller R_tc` for both
translation and rotation. It does not change the generic `teleop_node` or replace its tool-frame
mapping with a world-frame command.

The script cannot detect which physical end of the controller you mean by its tip: the held
pose defines the pairing. It requires a released clutch and fresh, steady orientation samples.
For the left hand, run the same calibration with `--arm left`:

```bash
python3 scripts/openarm_sim.py launch --teleop --align --arm left --delay 10
```

Keep the left grip released and hold the left controller in your comfortable hanging-gripper
orientation. It saves `controller_alignment_left.json` and `openarm.left.*` calibration fields;
your existing right-hand calibration remains unchanged. Both arms are blocked during the
calibration launch. Without left calibration, the right arm still works and the left waits.
Run `python3 scripts/openarm_sim.py configure` after upgrading to add the left controller configuration (saved calibrations
are retained). Rebuild ROS and the APK for this update.

It writes `controller_alignment.json` and updates `teleop.yaml`; `prepare` retains this saved
alignment. **Stop and restart `launch --teleop` to apply it.** If you change how you grip the
controller or the intended tip direction, repeat alignment. Begin with a small lift, then test
individual wrist rotations; a full imitation of human arm posture is not implied by TCP IK.

To return both arms to their home pose while the simulation remains running:

```bash
python3 scripts/openarm_sim.py reset
```

Release grip before reset. The service resets simulated joint state and targets immediately;
the affected arm cannot resume until it sees release then re-grip. This is simulation-only behavior.

Defaults under `vive_scene.ros__parameters` (override in `teleop.yaml`):

| Parameter | Default | Meaning |
|---|---:|---|
| `openarm.translation_scale` | 0.25 | Robot displacement per metre of controller displacement. |
| `openarm.max_translation_m` | 0.15 | Maximum target displacement per grip press. |
| `openarm.joint_speed_rad_s` | 1.5 | Maximum commanded joint velocity, not a certified physical velocity limit. |
| `openarm.max_linear_speed_m_s` | 0.5 | Maximum commanded tool linear speed. |
| `openarm.max_angular_speed_rad_s` | 2.0 | Maximum commanded tool angular speed. |
| `openarm.tracking_time_constant_s` | 0.12 | Time constant with which the tool closes its pose error; smaller is tighter and twitchier. |
| `openarm.damping` | 0.05 | Least-squares damping, phased in only as a singularity is approached. |
| `openarm.posture_gain_hz` | 0.5 | How hard the spare degree of freedom drifts the elbow back towards mid-range. |
| `openarm.alignment_tolerance_m` | 0.15 | Optional proximity threshold, used only with `openarm.require_position_alignment=true`. |
| `openarm.finger_speed_m_s` | 0.02 | Maximum rate of finger-target changes. |
| `openarm.timeout_s` | 0.25 | Maximum time without a delta arriving before holding and requiring re-grip. |

The control values ship in `config/vive_vr.yaml`, so `prepare` copies them into the generated
`teleop.yaml` and a run is retuned there without rebuilding.

Rotation is configurable with `openarm.max_rotation_rad`, default 1.75 radians (100 degrees),
so a 90-degree forward turn no longer trips the former 34-degree cutoff. The generated
`teleop.yaml` uses translation scale 1.0 and a 1.0 m displacement bound per press; the
class defaults in the table remain conservative for custom configurations. Robot reach and
joint limits still apply: a long human arm movement can request an unreachable TCP pose.
All seven joints of each arm participate in end-effector IK; human elbow posture is not measured.

Side-grip engagement requires fresh controller tracking and matching wrist orientation within
`openarm.alignment_tolerance_rad` (0.20 rad, about 11 degrees), using the saved pairing.
Stand comfortably: your hand does not need to overlap the virtual gripper or reach the floor.
Pressing side grip anchors the current robot tool pose; subsequent hand displacement drives
relative tool motion. Releasing and pressing again starts a new reference without a jump.
Optional `openarm.require_position_alignment=true` restores the proximity gate, using
`openarm.alignment_tolerance_m` (0.15 m). The default is false.
`openarm.require_alignment=false` bypasses alignment for automated delta-only tests.

The bridge publishes `/vive_vr/teleop/right/status` (`vive_vr_ros2/TeleopStatus`): a `state` of
`ready`, `align_pose`, `calibration_required`, `tracking_lost`, `release_grip`, `waiting_delta`,
`engaged` or `unreachable`, a `ready` flag, and both errors with the tolerances they are judged
against. `VrTeleopIndicator` in the client tints the gripper from red through amber to green
from that message. Green is the PC's `ready` flag rather than a threshold held on the headset,
so green always means the press will take; only the shade between is computed on the client.
During normal use the headset shows **only hand indicators**. A recovery panel appears only
after a forced stop or when the target is out of reach, naming the affected hand and explaining
how to continue. It guides you through releasing grip, matching the hand outline and re-gripping,
then disappears once movement resumes. Each hand has
a white outline for its current controller orientation and a cyan outline for the required
orientation; the target turns green when ready. The target uses a single crisp, rounded outline without a translucent shadow.
A separate ring above each hand gently pulses and fills as alignment improves; it becomes
steady and completes only when the PC reports ready. Rotate white into cyan, then press that grip.
The target is computed from that arm's current TCP and saved controller/tool pairing and is
published on `/vive_vr/teleop/{left,right}/alignment_pose` as `geometry_msgs/PoseStamped`.
Guides appear automatically in existing scenes. Neither hand needs to reach the robot.
Stale outlines and readiness are hidden after disconnect or 0.5 s without fresh data.
Position tolerance zero in the status means no proximity check; raw distance is diagnostic.

A forced stop now includes `stop_reason` in the status: `translation_limit`, `rotation_limit`,
`tracking_timeout`, `invalid_target`, or `reference_mismatch`. The temporary recovery panel translates this into a short cause and next action.
For generated configurations, bounds per press are 1 m and 1.75 rad (about 100 degrees).
Release grip, align the wrist, and press again to establish a new reference. Robot reach/joint
limits instead report `unreachable`; the servo continues pursuing the closest reachable pose.
Tracking freshness uses steady-clock message arrival, so headset timestamp corrections do not
open the clutch. A real pose dropout still stops motion and requires a physical release/re-press.

This change requires rebuilding ROS and the APK, then restarting the simulation.

Run `configure` and restart teleop after upgrading, preserving your saved alignments;
`align` also writes the pairing to the simulation configuration. The combined
`launch --teleop --align` workflow blocks robot engagement during calibration.

The first delta must be
near identity (within 2.5 cm of scaled translation and 0.15 radians). Oversized or invalid
targets disengage the bridge. Release and re-grip after a timeout or bound violation. Holding
captures current joint positions; physical inertia can still cause a small settling movement.

Freshness is when a message **arrived**, never the stamp it carries: those come from the
headset's clock estimate, which steps when re-fitted, and gating on them silently dropped every
delta until the next step ([Known limits](limits.md)). A gap in arrivals longer than
`openarm.timeout_s` still holds the arm.

The bridge is a resolved-rate servo, not a per-tick pose solve. It owns the commanded
configuration, seeded from the measured joints at grip press, and each tick moves it down the
pose error: the Jacobian from KDL's `ChainJntToJacSolver` over the chain `init_robot_from_mjcf`
builds to the selected `openarm_{left,right}_hand_tcp`, and the joint velocity from a damped least-squares inverse
damped only near a singularity (Nakamura and Hanafusa 1986; Chiaverini, Oriolo and Walker 1994),
with the spare seventh joint pulling the arm to mid-range through the null space (Liégeois 1977).

Integrating rather than re-solving keeps one IK branch per press and respects the velocity and
joint bounds by construction. Beyond its reach the arm tracks the closest pose it can hold;
`unreachable` appears only once the gap stops closing for half a second, so slewing towards a
distant but legal target still reads `engaged`.
Feedback is `/vive_vr/sim/right/ee_pose` (`geometry_msgs/PoseStamped`, MuJoCo world frame).
The scene's reset service resets home and requires release/re-grip before motion resumes.

Launch uses the separate `openarm_sim.launch.py`, which includes the existing tracking launch
and rosbridge. The generic `sim.launch.py`, SceneNode and TeleopNode remain unchanged. It uses the current
`ROS_DOMAIN_ID`, or 42 if unset. Set the same domain in all your ROS terminals. Domain selection
does not control which PC the headset app connects to; its connection must still point to your PC.
Fully restart the headset app after changing geometry so it fetches the new export.

For a different scene-server address:

```bash
python3 scripts/openarm_sim.py launch --host-ip 10.23.1.153
```

Use your current laptop LAN address, not necessarily the example above. This sets the geometry
server URL; it does not change the headset's rosbridge host configuration.

To re-export after manually editing the controlled model, without regenerating it:

```bash
python3 scripts/openarm_sim.py export
```

Other locations are supported with `--output`, `--description` (OpenArm description package
directory), and `--library` (MuJoCo shared library). Pass the same `--output` to every command.
`prepare --skip-export` prepares and validates the models without exporting the GLB.

## What the files mean

| File | Purpose |
|---|---|
| Original `openarm_v10.urdf.xacro` | Robot-description template in `openarm_description`; unchanged. |
| `openarm_v1.urdf` | Template expanded into links, joints, masses, limits and mesh references for both arms. |
| `openarm_v1_mujoco.urdf` | Import copy with absolute mesh paths, coloured visual parts, and separate collision shapes. Preserves fixed bodies and allows balancing inconsistent inertia values. |
| `openarm_v1.xml` | Converted MuJoCo model with corrected mesh references; no position motors. |
| `openarm_v1_motion_test.xml` | Copy with gravity and contacts disabled for initial motion checks. |
| `openarm_v1_controlled.xml` | Motion-test copy with 14 arm and 4 finger position motors, damping and two finger-coupling constraints. |
| `preview.yaml` | Copy of this project's ROS configuration with zero gravity, object grabbing disabled and a 0.002-second physics step. |
| `teleop.yaml` | Same configuration with the independent left/right simulation bridges enabled. |
| `controller_alignment.json` | Right controller/tool rotation, retained when regenerating models. |
| `controller_alignment_left.json` | Independent left controller/tool rotation, also retained. |
| `vr_scene_controlled/scene.glb` | Robot geometry displayed by the headset. |
| `vr_scene_controlled/manifest.json` | Mapping between the exported geometry and MuJoCo body IDs. |

**URDF** is ROS's robot-description format. **MJCF** is MuJoCo's XML model format.
The OpenArm description stores its visual colours in COLLADA (`.dae`) files. MuJoCo cannot
load those directly. `prepare` uses `scripts/openarm_visuals.py` (requires `python3-numpy`) to
convert each authored material part into an OBJ mesh plus a URDF diffuse colour. Dense visual
parts are clustered at 1 mm resolution for headset rendering; collisions and robot inertias
are unchanged. Generated OBJ files live in `visual_meshes/` beside the generated model.
The exporter includes visual group 1 only, keeping the collision hulls out of the VR view.
Unsupported COLLADA primitives, texture materials or coordinate conventions fail explicitly.

By default, `prepare` applies an approximate black-and-silver V1 hardware finish: dark arm
housings and brackets, with silver stand hardware and gripper details. This is a display
preset, not measured material data. Use `prepare --appearance cad` to retain the original
pale CAD colours. Neither option modifies the source OpenArm description.

After updating from an older plain-grey collision-only model, run `prepare` and restart the
simulation and headset app. The scene geometry changes, so `configure` alone is insufficient;
no APK rebuild is required. Both saved controller calibrations are retained.

A **mesh** is a 3D surface. Collision meshes are simpler than the visual meshes used in RViz,
so this headset model is grey and has simplified details.

The conversion reused an unmirrored finger mesh for `openarm_right_right_finger_collision`.
The script compares every imported collision mesh's file and scale with the source URDF and
repairs mismatches, rather than assuming generated names such as `finger1` always stay the same.

**Inertia** describes resistance to rotation. `balanceinertia` approximates inconsistent values
so MuJoCo can compile the model; it is not a measurement of the real robot's dynamics.

A **position actuator** is a simulated motor trying to reach a joint target. `kp` sets position
correction strength; `kv` and joint damping resist motion. The starting values are only for this
zero-gravity simulation, not physical hardware. Targets start at the selected home pose. The original joint limits
and imported actuator-force limits are retained. The two **mimic constraints** keep each gripper's
fingers moving together, as specified in the URDF.

## Verification and next work

The preparation command compiles the corrected model and the controlled model to detect invalid
MJCF, checks that both TCP bodies remain, and expects 18 movable joints. TCP means the tool
reference point at each gripper. Export loads the controlled model again.

During development, the full pipeline was run in a temporary directory: 26 bodies and 23
geometries exported. A headless simulation stayed exactly at rest for five simulated seconds;
the right elbow then reached a 0.15-radian position target without simulation warnings.
This is a basic check, not validation across all poses or gains.

`tests/openarm_ik_check.cpp` checks convergence on a reachable pose, MuJoCo and KDL agreeing on
it, actuator tracking, the per-tick velocity and limit bounds, null-space centring without
disturbing the tool, the home singularity, and out-of-reach settling.

The C++ checks run against `tests/openarm_test_arm.xml`, two seven-joint arms with OpenArm
left/right names, so `colcon test` needs no OpenArm description; set `OPENARM_TEST_MODEL` to a
generated model to run them against the real robot too.
`tests/openarm_alignment_check.py` checks the measured
frame rotation, translation and rotation direction preservation, and RPY conversion.
`tests/openarm_bridge_check.cpp` sends ROS messages through the bridge and checks grip gating,
motor target rate limits, right motion, gripper opening, left hold, release, re-clutch, timeout,
invalid and oversized input, alignment readiness, 90-degree rotation bounds and reset behavior.
It also covers the freeze directly: deltas carrying stamps that are seconds old, and stamps that
step backwards, must still be followed.
`tests/openarm_alignment_capture_check.py` checks calibration with ROS pose streams;
`tests/openarm_launch_check.py` checks calibration-process cleanup on success, failure,
cancellation and shutdown timeout. To reproduce on an isolated local ROS domain:

```bash
cd ~/vive_vr_ws
colcon build --packages-select vive_vr_ros2 --cmake-args -DBUILD_TESTING=ON \
  -DOPENARM_TEST_MODEL=$HOME/vive_vr_ws/models/openarm_v1/openarm_v1_controlled.xml
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ctest --test-dir build/vive_vr_ros2 --output-on-failure
```

Controller/tool axis alignment still needs an actual headset movement test. Left-arm teleop,
contact geometry, grasping and physical hardware control are not implemented by this bridge.
