# Simulation and physical robot {#page_openarm_modes}

Both modes use the same headset app and controller alignment. They have separate
application code, settings and ROS domains. The existing low-level launch files
remain available for compatibility; use these entry points for normal operation.

| | VR simulation | Physical OpenArm |
|---|---|---|
| Launch | `openarm_virtual.launch.py` | `openarm_real.launch.py` |
| ROS domain | 83 | 84 |
| Application code | `src/openarm/simulation/` | `src/openarm/hardware/` |
| State source | MuJoCo physics | Actual motor feedback |
| Motion settings | `<model_dir>/teleop.yaml`, `vive_scene` block | `config/openarm_real.yaml` |
| CAN / controller manager | Never started | Guarded driver and trajectory controllers |
| Enable service | Not needed | Explicit enable after startup |

`src/openarm/common/` contains IK and clutch/alignment behavior shared by both modes.
It contains no physics stepping or CAN transport. Sharing these algorithms keeps the
controller gestures consistent. Simulation changes belong in its application or
configuration; physical safety and command routing belong in the hardware application.
MuJoCo is still used by the physical bridge for model geometry and forward kinematics,
not to simulate robot motion.

## Start either mode

In Zsh, source the installed workspaces:

```zsh
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
```

For **simulation**:

```zsh
ros2 launch vive_vr_ros2 openarm_virtual.launch.py host_ip:=10.23.1.153
```

For the **physical robot**, first stop the simulation launch with Ctrl+C, then:

After a reboot or USB CAN-adapter reconnect, configure both interfaces (right is
`can0`, left is `can1`). Do this with the robot driver stopped:

```zsh
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up
```

Then launch from the same sourced terminal:

```zsh
sudo prlimit --pid $$ --rtprio=50:50
ros2 launch vive_vr_ros2 openarm_real.launch.py host_ip:=10.23.1.153
```

Use the PC's current LAN address if it changed. Both launches default to the existing
`~/vive_vr_ws/models/openarm_v1` model directory and the headset's existing network ports.
No APK rebuild is required. The physical launch starts disarmed. In a second sourced
terminal, enable it with:

```zsh
ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ros2 service call /openarm_hardware_preview/enable std_srvs/srv/SetBool '{data: true}'
```

The old service/executable name containing `preview` is retained for compatibility.
It belongs only to the physical bridge; it is not a simulation service.

## Independent settings and builds

Edit a copy of `config/openarm_real.yaml` and pass it as `hardware_params_file:=/path/to/real.yaml`
to tune physical VR behavior within the driver's hard limits. The simulation accepts
`sim_params_file:=/path/to/sim.yaml` instead. The real bridge imports only controller
pairing values from the simulation's `vive_scene` block, never its speed, gravity or
motion parameters. Input calibration is shared by default and can be supplied
independently with `input_params_file:=/path/to/calibrated_inputs.yaml` for real mode.

Three independent CMake options control the build:

| Build | `BUILD_OPENARM_SIM` | `BUILD_OPENARM_VR_BRIDGE` | `BUILD_OPENARM_HARDWARE` |
|---|---|---|---|
| Both | ON | ON | ON |
| Simulation only | ON | OFF | OFF |
| Physical only | OFF | ON | ON |
| Hardware feedback preview without CAN plugin | OFF | ON | OFF |

Pass these via `colcon build --packages-select vive_vr_ros2 --cmake-args ...`.
Use a clean build/install directory when switching to an exclusive build: colcon
does not remove binaries installed by a previous configuration.

## Running both stacks concurrently

The default ports support switching modes with one headset. To run both stacks at once,
keep physical mode on its defaults and start simulation with separate ports:

```zsh
ros2 launch vive_vr_ros2 openarm_virtual.launch.py host_ip:=10.23.1.153 \
  rosbridge_port:=9092 http_port:=8001 discovery_port:=0
```

Their fixed domains remain 84 and 83. Physical mode retains automatic headset discovery;
connect a separate client manually to simulation on WebSocket port 9092. A single
headset connection controls one stack at a time. These entry points set the domain for
their child processes; separate terminal diagnostics must specify the desired domain.

See [physical operation and fault handling](openarm_hardware.md) and
[simulation preparation and calibration](openarm_sim.md) for details.

## Return physical arms to joint zero

Start the updated `openarm_real.launch.py` and leave it running. The return script
moves both arms to their existing joint coordinates `[0, 0, 0, 0, 0, 0, 0]`.
It does **not** calibrate encoders or save a new zero. Grippers stay unchanged.
Support/recover the robot separately if a driver fault is latched; this routine
requires fresh feedback and healthy active controllers and will refuse a faulted
robot or a starting joint position outside its limits, except an elbow up to
0.02 rad (1.15 degrees) below its zero lower boundary. That small offset can only
hold or move inward; it does not redefine calibration or extend normal VR targets.

Clear the direct joint-space path for both arms before executing. This routine
has no collision planning: it cannot check obstacles, the table, or arm-to-arm
clearance. It moves both arms together with a smooth start/stop, peak joint speed
0.05 rad/s, and verifies measured positions settle within 1.5 degrees of zero.

```zsh
source /opt/ros/jazzy/setup.zsh
python3 /home/lightonray/Desktop/vive-focus-vision-ros2/scripts/openarm_return_to_zero.py --execute
```

The client selects localhost ROS domain 84. The bridge disarms VR and rejects VR
enabling while the return runs. Keep the script open: Ctrl+C requests a stop;
if the script disappears, missing keepalive stops the return within 0.5 s.
Stopping retains the last bounded targets; it does not release motor torque.
A separate cancellation command is:

```zsh
python3 /home/lightonray/Desktop/vive-focus-vision-ros2/scripts/openarm_return_to_zero.py --cancel
```

Any feedback/controller fault or excessive following error stops the return and
requires a new explicit request after recovery. Completion leaves VR disarmed;
use the usual enable service and release both grips to return to teleoperation.

To read the measured posture without moving:

```zsh
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_return_to_zero.py --status
```

The script prints J1–J7 in degrees for each arm. During `--execute` it reports
both elbow angles and the largest remaining zero error once per second. Zero
completion means measured angles within 1.5 degrees, not an
encoder recalibration or a claim of mechanically exact zero.

## Physical gripper controls

The real launch enables gripper buttons by default; simulation controls are unchanged.
Install the updated headset APK. Tap the left menu button briefly (under 0.8 s)
to toggle WORLD LOCKED; holding it retains the existing hand/controller-mode switch.
Lock freezes joystick translation, turning, height buttons and recentering, while
normal head tracking continues. Its label stays at the location where lock was toggled.
Lock before enabling real VR. Unlocking disarms real control; center both sticks,
lock again and explicitly enable to resume. Old APKs cannot supply the lock flag.
Each hand's rear index trigger opens its gripper while held and closes it when released.
Side grips toggle arm following; A/X and sticks do not control physical grippers. After enabling or input
loss, release the trigger, then press it to open. The initial release holds position;
closing on release starts only after that fresh press. Tracking loss or unlocking
holds the gripper rather than treating missing input as a release.
For the real robot, press the side grip once to engage arm following and again to
stop; releasing it does not stop an engaged arm. Align before engaging. Simulation
retains hold-to-move.

Grippers ramp up to 15 mm/s with a 75 mm/s² acceleration limit and require fresh controller tracking and button data.
Tracking loss or disarming requires another release before movement can resume.
Return-to-zero leaves grippers unchanged unless `--close-grippers` is supplied. Use `control_grippers:=false` to disable
this feature. Restart the real launch after rebuilding; no APK rebuild is needed.

## Return to zero and close both grippers

Keep the real launch running, with empty grippers and a clear arm path:

```zsh
source /opt/ros/jazzy/setup.zsh
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_return_to_zero.py --execute --close-grippers
```

This uses the existing encoder calibration. Both arms follow a smooth joint-space
path to zero at up to 0.05 rad/s; grippers target zero at up to 10 mm/s. Completion
requires measured arm error within 1.5 degrees and finger error within 1 mm for 0.5 s.
Measured gripper positions are printed; completion does not certify mechanical contact.
Ctrl+C cancels, retaining motor torque. VR stays disarmed afterward. Omit
`--close-grippers` to preserve gripper positions. `--status --close-grippers` only
reads positions. No direct CAN or trajectory publication is needed from the terminal.

The real VR defaults are 0.6 rad/s joint speed, 0.20 m/s tool translation and
1.0 rad/s tool rotation. Feedback freshness, following-error limits and CAN fault
handling remain enforced. These caps are tuning values, not a certified safety rating.

The guarded driver's gripper position gains are `kp=10`, `kd=0.15` (previously
`5`, `0.1`), to test stronger position correction with additional damping against
the observed stop-start motion and near-zero residual. Closing force increases;
validate with empty grippers after restarting the real launch. Physical smoothness
and closure still require observation. Desired motor velocity and feed-forward
torque remain zero, including during motion, so a retained CAN command stays a
position hold. The closed target and completion tolerance are unchanged; a timeout
must not be treated as confirmed closure. Startup logs print the active gains.

Startup holds measured joint positions; you do not need to manually match zero.
The real controller permits an elbow readback down to -0.05 rad and finger readback
down to -0.002 m, matching the driver's existing feedback acceptance. These are
recovery allowances, not new movement ranges: the driver allows an already-outside
target to hold or move inward, never farther outward or back outside after recovery.
Return-to-zero accepts that elbow startup offset. If readings exceed these allowances
or the calibration is wrong, diagnose the physical position/calibration rather than
repeatedly widening limits or resetting zero at an arbitrary pose.

The real input node uses `teleop.toggle_clutch=true`. Pose or button-stream loss
drops the toggle. Arming/disarming and homing reset it; release the button and press
again to engage. A new engagement captures a fresh hand reference. Gripper triggers operate independently of the arm toggle.

Left gripper gains are now `kp=15`, `kd=0.20` to test reduction of the measured
+0.75 mm residual; right remains `kp=10`, `kd=0.15`. The left closing force increases,
so first test with empty fingers. No negative closing target or extra torque bias
is introduced. Motor readback, not fingertip contact, determines reported completion.

The progress advisory is rebased when the end-effector target moves by 5 mm or
0.02 rad. It does not compare a pickup movement against the best error at an old
target. An advisory alone does not show the large recovery panel or disengage
following; actual stops still show recovery instructions. Joint, speed and feedback
limits remain unchanged.

### Real-robot VR orientation

The real launch uses the robot operator view. Its saved viewing angle is applied
around the OpenArm base at ROS world zero. Both the headset rig position and
heading orbit together, so the displayed base stays in place and the robot turns
about it. Published controller poses use that same rig transform; robot geometry,
end-effector guides, and controller-to-tool alignment remain in one ROS frame.
This does not move the physical robot or change motor calibration.

### Adjusting the real robot viewing direction

While the world is unlocked in the real robot operator view, release and press
the left rear trigger to start turning the displayed robot right about its base
at 15 degrees per second. Release and press again to stop at any angle. Tap the
left menu button to stop, lock the world, and save the angle on the headset; it
is restored on the next app launch. Losing controller tracking also stops the
adjustment. This changes the VR viewing frame, not the physical robot pose or
calibration. Head tracking stays active. While locked, the rear trigger retains
its gripper function.

While unlocked, hold the right rear trigger and move the controller to translate
the robot view one-to-one in any direction. Release to leave it there; repeat
to move farther. This stops automatic rotation. Locking saves the position and
angle. Tracking loss stops placement and requires a release before resuming.
Only the VR placement changes, never the physical base or motor calibration.

For a desk-mounted headset in the real operator view, the right stick moves the
viewpoint while unlocked: up/down moves forward/backward in the horizontal view
direction, left/right strafes. Maximum speed is 0.5 m/s. Center the stick after
unlocking before moving. Lock saves the viewpoint and disables stick locomotion.
Complete view placement before wrist calibration; keep teleoperation disarmed
during calibration and restart the robot launch after saving the new pairing.

Camera panel distance is fixed; the left stick no longer zooms the panels. In
the unlocked real operator view, left-stick horizontal input turns the viewpoint
left/right around the eyes at up to 45 degrees per second. The right stick walks
forward/back and strafes. Center sticks after unlocking. Lock stops navigation
and saves the view; normal head tracking remains active.

Left-stick up/down also tilts the view up/down (limited to 70 degrees each way).
While unlocked, hold right-controller A to raise the viewpoint or B to lower it
at 0.3 m/s. Release both after unlocking before use. Lock saves pitch and height
as well as position/yaw. A/B retain their recording functions while locked.

### Facing a desk-mounted headset

In the real operator view, unlock the world and orient the view behind the robot
first. With both controllers tracked, release then press **left X** to enable
**facing-headset** control. The unlocked status panel shows ON/OFF. This rotates
both controller poses 180 degrees about their captured midpoint, preserving
right/left controller identities. The operator frame is level and stays fixed
in robot/world coordinates while you subsequently move the camera to a side view.
Controller meshes, pointing rays and ROS controller poses share this transform;
head tracking is unchanged. Press X again while unlocked to restore normal mapping.
The mode starts OFF each app session; it cannot change while the world is locked.

After choosing the operator frame, position the viewing camera, lock the world,
and recalibrate wrist alignment for this mapping before engaging. Check small
right/forward/up movements in simulation or with control disarmed first. Switching
mode can visibly reposition controllers, so do it only during setup. X retains its
recording/discard behavior while locked. This mode applies to tracked controllers,
not bare-hand tracking.

View navigation eases into stick movement (look: 35 degrees/s maximum with a
120 ms response; walk: 0.5 m/s with a 100 ms response). Centering a stick stops
that navigation immediately. Right-stick walking overrides right-trigger dragging;
release the trigger before starting another drag. Looking vertically does not
block walking: it falls back to the saved view yaw.

Startup failures now explain whether a motor did not reply, reported a fault, or
returned a position outside the accepted startup range. Joint angles are shown
in degrees and gripper positions in millimetres. A position-range failure does
not imply a CAN wiring fault. `openarm_return_to_zero.py --execute` does not send
a return request when live joint positions are unavailable. Check the first
`STARTUP BLOCKED` or `LATCHED FAULT` message in the launch terminal; subsequent
controller errors can be consequences of that failure. These diagnostics do not
expand recovery limits, reset calibration, or automatically enable a faulted arm.

### Retry a blocked startup without restarting the whole launch

The controller manager stays available when initial hardware activation fails.
A rejected configuration remains unconfigured and reports a readable diagnostic
on `/vive_vr/hardware/right/diagnostic` (and `/left/diagnostic`).

Read the current state without enabling anything:

```bash
source /opt/ros/jazzy/setup.zsh
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_retry_startup.py
```

After resolving the reported cause, support the arms and keep the workspace clear:

```bash
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_retry_startup.py --execute
```

Retry first disarms VR, configures both unconfigured arms, then activates them at
freshly measured positions and starts their controllers. Enabling motors applies
holding torque. It does not home, change calibration, or widen joint limits.
If a check fails, it prints the latest driver explanation and stops. A timeout
means the resulting state is uncertain; inspect the launch before another retry.
Running-motion faults remain latched and require a restart after repair. An arm
already activated before a later failure can remain holding; the script does not
release torque automatically. VR must be explicitly enabled after success.

### Local return-to-zero without the headset or an IP address

Stop the existing robot launch first. With CAN configured and the realtime
allowance set, source the ROS and workspace environments, then run:

```bash
ros2 launch vive_vr_ros2 openarm_home.launch.py
```

This enables healthy hardware at its measured pose and provides the return
services without rosbridge, discovery, HTTP cameras or headset tracking. VR
arming is explicitly blocked in this mode. In another sourced terminal:

```bash
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_return_to_zero.py --execute
```

The path must be clear: this is still a direct, non-collision-planned return.
Startup position/feedback checks still apply; this mode does not bypass a joint
outside its accepted range. Support the arms before stopping the local launch;
ordinary shutdown can release torque. Stop it before starting the real VR launch.

Startup retry refuses mixed active/unconfigured arms: configuration can block
the controller manager long enough to starve the already-active arm's feedback.
In that case support the arms and restart after addressing the cause instead.

Local homing now accepts an elbow lower-bound feedback offset of at most 0.06 rad
(3.44 degrees), covering the observed -3.01 degree starting pose. Normal VR launch
retains the 0.05 rad allowance. This extra margin applies only to J4's lower bound;
other joints, upper limits and feedback freshness checks are unchanged. The driver
holds the measured pose first. The explicit return command uses the existing
0.05 rad/s trajectory and inward-only command corridor. Once commanded inside
the nominal range, commands cannot move back into the negative recovery corridor.
It does not change encoder calibration or provide collision avoidance. Do not use
recovery to force a joint against a mechanical stop. Positions below -3.44 degrees
still block startup and require diagnosis rather than successively larger margins.

### One-command local homing and hold

With CAN already up, stop other robot launches, source ROS and both workspace
setups, and grant realtime allowance in the same terminal. Run:

```bash
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_home.py --execute
```

Add `--close-grippers` only with empty, clear grippers. The supervisor starts the
local homing launch, waits for fresh feedback and all controllers, invokes the
existing return client, and leaves the hardware running after confirmed success.
It monitors feedback while holding and reports if holding is no longer confirmed.
No headset or IP is required. Full startup output is saved in the printed `/tmp`
log directory. Failed startup does not trigger a return or automatic retry; services
stay available until you support the arms and press Ctrl+C. Ctrl+C cancels any
ongoing return and stops the owned launch; ordinary shutdown may release torque.
The existing recovery envelope, motor fault guards and direct-path limitations apply.


### Desktop controls

Run `/usr/bin/python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_panel.py`.
The panel offers CAN setup, local homing/hold, joint and startup status, VR launch,
cameras/recording, and VR enable/disable. Each action opens a named terminal;
sudo passwords are entered there. The Wi-Fi IP is detected when the panel opens
and can be edited. CAN setup/homing/VR launch refuse an already-running controller.
Close the panel without affecting the terminals. Support the arms before using
Ctrl+C in a robot/home terminal, because stopping it can release holding torque.

Return completion now means all seven joints of both arms are within **1.5 degrees**
of stored zero. The target remains exactly zero; this accepts residual position
error rather than changing calibration or motor gains. The gripper tolerance is
unchanged. Progress identifies the joint with the largest error.

The desktop panel also provides first-time right/left wrist calibration (VR
is disarmed first), an explicit arms-and-grippers VR enable button, and live
recorder status. Start/stop/save/discard controls use the existing recorder API
and are enabled only for valid, recently received states. Saved count, frame count,
duration, review and export status also update when recording is controlled in VR.
A disconnected recorder shows an unknown state, never a false stopped/saved claim.
Calibration saves the controller-to-tool pairing; restart the VR robot launch to
load it. This is not motor encoder calibration.

Select a panel-owned command in the running list and press **Stop selected** to
send Ctrl+C to its process group. PID identity is checked before signalling;
unrelated/external commands are not killed. No force-kill is offered. Support arms
before stopping home/VR because torque may release. CAN setup still refuses an
active robot; select and stop the robot command first. Jobs launched outside this
panel must be stopped in their original terminals. Closing the panel leaves jobs
running, and reopening does not take ownership of old jobs.
