# OpenArm V1 live VR test {#page_openarm_hardware}

For the separate simulation/physical entry points and source layout, see
[OpenArm modes](openarm_modes.md). The older live launch below remains compatible.

The repo now includes a guarded CAN driver and a live VR command bridge. Preview
remains the default. Live output starts **disarmed** and requires the enable service
plus a released/re-pressed, aligned controller grip.

## This workstation

ROS 2 Jazzy; right arm on `can0`, left on `can1`; both at 1 Mbps arbitration / 5 Mbps
CAN-FD data. On 2026-09-22 all 16 motor IDs replied, and the user confirmed that the
physical zero posture matched feedback and both arms settled normally. No calibration
or motor parameters were changed. Existing controller/tool pairing files are reused.

An earlier guarded-driver run held the measured posture for more than five minutes
without a latched fault. A 10-second sample received 997 complete joint messages,
with a maximum arrival gap of 26 ms, both health streams true, and all five controllers
active. Both CAN interfaces reported zero bus errors. That run subsequently faulted
on both arms with `errno=105` (`ENOBUFS`, transmit buffer exhaustion), and the bridge
disarmed. The driver now spaces individual position frames by at least 250 microseconds,
aborts a batch at its first failed send or a 30-ms deadline, and never retries a
failed batch. Fault handling adds no further CAN traffic. Redundant state requests
were also removed (position commands already solicit replies). Queue sizes, motor
parameters, and feedback fault thresholds were not relaxed.

After installing the paced sender, an eight-minute feedback observation received
47,989 complete joint messages with zero unhealthy reports from either arm and a
maximum arrival gap of 31 ms. No driver fault appeared during the run. Its one-shot
controller service query timed out; a fresh five-second check then confirmed all five
controllers active, both guards healthy, and 16 joints reporting. The three targeted
pacing/live-bridge/preview tests passed, including send failure and delayed-wakeup
cases. The driver is left holding measured posture with VR disarmed. This validates
the observed hold run, not physical VR motion or every possible USB scheduling delay.

The same run later latched a write-cycle stall over 100 ms on both arms, with all CAN
error counters zero. Linux had denied the controller's FIFO scheduling request
(`Operation not permitted`, realtime-priority limit 0). The real-hardware launch now
requires a realtime-priority allowance of at least 50 before touching the motors.
Scheduling permission is necessary for this launch; it does not guarantee deadlines.

A subsequent FIFO-50 run still stalled: the controller measured 317 ms in its read
phase. The driver had been publishing reliable ROS health messages synchronously in
that phase, exposing motor updates to DDS backpressure. Health publication now runs
on a separate executor thread; the control loop only stores atomic status/timestamps.
The publisher reports unhealthy if the last validated read is over 100 ms old, so a
stalled loop cannot keep a healthy heartbeat alive. This removes a blocking path;
the revised driver still needs physical runtime validation.

## Build

```zsh
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
cd ~/vive_vr_ws
colcon build --packages-select vive_vr_ros2 --cmake-args \
  -DBUILD_SCENES=OFF -DBUILD_OPENARM_SIM=ON -DBUILD_OPENARM_VR_BRIDGE=ON -DBUILD_OPENARM_HARDWARE=ON
source ~/vive_vr_ws/install/setup.zsh
```

The optional hardware plugin requires `hardware_interface`, `pluginlib`, and the
OpenArm CAN library with `MotorLinkStats` and bus-health APIs. It is built **in this
repository**, without modifying the upstream OpenArm workspace. The launch replaces
only the real-hardware plugin in the generated description with
`vive_vr_ros2/GuardedOpenArm`; it does not edit source descriptions.

## Start the physical test

Stop the old physical driver and VR preview/simulation first. Never run two drivers
on the same CAN ports. Ordinary driver shutdown disables torque; support the arms if
needed. Keep the physical emergency stop available and the movement area clear.

After rebooting or reconnecting the USB CAN adapter, configure its interfaces while
the driver is stopped. Powering the robot alone does not bring up Linux CAN:

```zsh
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up
```

```zsh
sudo prlimit --pid $$ --rtprio=50:50
ulimit -r # must print 50 (or higher)
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
export ROS_DOMAIN_ID=84
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch vive_vr_ros2 openarm_live.launch.py \
  use_fake_hardware:=false host_ip:=10.23.1.153
```

Run these in the same terminal. `prlimit` changes only that terminal's scheduling
allowance and the allowance inherited by processes it starts; enter the sudo password
locally. ROS itself still runs as your normal user. A new terminal needs this command
again. Confirm the launch no longer reports `Could not enable FIFO RT scheduling`.
An account-wide persistent setup is described in the
[ROS controller manager documentation](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html).

Use the PC's current LAN address if it changes. This serves the scene to the headset
and starts tracking, the hardware driver, and the bridge. The guarded driver captures
fresh measured joints and **holds that pose on activation**; it does not home or zero
the robot. The `model_dir` argument defaults to `~/vive_vr_ws/models/openarm_v1`.

In another terminal with the same sourced environment and domain, arm VR output:

```zsh
ros2 service call /openarm_hardware_preview/enable std_srvs/srv/SetBool '{data: true}'
```

The service name is shared with preview mode. It refuses arming without fresh complete
joint feedback, both guarded-driver health streams, and all four active trajectory
controllers. Release both grips, match the orientation guides, then hold one grip and
make a small movement. Start with one arm. Releasing grip holds its last bounded target.
Fresh joystick samples continuously confirm an already-released grip: a controller
resting with its grip released does not need an extra squeeze/release after arming.
Held grips remain edge-triggered and cannot automatically re-engage after a reset.
The grippers remain held; trigger control is off for the first test. To enable trigger
control on a later run, add `control_grippers:=true` (the V1 motor-to-finger mapping is
approximate and needs physical validation).

Disable VR commands while retaining the last bounded targets:

```zsh
ros2 service call /openarm_hardware_preview/enable std_srvs/srv/SetBool '{data: false}'
```

This service is **not an emergency stop**. It depends on ROS and a running process.

## Guards and timing

- Hardware loop: 100 Hz, non-blocking receive. A write-cycle gap over 100 ms latches a fault.
- Position frames are paced, with an 8-ms maximum batch duration per arm. Scheduling
  delays cannot produce a catch-up burst; a missed batch deadline latches a fault.
- Every motor must have replied within 100 ms. Missing replies, malformed packets, motor
  error/disabled status during operation, nonfinite/out-of-range states, and CAN bus or
  transmit errors fault the affected hardware component. It cannot automatically resume.
  Fault logs include feedback ages for all eight motors on that arm. A motor feedback
  timeout is separate from the VR controller tracking timeout; increasing hand-motion
  range does not fix missing motor replies. Green alignment guides appear only while
  ready/alignment matching, so they hide during an engaged grip or a hardware disarm.
- Position-only hardware commands are finite, within the model envelope (small measured
  startup tolerance allowed), within 0.25 rad of feedback, and rate limited to 0.30 rad/s.
  Gripper equivalents are 0.01 m following error and 0.005 m/s.
- VR targets run at 50 Hz: translation scale 1.0, at most 50 cm / 180 degrees per grip,
  tool speed 0.10 m/s / 0.5 rad/s, no null-space posture drift. Larger hand offsets cap
  the target at this envelope instead of dropping the grip. Moving back within the
  envelope continues the same grip. The first-delta reference check remains active.
  Hand translation now requests equal tool displacement (10 cm hand = 10 cm target),
  relative to the pose captured at grip engagement, within the reachable workspace.
  Tracking time constant is 0.15 s; the physical speed caps still limit how quickly
  the robot can follow a fast hand. This profile does not promise one-to-one speed.
  The bridge checks following error (0.20 rad / 0.008 m) and limits each command increment.
- With gripper control disabled, trigger-derived gripper targets are excluded from
  arm command validation; they cannot falsely trip the arm's following-error guard.
- Arm and gripper trajectory outputs are separate. Controller trajectories span 40 ms;
  `interpolate_from_desired_state=true` continues the streamed trajectory from the
  previous command rather than restarting it from lagging feedback every 20 ms.
  Both driver and bridge continue to validate commands against actual feedback.
  `cmd_timeout=0.2` aborts stale input 200 ms after the final point. A producer crash can
  finish only that short bounded target before the controller holds.
- Missing/invalid joint feedback, lost health, or an inactive controller disarms VR.
  Recovery requires explicit enable and a new grip cycle. Controller tracking loss
  uses the existing 250 ms timeout and requires releasing/re-pressing grip.

On a hardware fault the guarded plugin retains its last bounded targets where CAN is
still available, reports ERROR and publishes unhealthy. It deliberately does not drop
all motor torque automatically. A disconnected bus cannot carry a stop command: use
the physical E-stop. Motor watchdog registers remain unchanged. PC/power failure and
physical fault-stop behavior need supervised hardware validation. This is not a
certified safety controller, and collision avoidance is not implemented.

The rate reduction removes the previous 750 Hz requirement; it does not make Linux
real-time. Scheduling warnings may still occur under load. Fault handling bounds a
long stall, rather than assuming every deadline is met. Do not build large projects
while testing arm movement.

## Preview and mock tests

`openarm_hardware_preview.launch.py` without `enable_commands:=true` publishes only
`/vive_vr/hardware_preview/{right,left}/joint_trajectory`. Never remap these eight-joint
preview messages to the seven-joint physical controllers. The VR scene always follows
measured feedback using kinematics, not simulated physics.

`openarm_hardware.launch.py` defaults to mock hardware. Mock hardware intentionally
supplies no guarded-driver health reports, so a normal live bridge cannot arm against
it. The isolated automated test supplies synthetic health and controller services.

All 12 core CTest checks passed, including live arming, feedback/health/controller/tracking
loss, re-engagement, arm isolation, gripper gating, rate limiting, and prior simulation
regressions. The workstation's extended suite passed 14 of 15 checks: its additional
V1-model IK convergence check fails the 1-mm position threshold, including when run
against the current deployment model. The boundary assertion now accounts for the
existing 0.01-rad joint-stop margin; convergence remains unresolved. Treat physical
testing as experimental, and stop if tool tracking is incorrect.
A separate mock-controller test executed a 0.01-rad trajectory, observed
the command-timeout warning, and verified a stable hold. These do not substitute for
physical cable-disconnect or E-stop tests.

After the requested range/speed increase, all 12 core checks passed again, including
large-offset target capping without losing the grip and returning within the same
grip. The mock-controller check also streamed 30 updates at 50 Hz with 40-ms horizons,
verified movement, and verified hold after the stream ended. The faster profile and
larger following-error envelope still require supervised physical validation.

References: [OpenArm setup](https://docs.openarm.dev/software/setup/motor-config/) and
[Jazzy controller timeout parameters](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html).

### Capture intermittent feedback loss

The command loop stays at 100 Hz. The guarded driver additionally drains received
frames after each paced motor send, without extra polls or a higher transmit rate.
Fault snapshots include command, reply, rejected-command and malformed-frame counts.
Do not interpret a successful socket write as proof a motor received the command.

For a receive-only capture, start this in a separate terminal before the real launch:

```zsh
python3 ~/Desktop/vive-focus-vision-ros2/scripts/openarm_can_watch.py --seconds 90
```

It records per-motor command/reply counts and reply gaps to
`/tmp/openarm_can_watch.jsonl`, and reports commands observed without a recent reply.
It opens raw CAN sockets to listen; it never transmits, changes bitrate, resets a
motor, or enables torque. A capture after a latched fault cannot reconstruct the
missing replies before that fault, so capture across a fresh startup and failure.

### Small elbow offsets at startup

For the guarded real driver only, the ROS controller description accepts an elbow
lower-bound offset down to -0.02 rad. The driver receives the original nominal
limits (elbow minimum 0) before that controller description is adjusted. At
activation it holds the measured pose. An outside command may hold or move back
inward, never farther outside; measured drift cannot expand that command range.
Once the sent target is inside, the original limits apply again. Simulation and
fake-hardware descriptions retain their existing limits. This avoids a controller
startup failure for the observed -0.60-degree offset without changing motor zero.

Use `openarm_return_to_zero.py --status` to read angles, or `--execute` to request
a slow return after startup. Larger elbow offsets and missing feedback are still
rejected. Zero is verified with a 1-degree measured-position tolerance.

For the guarded real grippers, the ROS controller description also accepts a
closed-position readback down to -0.0002 m (-0.2 mm). This covers the observed
-0.136 mm hold without repeated limit-clamping messages. The driver retains its
original 0..0.044 m limits: an existing negative startup target may hold or return
inward, but new targets cannot close farther outside that position. The opening
limit, calibration, motor gains and fake/simulation limits are unchanged.
