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
0.05 rad/s, and verifies measured positions settle within 1 degree of zero.

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
completion means measured angles within 1 degree, not an
encoder recalibration or a claim of mechanically exact zero.

## Physical gripper controls

The real launch enables gripper buttons by default; simulation controls are unchanged.
Install the updated headset APK. Tap the left menu button briefly (under 0.8 s)
to toggle WORLD LOCKED; holding it retains the existing hand/controller-mode switch.
Lock freezes joystick translation, turning, height buttons and recentering, while
normal head tracking continues. Its label stays at the location where lock was toggled.
Lock before enabling real VR. Unlocking disarms real control; center both sticks,
lock again and explicitly enable to resume. Old APKs cannot supply the lock flag.
Each hand's stick up opens its gripper, down closes it, and center stops/holds.
A 0.2 neutral deadzone and 0.5 engagement threshold reject small accidental deflections.
Rear triggers and A/X no longer control physical grippers. After enabling or input
loss, center the stick before issuing another command.
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
requires measured arm error within 1 degree and finger error within 1 mm for 0.5 s.
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
again to engage. A new engagement captures a fresh hand reference. Gripper sticks operate independently of the arm toggle.

Left gripper gains are now `kp=15`, `kd=0.20` to test reduction of the measured
+0.75 mm residual; right remains `kp=10`, `kd=0.15`. The left closing force increases,
so first test with empty fingers. No negative closing target or extra torque bias
is introduced. Motor readback, not fingertip contact, determines reported completion.
