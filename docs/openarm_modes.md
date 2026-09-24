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
